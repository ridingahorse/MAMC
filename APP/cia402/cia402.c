/**
  * @file    cia402.c
  * @brief   CiA 402 主站侧 —— 非阻塞初始化状态机实现
  *
  * 本模块只负责 CiA 402 专属部分：从站已进 OP 后，设模式 + 402 状态转换。
  * 通用 CANopen 初始化（等PRE-OP、配心跳、配PDO、发NMT进OP）由 canopen_init
  * 模块完成，本模块不关心。
  *
  * 流程：等OP → 设模式 → shutdown → switch_on → enable_op → RUNNING
  * 全程非阻塞，靠 cia402_process() 周期驱动。
  */
 #include "cia402.h"

#include "canopen_master.h"
#include "canopen_init.h"   /* co_pdo_config_t, co_init_restart, co_init_done */
#include "OD.h"              /* OD_RAM 变量 */
#include "301/CO_HBconsumer.h" /* CO_HBconsNode_t, CO_HBconsumer_getIdxByNodeId */
#include "main.h"         /* HAL_GetTick() */
#include "s_curve.h"         /* 软件 S 曲线轨迹规划 */
 
 #include <stdio.h>
 #include <string.h>
 
 /* ═══════════════════════════════════════════════════════════════
  *  私有全局
  * ═══════════════════════════════════════════════════════════════ */
 
 static CO_t* s_co = NULL;
static cia402_slave_t s_slaves[CIA402_MAX_SLAVES];
static const cia402_slave_config_t* s_cfgs[CIA402_MAX_SLAVES];

/* 根据 od_base_index 把 OD_RAM 变量指针填到 slave 里。
 * slave1: base=0x2000 → x2000~x2005
 * slave2: base=0x2010 → x2010~x2015 */
static void bind_od_ptrs(cia402_slave_t* s, uint16_t base)
{
    switch (base) {
    case 0x2000:
        s->ctrl       = &OD_RAM.x2000_controlword;
        s->target_pos = &OD_RAM.x2001_target_position;
        s->target_vel = &OD_RAM.x2002_target_velocity;
        s->status     = &OD_RAM.x2003_statusword;
        s->actual_pos = &OD_RAM.x2004_actual_position;
        s->actual_vel = &OD_RAM.x2005_actual_velocity;
        break;
    case 0x2010:
        s->ctrl       = &OD_RAM.x2010_controlword;
        s->target_pos = &OD_RAM.x2011_target_position;
        s->target_vel = &OD_RAM.x2012_target_velocity;
        s->status     = &OD_RAM.x2013_statusword;
        s->actual_pos = &OD_RAM.x2014_actual_position;
        s->actual_vel = &OD_RAM.x2015_actual_velocity;
        break;
    default:
        s->ctrl = NULL; s->target_pos = NULL; s->target_vel = NULL;
        s->status = NULL; s->actual_pos = NULL; s->actual_vel = NULL;
        break;
    }
}
 
 /* ═══════════════════════════════════════════════════════════════
  *  工具函数
  * ═══════════════════════════════════════════════════════════════ */
 
 static uint32_t now_ms(void) {
     return HAL_GetTick();
 }

 /* ═══════════════════════════════════════════════════════════════
  *  SDO 发起辅助 —— 发起返回值只在这里检查一次
  *
  *  co_master_start_* 返回 false 时客户端是【空闲】的。不检查的话，
  *  下一个周期看到的现象和「事务成功」完全一样，状态机会静默地往下走，
  *  最后表现为「电机不动，串口却什么都没说」。0x6040/0x6041 的发起
  *  收敛到这两个函数里，就不会有哪一处漏掉检查。
  * ═══════════════════════════════════════════════════════════════ */

 /** @brief 写 402 控制字 0x6040；成功则推进 deadline，失败返回 false */
 static bool_t sdo_write_ctrl(cia402_slave_t* s, uint16_t cmd) {
     if (!co_master_start_write(s->node_id, 0x6040, 0, &cmd, 2, CIA402_STEP_TIMEOUT_MS)) {
         printf("CiA402: node=%u 写控制字 0x%04X 发起失败: %s\r\n", (unsigned)s->node_id, (unsigned)cmd,
                co_master_start_error_name(co_master_get_start_error()));
         return false;
     }
     s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
     return true;
 }

 /** @brief 读 402 状态字 0x6041；成功则推进 deadline，失败返回 false */
 static bool_t sdo_read_status(cia402_slave_t* s) {
     if (!co_master_start_read(s->node_id, 0x6041, 0, CIA402_STEP_TIMEOUT_MS)) {
         printf("CiA402: node=%u 读状态字 0x6041 发起失败: %s\r\n", (unsigned)s->node_id,
                co_master_start_error_name(co_master_get_start_error()));
         return false;
     }
     s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
     return true;
 }

 /**
   * @brief 心跳表里本从站的那一项；表里没有它时返回 NULL
   * @note  抽出来只有一个理由：本文件有三处要读心跳（等 OP、运行中看门狗…），
   *        而 hb_idx 在注册失败时是 0xFF，直接拿去索引 monitoredNodes 会越界。
   */
 static CO_HBconsNode_t* hb_node(const cia402_slave_t* s) {
     if (s_co == NULL || s_co->HBcons == NULL
         || s->hb_idx >= s_co->HBcons->numberOfMonitoredNodes) {
         return NULL;
     }
     return &s_co->HBcons->monitoredNodes[s->hb_idx];
 }

 /** @brief 从站是否在线且在 OP —— 所有「能干活」的前提 */
 static bool_t slave_is_operational(const cia402_slave_t* s) {
     const CO_HBconsNode_t* n = hb_node(s);
     return (n != NULL)
            && (n->HBstate == CO_HBconsumer_ACTIVE)
            && (n->NMTstate == CO_NMT_OPERATIONAL);
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  402 状态转换通用处理（shutdown / switch_on / enable_op）
  *
  *  流程：写控制字 0x6040 → 读状态字 0x6041 确认目标状态
  *  返回：true=转换完成, false=进行中
  * ═══════════════════════════════════════════════════════════════ */
 
 static bool_t cia402_transition_tick(cia402_slave_t* s, uint16_t cmd, uint16_t target_status) {
     co_master_status_t st = { CO_MASTER_OK, 0 };

     switch (co_master_step_gate(s->node_id, s->deadline, &st)) {
     case CO_SDO_STEP_WAIT:
         /* 我的事务还在跑，或别人占着客户端 —— 两种都不算我超时 */
         return false;

     case CO_SDO_STEP_FAIL:
         s->fault = true;
         s->fault_code = (st.result == CO_MASTER_ERR_TIMEOUT) ? CIA402_ERR_TRANS_TIMEOUT
                                                              : CIA402_ERR_TRANS_SDO;
         return false;

     case CO_SDO_STEP_DONE: {
         if (s->trans_step == 0) {
             /* 控制字写完了。下一步改成读状态字，由下一轮的 NEXT 发起 ——
              * 发起和取结果必须都在闸门这一处，中途不能自己动手，
              * 否则结果会被别人的事务盖掉。 */
             s->trans_step = 1;
             return false;
         }
         uint16_t status;
         if (co_master_get_u16(&status) && ((status & CIA402_STS_MASK) == target_status)) {
             return true;   /* 转换完成 */
         }
         /* 还没到目标状态：trans_step 不变，下一轮 NEXT 再读一次。
          * 这个「再读一次」必须有整体预算兜着，否则从站一直不响应目标状态时
          * 就是无限循环 —— 每一步都成功，永远等不到任何超时。 */
         if (now_ms() >= s->state_deadline) {
             s->fault = true;
             s->fault_code = CIA402_ERR_TRANS_TIMEOUT;
         }
         return false;
     }

     case CO_SDO_STEP_NEXT:
     default:
         if (s->trans_step == 0) {
             if (!sdo_write_ctrl(s, cmd)) {
                 s->fault = true; s->fault_code = CIA402_ERR_TRANS_SDO;
             }
         } else {
             if (!sdo_read_status(s)) {
                 s->fault = true; s->fault_code = CIA402_ERR_TRANS_SDO;
             }
         }
         return false;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：enter 函数（状态切换时执行一次）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_enter(cia402_slave_t* s) {
    uint8_t  idx = (uint8_t)(s - s_slaves);
    const cia402_slave_config_t* cfg = s_cfgs[idx];

    /* enter 只摆状态（子步骤、超时），**不碰总线**。所有 SDO 一律由
     * fsm_tick 经 co_master_step_gate 发起 —— 发起和取结果必须在同一处，
     * 否则发出去的结果会被别的事务截走。 */
    s->trans_step = 0;
    /* 注意：这里【不清】s->retry。它记的是「这个轴重走过几次使能流程」，
     * 属于整个使能过程的状态，每次进状态都抹掉就等于无限重试。
     * 只有真正跑起来（CIA402_RUNNING）才清零。 */

    /* 默认预算：一步 SDO 的时间（客户端超时 + 100ms 余量）。
     * 下面只对「不是一步 SDO」的状态做覆盖。 */
    s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;

     switch (s->state) {
     case CIA402_WAIT_OP:
         s->deadline = now_ms() + CIA402_OP_TIMEOUT_MS;
         break;

     case CIA402_SET_MODE:
     case CIA402_SHUTDOWN:
     case CIA402_SWITCH_ON:
     case CIA402_ENABLE_OP:
     case CIA402_DISABLE_OP:
         /* 设模式、402 状态转换：形状都是「写一个对象再确认」。
          * 写什么、确认什么由 fsm_tick 按状态决定，这里不需要任何动作 ——
          * 用默认的 SDO 预算即可。但要单独给一个【整体】预算：驱动器一直不回
          * 目标状态时，光靠单次 SDO 超时是拦不住的（每次都成功）。 */
         s->state_deadline = now_ms() + CIA402_TRANS_TIMEOUT_MS;
         break;

    case CIA402_CFG_PARAMS:
        /* 根据当前模式选参数表，逐个 SDO 下发。
         * 位置类（PP / CSP）用 pos_params，速度类（PV）用 vel_params。 */
        if (s->target_mode == CIA402_MODE_PROFILE_VEL) {
            s->active_params = cfg->vel_params;
        } else if (s->target_mode == CIA402_MODE_PROFILE_POS ||
                   s->target_mode == CIA402_MODE_CSP) {
            s->active_params = cfg->pos_params;
        } else {
            s->active_params = NULL;   /* 其它模式没有参数表，直通 RUNNING */
        }
        s->param_step = 0;
        break;

    case CIA402_WAIT_RECONFIG:
        /* 触发 canopen_init 重配 PDO（拉 PRE-OP → 配 PDO → 进 OP） */
        co_init_restart(idx, s->pending_pdo_cfg);
        s->deadline = now_ms() + 10000U;  /* 重配超时 10s */
        break;
 
     case CIA402_RUNNING:
         s->retry = 0;   /* 真的跑起来了，重试预算归零 */
         printf("CiA402: node=%u init done, RUNNING\r\n", s->node_id);
         /* CSP 下主机必须持续下发目标点。进场第一件事是把目标锁到【当前实际位置】，
          * 否则会拿上一轮的残留目标（切换时被清零成 0）当指令发出去，
          * 驱动器会直接朝 0 位飞车。
          * 这里能安全读 x2004：从站早在重配 PDO 后就回了 OP，TPDO1 一直以 10ms
          * 上报，等状态机走完 SET_MODE/CFG_PARAMS/shutdown… 早已收到若干帧。 */
         if (s->target_mode == CIA402_MODE_CSP) {
             *s->ctrl           = CIA402_CMD_ENABLE_OP;
             *s->target_pos     = *s->actual_pos;
             s->scurve_active    = false;
             s->scurve_last_tick = now_ms();
         }
         break;
 
     case CIA402_FAULT:
         s->deadline = now_ms() + CIA402_FAULT_RETRY_MS;
         printf("CiA402: node=%u fault, code=0x%04X（已重试 %u/%u 次）\r\n",
                s->node_id, s->fault_code, (unsigned)s->retry, (unsigned)CIA402_FAULT_RETRY_MAX);
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
 *  大状态机：tick 函数（每周期执行）
 * ═══════════════════════════════════════════════════════════════ */

static void trajectory_tick(cia402_slave_t* s);

static void fsm_tick(cia402_slave_t* s) {
     CO_HBconsNode_t* node;
     uint8_t idx = (uint8_t)(s - s_slaves);

     /* 这里不再需要「SDO 令牌」—— 串行化已经由 co_master_step_gate 的归属
      * 判断完成，而且是按节点区分的：别的从站在用 SDO 时本状态机只会 WAIT，
      * 各走各的，谁的事务超时才算在谁头上。 */

     switch (s->state) {
 
    /* ── 等从站进 OP，进 OP 后先写 0x6040=0 强制 Switch on disabled，防止残留目标误动作 ── */
    case CIA402_WAIT_OP: {
        node = hb_node(s);
        if (node == NULL) {
            s->fault = true;
            s->fault_code = CIA402_ERR_OP_TIMEOUT;
            s->state = CIA402_FAULT;
            break;
        }
        if (!slave_is_operational(s)) {
            if (now_ms() >= s->deadline) {
                s->fault = true;
                s->fault_code = CIA402_ERR_OP_TIMEOUT;
                s->state = CIA402_FAULT;
            }
            break;
        }
        /* 进 OP 后，先写 0x6040=0x0000 让从站进入 Switch on disabled（安全停机）。
         * 这一步是【尽力而为】的：从站刚进 OP 时无法保证它处于 402 的哪个状态，
         * 写 0 只是把起点统一。它失败了也不该把轴判死 —— 后面 SET_MODE /
         * SHUTDOWN 还要写同一个对象，真有问题会在那里带着更准确的错误码报出来，
         * 而这两条路都会重试，不会因为一次瞬时失败就整机断电。 */
        switch (co_master_step_gate(s->node_id, s->deadline, NULL)) {
        case CO_SDO_STEP_WAIT:
            break;
        case CO_SDO_STEP_FAIL:
        case CO_SDO_STEP_DONE:
            s->trans_step = 0;
            s->state = CIA402_SET_MODE;
            break;
        case CO_SDO_STEP_NEXT:
        default:
            if (!sdo_write_ctrl(s, 0x0000U)) {
                /* 连发起都做不到，说明客户端本身有问题，值得当场报出来 */
                s->fault = true; s->fault_code = CIA402_ERR_TRANS_SDO;
                s->state = CIA402_FAULT;
            }
            break;
        }
        break;
    }
 
     /* ── 设置运行模式：写 0x6060，读 0x6061 确认 ── */
     case CIA402_SET_MODE: {
         co_master_status_t st = { CO_MASTER_OK, 0 };
         switch (co_master_step_gate(s->node_id, s->deadline, &st)) {
         case CO_SDO_STEP_WAIT:
             break;

         case CO_SDO_STEP_FAIL:
             s->fault = true;
             s->fault_code = (st.result == CO_MASTER_ERR_TIMEOUT) ? CIA402_ERR_MODE_TIMEOUT
                                                                  : CIA402_ERR_MODE_SDO;
             s->state = CIA402_FAULT;
             break;

         case CO_SDO_STEP_DONE:
             if (s->trans_step == 0) {
                 s->trans_step = 1;   /* 模式已写，下一轮改为读 0x6061 确认 */
                 break;
             }
             {
                 uint8_t actual;
                 if (co_master_get_u8(&actual) && actual == (uint8_t)s->target_mode) {
                     s->state = CIA402_CFG_PARAMS;
                 } else {
                     s->fault = true; s->fault_code = CIA402_ERR_MODE_MISMATCH;
                     s->state = CIA402_FAULT;
                 }
             }
             break;

         case CO_SDO_STEP_NEXT:
         default:
             if (s->trans_step == 0) {
                 uint8_t mode = (uint8_t)s->target_mode;
                 if (!co_master_start_write(s->node_id, 0x6060, 0, &mode, 1, CIA402_SDO_TIMEOUT_MS)) {
                     printf("CiA402: node=%u 写模式 0x6060 发起失败: %s\r\n", (unsigned)s->node_id,
                            co_master_start_error_name(co_master_get_start_error()));
                     s->fault = true; s->fault_code = CIA402_ERR_MODE_SDO;
                     s->state = CIA402_FAULT;
                     break;
                 }
             } else {
                 if (!co_master_start_read(s->node_id, 0x6061, 0, CIA402_SDO_TIMEOUT_MS)) {
                     printf("CiA402: node=%u 读模式 0x6061 发起失败: %s\r\n", (unsigned)s->node_id,
                            co_master_start_error_name(co_master_get_start_error()));
                     s->fault = true; s->fault_code = CIA402_ERR_MODE_SDO;
                     s->state = CIA402_FAULT;
                     break;
                 }
             }
             s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
             break;
         }
         break;
     }
 
     /* ── shutdown ── */
     case CIA402_SHUTDOWN:
         if (cia402_transition_tick(s, CIA402_CMD_SHUTDOWN, CIA402_STS_READY_SWITCH)) {
             s->state = CIA402_SWITCH_ON;
         } else if (s->fault) {
             s->state = CIA402_FAULT;
         }
         break;
 
     /* ── switch on ── */
     case CIA402_SWITCH_ON:
         if (cia402_transition_tick(s, CIA402_CMD_SWITCH_ON, CIA402_STS_SWITCHED_ON)) {
             s->state = CIA402_ENABLE_OP;
         } else if (s->fault) {
             s->state = CIA402_FAULT;
         }
         break;
 
     /* ── enable operation ── */
    case CIA402_ENABLE_OP:
        if (cia402_transition_tick(s, CIA402_CMD_ENABLE_OP, CIA402_STS_OP_ENABLED)) {
            s->state = CIA402_RUNNING;   /* 进OP后下发该模式参数 */
        } else if (s->fault) {
            s->state = CIA402_FAULT;
        }
        break;

    /* ── 下发模式参数（加速度/减速度/轮廓速度等） ── */
    case CIA402_CFG_PARAMS: {
        const cia402_mode_params_t* p = s->active_params;
        if (p == NULL) {  /* 没有参数表，直接进 RUNNING */
            s->state = CIA402_RUNNING;
            break;
        }
        co_master_status_t st = { CO_MASTER_OK, 0 };
        switch (co_master_step_gate(s->node_id, s->deadline, &st)) {
        case CO_SDO_STEP_WAIT:
            break;

        case CO_SDO_STEP_FAIL:
            s->fault = true;
            s->fault_code = (st.result == CO_MASTER_ERR_TIMEOUT) ? CIA402_ERR_MODE_TIMEOUT
                                                                 : CIA402_ERR_MODE_SDO;
            s->state = CIA402_FAULT;
            break;

        case CO_SDO_STEP_DONE:
            /* 上一条写成功。param_step 在发起时就已自增，下一轮 NEXT 接着找下一条。 */
            break;

        case CO_SDO_STEP_NEXT:
        default: {
            /* param_step：
             *   0=清上一模式残留目标(pos:0x607A=0, vel:0x60FF=0) 强制写，不走val==0跳过
             *   1=加速度 0x6083, 2=减速度 0x6084
             *   3=轮廓速度 0x6081, 4=最大轮廓速度 0x6082
             *   5=全部下发完 → 进 SHUTDOWN */
            uint16_t obj_idx = 0;
            uint32_t val = 0;
            while (s->param_step < 5u) {
                switch (s->param_step) {
                case 0:
                    /* 清残留目标：值为 0 也必须写，所以单独判断 */
                    obj_idx = (s->target_mode == CIA402_MODE_PROFILE_VEL) ? 0x60FFu : 0x607Au;
                    val = 0;
                    break;
                case 1:  obj_idx = 0x6083u; val = p->acceleration; break;
                case 2:  obj_idx = 0x6084u; val = p->deceleration; break;
                case 3:  obj_idx = 0x6081u; val = p->profile_velocity; break;
                default: obj_idx = 0x6082u; val = p->max_profile_vel; break;
                }
                if (s->param_step == 0u || val != 0u) {
                    break;      /* 这一条要写 */
                }
                s->param_step++;  /* val==0 表示上层不配置该项，跳过 */
            }
            if (s->param_step >= 5u) {
                s->state = CIA402_SHUTDOWN;
                break;
            }
            if (!co_master_start_write(s->node_id, obj_idx, 0, &val, 4, CIA402_SDO_TIMEOUT_MS)) {
                printf("CiA402: node=%u 写参数 0x%04X 发起失败: %s\r\n", (unsigned)s->node_id,
                       (unsigned)obj_idx, co_master_start_error_name(co_master_get_start_error()));
                s->fault = true; s->fault_code = CIA402_ERR_MODE_SDO;
                s->state = CIA402_FAULT;
                break;
            }
            s->param_step++;
            s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
            break;
        }
        }
        break;
    }

    /* ── 停机（切换模式前） ── */
    case CIA402_DISABLE_OP:
        /* 等状态字变成 Switched on（bit2=0） */
        if (cia402_transition_tick(s, CIA402_CMD_DISABLE_OP, CIA402_STS_SWITCHED_ON)) {
            s->state = CIA402_WAIT_RECONFIG;
        } else if (s->fault) {
            s->state = CIA402_FAULT;
        }
        break;

    /* ── 等 canopen_init 重配 PDO 完成 ── */
    case CIA402_WAIT_RECONFIG:
        if (co_init_done(idx)) {
            /* PDO 重配完成，从站已进 OP，切换模式并重新使能 */
            s->target_mode = s->pending_mode;
            s->state = CIA402_SET_MODE;
        } else if (now_ms() >= s->deadline) {
            s->fault = true;
            s->fault_code = CIA402_ERR_OP_TIMEOUT;
            s->state = CIA402_FAULT;
        }
        break;
 
     /* ── 运行中 / 故障 ── */
     case CIA402_RUNNING:
         /* RUNNING 的前提是「从站在线且在 OP」。掉线之后继续往 PDO 里写
          * 没有任何效果，而从站重新上线时会停在 Switch on disabled ——
          * 必须退回 WAIT_OP 重走一遍使能流程，否则就是「总线上一切正常、
          * 串口也安静，但轴不动」。canopen_init 那边对应的是故障重试。 */
         if (!slave_is_operational(s)) {
             printf("CiA402: node=%u 掉线，退回等待 OP\r\n", (unsigned)s->node_id);
             s->fault = false;
             s->fault_code = CIA402_ERR_NONE;
             s->state = CIA402_WAIT_OP;
             break;
         }
         trajectory_tick(s);
         break;

     case CIA402_FAULT:
         /* 有限次重走使能流程：从 WAIT_OP 重新来一遍（等 OP → 写 0x6040=0
          * → 设模式 → 参数 → shutdown → switch on → enable op）。
          * 这条路径平时切换模式也在走，是验证过的。 */
         if (s->retry < CIA402_FAULT_RETRY_MAX && now_ms() >= s->deadline) {
             s->retry++;
             s->fault = false;
             s->fault_code = CIA402_ERR_NONE;
             s->trans_step = 0;
             printf("CiA402: node=%u 重试第 %u 次\r\n", (unsigned)s->node_id, (unsigned)s->retry);
             s->state = CIA402_WAIT_OP;
         }
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  状态机分发：状态切换时调 enter，每周期调 tick
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_dispatch(cia402_slave_t* s) {
     if (s->state != s->prev_state) {
         s->prev_state = s->state;
         fsm_enter(s);
     }
     fsm_tick(s);
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  轨迹推进（RUNNING 状态的固有行为）
  *
  *  只在 CSP 模式下工作 —— 这是"主机生成轨迹"的模式，主机必须【持续】
  *  每周期下发一个目标点，驱动器只负责跟随。
  *     轨迹在跑   → 下发 S 曲线算出来的中间点
  *     轨迹跑完了 → 继续按同样周期重发最后一个点（保持位置）
  *  不能因为没在跑轨迹就停发：CSP 下长时间收不到 RPDO 驱动器会报跟随/超时。
  *
  *  PP / PV 模式下本函数直接返回，由驱动器自己规划。
  * ═══════════════════════════════════════════════════════════════ */

 static void trajectory_tick(cia402_slave_t* s) {
    if (s->target_mode != CIA402_MODE_CSP) return;
    uint8_t idx = (uint8_t)(s - s_slaves);
    const cia402_slave_config_t* cfg = s_cfgs[idx];
    if (cfg == NULL) return;

    /* dt_ms 配成 0 会让下面这个门槛永远为假，变成每轮主循环都发一帧，这里兜一下 */
    uint32_t period = (cfg->traj.dt_ms != 0u) ? cfg->traj.dt_ms : 10u;

    uint32_t elapsed = now_ms() - s->scurve_last_tick;
    if (elapsed < period) return;
    s->scurve_last_tick = now_ms();

    int32_t p;
    if (s->scurve_active) {
        /* 把【实际】经过的时间喂给规划器，主循环抖动不会累积成轨迹变形 */
        p = s_curve_step(&s->scurve, elapsed);
        if (s_curve_done(&s->scurve)) {
            s->scurve_active = false;
        }
    } else {
        p = (int32_t)(*s->target_pos);   /* 保持最后的点 */
    }

    *s->ctrl       = CIA402_CMD_ENABLE_OP;  /* CSP 不用 bit4/5/6 */
    *s->target_pos = (uint32_t)p;
    if (s_co != NULL) {
        /* TPDO 通道按 od_base_index 推导：0x2000→TPDO[0], 0x2010→TPDO[1] */
        uint8_t tpdo_idx = (cfg->od_base_index - 0x2000u) / 0x10u;
        CO_TPDOsendRequest(&s_co->TPDO[tpdo_idx]);
    }
}

 /* ═══════════════════════════════════════════════════════════════
  *  对外接口
  * ═══════════════════════════════════════════════════════════════ */
 
 void cia402_init(uint8_t idx, CO_t* co, const cia402_slave_config_t* cfg, cia402_mode_t mode) {
    if (idx >= CIA402_MAX_SLAVES) return;
    s_co = co;
    s_cfgs[idx] = cfg;
    cia402_slave_t* s = &s_slaves[idx];
    memset(s, 0, sizeof(*s));
    s->node_id = cfg->node_id;
    s->target_mode = mode;
    s->state = CIA402_WAIT_OP;
    s->prev_state = (cia402_state_t)-1;  /* 强制第一次进 enter */

    /* 绑定该从站的过程数据 OD 变量指针 */
    bind_od_ptrs(s, cfg->od_base_index);

    /* 心跳消费者由 canopen_init 配置，调用方应保证 canopen_init 已完成 */
    if (s_co != NULL && s_co->HBcons != NULL) {
        s->hb_idx = (uint8_t)CO_HBconsumer_getIdxByNodeId(s_co->HBcons, cfg->node_id);
    }

    printf("CiA402: init idx=%u node=%u mode=%d\r\n", idx, cfg->node_id, mode);
}

void cia402_process(uint8_t idx) {
    if (idx >= CIA402_MAX_SLAVES) return;
    fsm_dispatch(&s_slaves[idx]);
}
 
 cia402_state_t cia402_get_state(uint8_t idx) {
     if (idx >= CIA402_MAX_SLAVES) return CIA402_FAULT;
     return s_slaves[idx].state;
 }
 
 uint16_t cia402_get_fault_code(uint8_t idx) {
    if (idx >= CIA402_MAX_SLAVES) return 0;
    return s_slaves[idx].fault_code;
}

bool_t cia402_switch_mode(uint8_t idx, cia402_mode_t mode) {
    if (idx >= CIA402_MAX_SLAVES) return false;
    cia402_slave_t* s = &s_slaves[idx];
    const cia402_slave_config_t* cfg = s_cfgs[idx];
    if (s->state != CIA402_RUNNING) {
        printf("CiA402: idx=%u switch_mode rejected, not RUNNING (state=%d)\r\n", idx, s->state);
        return false;
    }
    /* 先通过 PDO 发一次：控制字=停机 + 目标=0，让从站在重配前就停住且目标清零 */
    *s->ctrl = CIA402_CMD_DISABLE_OP;  /* 0x0007，清bit3停机 */
    if (s->target_mode == CIA402_MODE_PROFILE_VEL) {
        *s->target_vel = 0;
    } else {
        *s->target_pos = 0;
    }
    if (s_co != NULL) {
        uint8_t tpdo_idx = (cfg->od_base_index - 0x2000u) / 0x10u;
        CO_TPDOsendRequest(&s_co->TPDO[tpdo_idx]);
    }
    const co_pdo_config_t* pdo_cfg = NULL;
    switch (mode) {
    case CIA402_MODE_PROFILE_VEL: pdo_cfg = cfg->vel_cfg; break;
    case CIA402_MODE_PROFILE_POS:
    case CIA402_MODE_CSP:         pdo_cfg = cfg->pos_cfg; break;
    default:                      break;
    }
    if (pdo_cfg == NULL) {
        printf("CiA402: idx=%u switch_mode rejected, no PDO config for mode=%d\r\n", idx, mode);
        return false;
    }
    s->scurve_active = false;
    s->pending_mode = mode;
    s->pending_pdo_cfg = pdo_cfg;
    s->state = CIA402_DISABLE_OP;
    printf("CiA402: idx=%u mode switch requested, target mode=%d\r\n", idx, mode);
    return true;
}

void cia402_set_velocity(uint8_t idx, int32_t rpm) {
    if (idx >= CIA402_MAX_SLAVES) return;
    cia402_slave_t* s = &s_slaves[idx];
    const cia402_slave_config_t* cfg = s_cfgs[idx];
    uint32_t pps = (uint32_t)((int64_t)rpm * cfg->pulse_per_rev / 60);
    *s->ctrl = CIA402_CMD_ENABLE_OP;
    *s->target_vel = pps;
    if (s_co != NULL) {
        uint8_t tpdo_idx = (cfg->od_base_index - 0x2000u) / 0x10u;
        CO_TPDOsendRequest(&s_co->TPDO[tpdo_idx]);
    }
}

void cia402_set_position_abs(uint8_t idx, int32_t position) {
    if (idx >= CIA402_MAX_SLAVES) return;
    cia402_slave_t* s = &s_slaves[idx];
    const cia402_slave_config_t* cfg = s_cfgs[idx];

    const cia402_traj_config_t* t = (cfg != NULL) ? &cfg->traj : NULL;

    if (s->target_mode == CIA402_MODE_CSP
        && t != NULL
        && t->curve_type == CIA402_TRAJ_S_CURVE
        && t->v_max != 0u && t->a_max != 0u && t->jerk != 0u) {

        s_curve_init(&s->scurve, position, (int32_t)(*s->actual_pos),
                     t->v_max, t->a_max, t->jerk, t->dt_ms);
        s->scurve_active    = true;
        s->scurve_last_tick = now_ms();
        return;
    }

    /* PP（或 CSP 但没配曲线）：把最终目标交给驱动器，一次写完 */
    *s->ctrl = (s->target_mode == CIA402_MODE_CSP)
             ? CIA402_CMD_ENABLE_OP    /* CSP 不用 bit4/5/6 */
             : CIA402_CMD_POS_ABS;     /* PP：bit4 上升沿触发新目标 */
    *s->target_pos = (uint32_t)position;
    if (s_co != NULL) {
        uint8_t tpdo_idx = (cfg->od_base_index - 0x2000u) / 0x10u;
        CO_TPDOsendRequest(&s_co->TPDO[tpdo_idx]);
    }
}

void cia402_set_position_rel(uint8_t idx, int32_t delta) {
    if (idx >= CIA402_MAX_SLAVES) return;
    cia402_slave_t* s = &s_slaves[idx];
    int32_t cur = (int32_t)(*s->actual_pos);
    cia402_set_position_abs(idx, cur + delta);
}

bool_t cia402_position_reached(uint8_t idx) {
    if (idx >= CIA402_MAX_SLAVES) return false;
    cia402_slave_t* s = &s_slaves[idx];

    if (s->target_mode != CIA402_MODE_PROFILE_POS
        && s->target_mode != CIA402_MODE_CSP) {
        return false;
    }
    if (s->scurve_active) {
        return false;
    }
    return ((*s->status & CIA402_STS_TARGET_REACHED) ? true : false);
}

uint16_t cia402_get_statusword(uint8_t idx) {
    if (idx >= CIA402_MAX_SLAVES) return 0;
    return *s_slaves[idx].status;
}

int32_t cia402_get_actual_velocity(uint8_t idx) {
    if (idx >= CIA402_MAX_SLAVES) return 0;
    uint32_t pps = *s_slaves[idx].actual_vel;
    return (int32_t)((int64_t)pps * 60 / s_cfgs[idx]->pulse_per_rev);
}

int32_t cia402_get_actual_position(uint8_t idx) {
    if (idx >= CIA402_MAX_SLAVES) return 0;
    return (int32_t)(*s_slaves[idx].actual_pos);
}