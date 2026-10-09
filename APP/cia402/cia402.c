/**
  * @file    cia402.c
  * @brief   CiA 402 主站侧 —— 非阻塞初始化状态机实现
  *
  * 本模块只负责 CiA 402 专属部分：从站已进 OP 后，设模式 + 402 状态转换。
  * 通用 CANopen 初始化（等PRE-OP、配心跳、配PDO、发NMT进OP）由 canopen_init
  * 模块完成，本模块不关心。
  *
  * 流程：等OP → 设模式 → 参数 → shutdown → switch_on → enable_op → RUNNING
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
    /* PV 启动沿的步序，和 trans_step 一样进状态就归零。
     * 两者必须分开计数：本函数是【下一拍】才跑的（fsm_dispatch 按
     * state != prev_state 触发），所以「state 已置成 RUNNING、enter 还没跑」
     * 之间存在一个窗口。这时 trans_step 还停在 402 转换留下的 1，
     * pv_start_busy() 就会误判「启动沿正在走」，把 cia402_set_velocity 里那次
     * 控制字写入挡掉 —— 而 *s->ctrl 此时还是 OD 初值 0，这一帧出去，
     * 驱动器当场掉进 Switch On Disabled（0x0670），0x000F 再也唤不回来。踩过。 */
    s->pv_step = 0;
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
         /* 进场第一件事：把控制字的【常驻值】摆成 0x000F。它是 RPDO 上的内容，
          * 谁发帧谁就把它带出去 —— 留着初值 0 的话，任何一帧（set_velocity 的
          * 周期重发就会）都让驱动器直接掉出 Operation enabled、状态字回 0x0670
          * （Switch On Disabled），而且 0x000F 唤不回来，得重走 0x06→0x07→0x0F，
          * 等于两轴一起哑掉、只能断电。这个坑踩过一次，别再把控制字留空。 */
         *s->ctrl = CIA402_CMD_ENABLE_OP;
         /* CSP 下主机必须持续下发目标点。目标锁到【当前实际位置】，否则会拿上一轮
          * 的残留目标（切换时被清零成 0）当指令发出去，驱动器会直接朝 0 位飞车。
          * 这里能安全读 x2004：从站早在重配 PDO 后就回了 OP，TPDO1 一直以 10ms
          * 上报，等状态机走完 SET_MODE/CFG_PARAMS/shutdown… 早已收到若干帧。 */
         if (s->target_mode == CIA402_MODE_CSP) {
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
static void pv_start_tick(cia402_slave_t* s, uint8_t idx);

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
            /* 使能完成。速度模式还欠一次 bit8 启动触发，那个由 RUNNING 里的
             * pv_start_tick 补 —— 它不能放在这里：手册要求目标速度先写好，
             * 而此刻 0x60FF 刚被 CFG_PARAMS 清成 0。 */
            s->state = CIA402_RUNNING;
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
         pv_start_tick(s, idx);   /* PV：补一次 bit8 下降沿启动触发（PP/CSP 不走） */
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
 
 /* 启动沿是否【正在走】—— ① 已经发出去、③ 还没走完的那一段（步 1~5）。
  * 这一段控制字归 pv_start_tick 独占：沿靠 0x000F→0x010F→0x000F 三帧的【顺序】
  * 成立，谁再插一帧 0x000F，中间那拍 halt 就被冲掉，沿不复存在。
  * cia402_set_velocity 的周期重发就是这样一个写入者，所以它必须问过这里（见该函数）。
  *
  * 步 0【不算】占用：那时 ① 还没发，而 set_velocity 正要靠写 0x000F 把驱动器留在
  * Operation enabled。把它也挡掉，发出去的帧就带着控制字的初值 0，驱动器立刻掉出
  * 使能再也回不来 —— 两轴一起哑掉。踩过。
  *
  * 看的是 pv_step 而不是 trans_step：后者是 402 使能转换的子步骤，在
  * 「state 已置成 RUNNING、fsm_enter 还没跑」的那一拍里它还停在 1，
  * 会让这里误判成「沿正在走」—— 又回到上面那个初值 0 的坑。见 fsm_enter。 */
 static bool_t pv_start_busy(const cia402_slave_t* s) {
     return (s->state == CIA402_RUNNING)
         && (s->target_mode == CIA402_MODE_PROFILE_VEL)
         && (s->pv_step >= 1u) && (s->pv_step <= 5u);
 }

 /* ═══════════════════════════════════════════════════════════════
  *  速度模式启动触发（PV 专属，PP / CSP 绝不走这里）
  *
  *  旧固件（MS12A/MS10A）的 PV 速度发生器要控制字 bit8 走一次 1→0 的下降沿
  *  才启动：402 状态机已经 Operation Enabled、0x6060=3、0x60FF 也写好了目标
  *  速度，只给 0x000F 电机照样不转（0x606C 恒为 0、无报错、电机保持励磁）。
  *  新固件（MS12B/MS12C）没有这条厂商特殊逻辑，兼容该时序，所以不做固件区分。
  *
  *  手册 4.4 的两条硬约束，决定了这段代码的形状：
  *    1. 目标速度必须先写好，再做 bit8 跳变 —— 脉冲早于速度设置【无效】；
  *    2. 脉冲每次切入 PV 模式只执行一次，运行中更新 0x60FF 不需要重复。
  *
  *  所以触发点不能放在使能那一刻：那时 0x60FF 刚被 CFG_PARAMS 清成 0，
  *  打出来的沿是废的，而这唯一的一次机会就用掉了（踩过这个坑）。
  *  正确时机是「上层第一次下达非零速度指令之后」，也就是 RUNNING 里。
  *
  *  步序（每步间隔 CIA402_HALT_GAP_MS，6 步走完 ≈120ms）：
  *    0: ① 0x000F + 目标速度 —— 确保目标先上总线（约束 1）
  *    1:    SYNC
  *    2: ② 0x010F          —— bit8 置位，驱动器进入停止
  *    3:    SYNC
  *    4: ③ 0x000F          —— bit8 下降沿，触发速度模式开始运转
  *    5:    SYNC
  *
  *  帧与帧要隔开：CO_TPDOsendRequest() 只是置个标志，真正的发送在下一轮
  *  CO_TPDO_process()，写太快会把还没上总线的前一帧直接覆盖掉，下降沿就不存在了。
  *
  *  帧后必须跟一条 SYNC：从站现在配成【同步接收】（0x1400.02 = 0x01），两次 SYNC
  *  之间它只锁存最后一帧 —— ①②③ 挤在同一段里只有 ③ 活得下来，中间那拍 halt 从
  *  没生效，下降沿同样不存在。新固件不靠这个沿，漏了看不出问题；旧固件会直接退回
  *  「使能了但不发脉冲」，和当初那个故障一模一样。
  *
  *  这 6 步期间【控制字归本函数独占】，别的写控制字的地方要让路（pv_start_busy），
  *  否则它插进来的一帧 0x000F 同样能把 halt 冲掉。
  * ═══════════════════════════════════════════════════════════════ */

static void pv_start_tick(cia402_slave_t* s, uint8_t idx) {
    const cia402_slave_config_t* cfg = s_cfgs[idx];
    bool_t sync_step;

    /* 只有 PV 需要（位置 / 回零模式下 bit8 就是标准 Halt，乱发会引入异常行为），
     * 且只在本次切入 PV 的启动沿还没走完时才推进 —— 走完就是普通调速了。 */
    if (s->target_mode != CIA402_MODE_PROFILE_VEL) return;
    if (s->pv_step > 5u) return;

    /* 步与步之间必须真的隔开，理由见函数头那两条约束：CO_TPDOsendRequest() 只是
     * 置个标志，真正的发送在下一轮 CO_TPDO_process()（1ms 中断）里。步子走太快，
     * 前一帧还没上总线就被后一帧覆盖 —— 最要命的是 ② halt 被 ③ 盖掉，bit8 的
     * 下降沿压根不存在，轴退回「使能了但不发脉冲」。
     *
     * 下面每步末尾设的 s->deadline 本来就是为这件事设的，但一直只设不判；实际靠的
     * 是那几句 printf 的串口耗时（115200 波特，一行三五毫秒）把步子撑开 —— 碰巧成立，
     * 主循环一变快就散。这里改成真的等。
     * pv_step==0 不判：它是这段的第一步，没有「上一帧」要等，而且它得尽快响应
     * 上层第一次下达的非零指令。 */
    if (s->pv_step != 0u && now_ms() < s->deadline) return;

    sync_step = ((s->pv_step & 1u) != 0u);   /* 奇数步只补 SYNC，没有帧 */

    if (sync_step) {
        co_sync_send();
    } else if (s->pv_step == 0u) {
        /* 等上层第一次下达非零速度指令。切入 PV 时 0x60FF 已被清 0，
         * 所以「它非零」就是「上层已经下过速度指令」的判据。
         * 目标为 0 时绝不能打这个沿 —— 约束 1 说那样无效，而约束 2 说
         * 每次切入 PV 只有一次机会，浪费掉这个轴就再也起不来了。 */
        if (*s->target_vel == 0u) return;
        *s->ctrl = CIA402_CMD_ENABLE_OP;
        printf("CiA402: node=%u PV 启动 ①：0x000F + 目标速度 %lu pps\r\n",
               (unsigned)s->node_id, (unsigned long)*s->target_vel);
    } else if (s->pv_step == 2u) {
        /* 走②之前上层把目标撤回了（STOP 拍）？那这次沿就别打了 —— 约束 1 说
         * 目标为 0 时打了无效，而机会只有一次。退回未开始，等下一次非零指令；
         * 此时 halt 还没发出去，驱动器不知道有这么回事。 */
        if (*s->target_vel == 0u) { s->pv_step = 0u; return; }
        *s->ctrl = CIA402_CMD_ENABLE_OP_HALT;
        printf("CiA402: node=%u PV 启动 ②：0x010F（bit8 置位）\r\n", (unsigned)s->node_id);
    } else {
        *s->ctrl = CIA402_CMD_ENABLE_OP;
        printf("CiA402: node=%u PV 启动 ③：0x000F（bit8 下降沿）\r\n", (unsigned)s->node_id);
    }

    /* 把这一帧真的推上总线（TPDO 序号由 0x2000/0x2010 基址算出）。
     * SYNC 拍没有帧要发 —— 下一拍的 SYNC 就是来锁存这一帧的。 */
    if (!sync_step && s_co != NULL && cfg != NULL) {
        CO_TPDOsendRequest(&s_co->TPDO[(cfg->od_base_index - 0x2000u) / 0x10u]);
    }
    s->pv_step++;
    s->deadline = now_ms() + CIA402_HALT_GAP_MS;
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

    /* 过程映象的初值必须是「保持使能 + 零目标」，不能是 OD.c 给的开机初值 0。
     *
     * 为什么：这两台驱动器收到控制字 0 就掉进 Switch On Disabled，而从那里
     * 0x000F 唤不回来，只能重走 0x06→0x07→0x0F（见 cia402.h 里 pv_step 的注释，
     * 同一个坑）。
     *
     * 这个 0 会真的上总线：apply_master_pdo_mapping() 末尾的 CO_CANopenInitPDO()
     * 借 CO_TPDO_init() 把主站每个 TPDO 的 sendRequest 无条件置起
     * （见 CO_PDO.c:1108），而主站自己已经是 OPERATIONAL、TPDO 在 1ms 中断里发
     * —— 于是重配 PDO 那一刻，OD 里当时的过程数据就被发了出去。谁在 OP 谁中招。
     *
     * 主站每次重启时两台驱动器都还在 OP（主站复位不会让它们掉出 OP），谁先配完
     * PDO 谁就把当时正在使能的另一台打死 —— 现象就是「随机只有一台不转、
     * 而且轴是松的（根本没使能）」。
     *
     * 堵发送的那一半在 canopen_init.c（重配后清 sendRequest）；这里堵内容：
     * 把初值摆成安全常驻值，之后不管哪一帧被提前发出去，内容都只是「使能 + 零速」，
     * 对驱动器无害。0x000F 发给还没使能的轴 = 使能，发给已在 OP 的轴 = 保持使能。 */
    if (s->ctrl != NULL) *s->ctrl = CIA402_CMD_ENABLE_OP;

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

bool_t cia402_pv_ready(uint8_t idx) {
    if (idx >= CIA402_MAX_SLAVES) return false;
    /* pv_start_tick 走完 6 步（0~5）就停在 6；中途被上层撤回目标会退回 0，
     * 那种情况就是「还没走完」，等它重新开始。 */
    return s_slaves[idx].pv_step > 5u;
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

    /* 启动沿期间控制字归 pv_start_tick 独占：那段时序靠 0x000F→0x010F→0x000F
     * 三帧的【顺序】成立，这里再插一帧 0x000F 就把中间那拍 halt 冲掉了。
     * 目标速度照写不误 —— 它本来就要先于沿到位（手册约束 1）；给 0 就是停机指令。 */
    if (!pv_start_busy(s)) {
        *s->ctrl = CIA402_CMD_ENABLE_OP;
    }
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