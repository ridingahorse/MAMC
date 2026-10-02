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
static cia402_slave_t s_slave;
static const cia402_slave_config_t* s_cfg = NULL;
 
 /* ═══════════════════════════════════════════════════════════════
  *  工具函数
  * ═══════════════════════════════════════════════════════════════ */
 
 static uint32_t now_ms(void) {
     return HAL_GetTick();
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  402 状态转换通用处理（shutdown / switch_on / enable_op）
  *
  *  流程：写控制字 0x6040 → 读状态字 0x6041 确认目标状态
  *  返回：true=转换完成, false=进行中
  * ═══════════════════════════════════════════════════════════════ */
 
 static bool_t cia402_transition_tick(cia402_slave_t* s, uint16_t cmd, uint16_t target_status) {
     (void)cmd;
     co_master_poll();
 
     if (co_master_busy()) {
         if (now_ms() >= s->deadline) {
             s->fault = true;
             s->fault_code = CIA402_ERR_TRANS_TIMEOUT;
         }
         return false;
     }
 
     if (co_master_get_result() != CO_MASTER_OK) {
         s->fault = true;
         s->fault_code = CIA402_ERR_TRANS_SDO;
         return false;
     }
 
     if (s->trans_step == 0) {
         /* 控制字写完了，读状态字确认 */
         s->trans_step = 1;
         co_master_start_read(s->node_id, 0x6041, 0, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
     } else {
         /* 读到状态字了 */
         uint16_t status;
         if (co_master_get_u16(&status)) {
             if ((status & CIA402_STS_MASK) == target_status) {
                 return true;  /* 转换完成 */
             }
         }
         /* 还没到目标状态，再读一次 */
         co_master_start_read(s->node_id, 0x6041, 0, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
     }
 
     return false;
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：enter 函数（状态切换时执行一次）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_enter(cia402_slave_t* s) {
     uint8_t  u8val;
     uint16_t u16val;
 
     s->trans_step = 0;
     s->retry = 0;
 
     switch (s->state) {
     case CIA402_WAIT_OP:
         s->trans_step = 0;
         s->deadline = now_ms() + CIA402_OP_TIMEOUT_MS;
         break;
 
     case CIA402_SET_MODE:
         u8val = (uint8_t)s->target_mode;
         co_master_start_write(s->node_id, 0x6060, 0, &u8val, 1, CIA402_SDO_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
         break;
 
     case CIA402_SHUTDOWN:
         u16val = CIA402_CMD_SHUTDOWN;
         co_master_start_write(s->node_id, 0x6040, 0, &u16val, 2, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
         break;
 
     case CIA402_SWITCH_ON:
         u16val = CIA402_CMD_SWITCH_ON;
         co_master_start_write(s->node_id, 0x6040, 0, &u16val, 2, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
         break;
 
     case CIA402_ENABLE_OP:
        u16val = CIA402_CMD_ENABLE_OP;
        co_master_start_write(s->node_id, 0x6040, 0, &u16val, 2, CIA402_STEP_TIMEOUT_MS);
        s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
        break;

    case CIA402_CFG_PARAMS:
        /* 根据当前模式选参数表，逐个 SDO 下发。
         * 位置类（PP / CSP）用 pos_params，速度类（PV）用 vel_params。 */
        if (s->target_mode == CIA402_MODE_PROFILE_VEL) {
            s->active_params = s_cfg->vel_params;
        } else if (s->target_mode == CIA402_MODE_PROFILE_POS ||
                   s->target_mode == CIA402_MODE_CSP) {
            s->active_params = s_cfg->pos_params;
        } else {
            s->active_params = NULL;   /* 其它模式没有参数表，直通 RUNNING */
        }
        s->param_step = 0;
        s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
        break;

    case CIA402_DISABLE_OP:
        /* 写控制字 0x0007（清 bit2），从 Operation enabled 退回 Switched on */
        u16val = CIA402_CMD_DISABLE_OP;
        co_master_start_write(s->node_id, 0x6040, 0, &u16val, 2, CIA402_STEP_TIMEOUT_MS);
        s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
        break;

    case CIA402_WAIT_RECONFIG:
        /* 触发 canopen_init 重配 PDO（拉 PRE-OP → 配 PDO → 进 OP） */
        co_init_restart(s->pending_pdo_cfg);
        s->deadline = now_ms() + 10000U;  /* 重配超时 10s */
        break;
 
     case CIA402_RUNNING:
         printf("CiA402: node=%u init done, RUNNING\r\n", s->node_id);
         /* CSP 下主机必须持续下发目标点。进场第一件事是把目标锁到【当前实际位置】，
          * 否则会拿上一轮的残留目标（切换时被清零成 0）当指令发出去，
          * 驱动器会直接朝 0 位飞车。
          * 这里能安全读 x2004：从站早在重配 PDO 后就回了 OP，TPDO1 一直以 10ms
          * 上报，等状态机走完 SET_MODE/CFG_PARAMS/shutdown… 早已收到若干帧。 */
         if (s->target_mode == CIA402_MODE_CSP) {
             OD_RAM.x2000_controlword     = CIA402_CMD_ENABLE_OP;
             OD_RAM.x2001_target_position = OD_RAM.x2004_actual_position;
             s->scurve_active    = false;
             s->scurve_last_tick = now_ms();
         }
         break;
 
     case CIA402_FAULT:
         printf("CiA402: node=%u fault, code=0x%04X\r\n", s->node_id, s->fault_code);
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
 *  大状态机：tick 函数（每周期执行）
 * ═══════════════════════════════════════════════════════════════ */

static void trajectory_tick(cia402_slave_t* s);

static void fsm_tick(cia402_slave_t* s) {
     CO_HBconsNode_t* node;
 
     switch (s->state) {
 
    /* ── 等从站进 OP，进 OP 后先写 0x6040=0 强制 Switch on disabled，防止残留目标误动作 ── */
    case CIA402_WAIT_OP: {
        node = &s_co->HBcons->monitoredNodes[s->hb_idx];
        if (node->HBstate != CO_HBconsumer_ACTIVE
            || node->NMTstate != CO_NMT_OPERATIONAL) {
            if (now_ms() >= s->deadline) {
                s->fault = true;
                s->fault_code = CIA402_ERR_OP_TIMEOUT;
                s->state = CIA402_FAULT;
            }
            break;
        }
        /* 进 OP 后，先写 0x6040=0x0000 让从站进入 Switch on disabled（安全停机） */
        co_master_poll();
        if (s->trans_step == 0) {
            uint16_t zero = 0x0000U;
            co_master_start_write(s->node_id, 0x6040, 0, &zero, 2, CIA402_STEP_TIMEOUT_MS);
            s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
            s->trans_step = 1;
        } else if (!co_master_busy() || now_ms() >= s->deadline) {
            s->trans_step = 0;
            s->state = CIA402_SET_MODE;
        }
        break;
    }
 
     /* ── 设置运行模式：写 0x6060，读 0x6061 确认 ── */
     case CIA402_SET_MODE:
         co_master_poll();
         if (co_master_busy()) {
             if (now_ms() >= s->deadline) {
                 s->fault = true; s->fault_code = CIA402_ERR_MODE_TIMEOUT;
                 s->state = CIA402_FAULT;
             }
             break;
         }
         if (co_master_get_result() != CO_MASTER_OK) {
             s->fault = true; s->fault_code = CIA402_ERR_MODE_SDO;
             s->state = CIA402_FAULT;
             break;
         }
         if (s->trans_step == 0) {
             /* 写完模式，读 0x6061 确认 */
             s->trans_step = 1;
             co_master_start_read(s->node_id, 0x6061, 0, CIA402_SDO_TIMEOUT_MS);
             s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
         } else {
             uint8_t actual;
             if (co_master_get_u8(&actual) && actual == (uint8_t)s->target_mode) {
                 s->state = CIA402_CFG_PARAMS;
             } else {
                 s->fault = true; s->fault_code = CIA402_ERR_MODE_MISMATCH;
                 s->state = CIA402_FAULT;
             }
         }
         break;
 
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
        co_master_poll();
        if (co_master_busy()) {
            if (now_ms() >= s->deadline) {
                s->fault = true; s->fault_code = CIA402_ERR_MODE_SDO;
                s->state = CIA402_FAULT;
            }
            break;
        }
        /* param_step：
         *   0=清上一模式残留目标(pos:0x607A=0, vel:0x60FF=0) 强制写，不走val==0跳过
         *   1=加速度 0x6083, 2=减速度 0x6084
         *   3=轮廓速度 0x6081, 4=最大轮廓速度 0x6082, 5=完成→SHUTDOWN */
        uint16_t idx = 0; uint32_t val = 0;
        bool_t do_write = true;
        bool_t force = false;   /* step0 清目标，val=0 也要写 */
        switch (s->param_step) {
        case 0:
            force = true;
            if (s->target_mode == CIA402_MODE_PROFILE_VEL) {
                idx = 0x60FF; val = 0;   /* 清残留目标速度 */
            } else {
                idx = 0x607A; val = 0;   /* 清残留目标位置（PP / CSP） */
            }
            break;
        case 1: idx = 0x6083; val = p->acceleration; break;
        case 2: idx = 0x6084; val = p->deceleration; break;
        case 3: idx = 0x6081; val = p->profile_velocity; break;
        case 4: idx = 0x6082; val = p->max_profile_vel; break;
        default: s->state = CIA402_SHUTDOWN; do_write = false; break;
        }
        if (do_write) {
            if (!force && val == 0) {
                /* 值为0跳过，不写SDO（step0除外） */
                s->param_step++;
            } else {
                co_master_start_write(s->node_id, idx, 0, &val, 4, CIA402_SDO_TIMEOUT_MS);
                s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
                s->param_step++;
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
        if (co_init_done()) {
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
         trajectory_tick(s);
         break;
     case CIA402_FAULT:
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
     if (s_cfg == NULL) return;

     /* dt_ms 配成 0 会让下面这个门槛永远为假，变成每轮主循环都发一帧，这里兜一下 */
     uint32_t period = (s_cfg->traj.dt_ms != 0u) ? s_cfg->traj.dt_ms : 10u;

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
         p = (int32_t)OD_RAM.x2001_target_position;   /* 保持最后的点 */
     }

     OD_RAM.x2000_controlword = CIA402_CMD_ENABLE_OP;  /* CSP 不用 bit4/5/6 */
     OD_RAM.x2001_target_position = (uint32_t)p;
     if (s_co != NULL) {
         CO_TPDOsendRequest(&s_co->TPDO[0]);
     }
 }

 /* ═══════════════════════════════════════════════════════════════
  *  对外接口
  * ═══════════════════════════════════════════════════════════════ */
 
 void cia402_init(CO_t* co, const cia402_slave_config_t* cfg, cia402_mode_t mode) {
     s_co = co;
     s_cfg = cfg;
     memset(&s_slave, 0, sizeof(s_slave));
     s_slave.node_id = cfg->node_id;
     s_slave.target_mode = mode;
     s_slave.state = CIA402_WAIT_OP;
     s_slave.prev_state = (cia402_state_t)-1;  /* 强制第一次进 enter */

     /* 心跳消费者由 canopen_init 配置，调用方应保证 canopen_init 已完成 */
     if (s_co != NULL && s_co->HBcons != NULL) {
         s_slave.hb_idx = (uint8_t)CO_HBconsumer_getIdxByNodeId(s_co->HBcons, cfg->node_id);
     }

     printf("CiA402: init, node=%u, mode=%d\r\n", cfg->node_id, mode);
 }
 
 void cia402_process(void) {
    fsm_dispatch(&s_slave);
}
 
 cia402_state_t cia402_get_state(void) {
     return s_slave.state;
 }
 
 uint16_t cia402_get_fault_code(void) {
    return s_slave.fault_code;
}

bool_t cia402_switch_mode(cia402_mode_t mode) {
    if (s_slave.state != CIA402_RUNNING) {
        printf("CiA402: switch_mode rejected, not RUNNING (state=%d)\r\n", s_slave.state);
        return false;
    }
    /* 先通过 PDO 发一次：控制字=停机 + 目标=0，让从站在重配前就停住且目标清零，
       防止从站进OP瞬间用残留目标高速误动作 */
    OD_RAM.x2000_controlword = CIA402_CMD_DISABLE_OP;  /* 0x0007，清bit3停机 */
    if (s_slave.target_mode == CIA402_MODE_PROFILE_VEL) {
        OD_RAM.x2002_target_velocity = 0;
    } else {
        OD_RAM.x2001_target_position = 0;
    }
    if (s_co != NULL) {
        CO_TPDOsendRequest(&s_co->TPDO[0]);
    }
    /* 根据目标模式选对应的 PDO 配置（从 s_cfg 里取，上层不用管）。
     * CSP 和 PP 的数据流一样（控制字 + 目标位置），共用 pos_cfg。 */
    const co_pdo_config_t* pdo_cfg = NULL;
    switch (mode) {
    case CIA402_MODE_PROFILE_VEL: pdo_cfg = s_cfg->vel_cfg; break;
    case CIA402_MODE_PROFILE_POS:
    case CIA402_MODE_CSP:         pdo_cfg = s_cfg->pos_cfg; break;
    default:                      break;
    }
    if (pdo_cfg == NULL) {
        printf("CiA402: switch_mode rejected, no PDO config for mode=%d\r\n", mode);
        return false;
    }
    /* 切模式时正在跑的软件轨迹作废 —— 不清的话 cia402_position_reached()
     * 会一直返回 false，而且重配完 PDO 回来后会用旧轨迹的中间点乱动 */
    s_slave.scurve_active = false;

    s_slave.pending_mode = mode;
    s_slave.pending_pdo_cfg = pdo_cfg;
    s_slave.state = CIA402_DISABLE_OP;
    printf("CiA402: mode switch requested, target mode=%d\r\n", mode);
    return true;
}

void cia402_set_velocity(int32_t rpm) {
    /* rpm → pps：pps = rpm * pulse_per_rev / 60 */
    uint32_t pps = (uint32_t)((int64_t)rpm * s_cfg->pulse_per_rev / 60);
    OD_RAM.x2000_controlword = CIA402_CMD_ENABLE_OP;
    OD_RAM.x2002_target_velocity = pps;
    if (s_co != NULL) {
        CO_TPDOsendRequest(&s_co->TPDO[0]);
    }
}

void cia402_set_position_abs(int32_t position) {
    /* 决定权在【运行模式】，不在 traj 配置：
     *   CSP   → 轨迹归主机，起软件曲线，由 trajectory_tick 逐点下发；
     *   其它  → 轨迹归驱动器，只把最终目标写一次。
     * 这样"模式"和"谁插值"不可能配出矛盾组合。
     * traj 配置只在 CSP 生效；PP 下即使配了 S_CURVE 也不会被调用。 */
    const cia402_traj_config_t* t = (s_cfg != NULL) ? &s_cfg->traj : NULL;

    if (s_slave.target_mode == CIA402_MODE_CSP
        && t != NULL
        && t->curve_type == CIA402_TRAJ_S_CURVE
        && t->v_max != 0u && t->a_max != 0u && t->jerk != 0u) {

        s_curve_init(&s_slave.scurve, position, (int32_t)OD_RAM.x2004_actual_position,
                     t->v_max, t->a_max, t->jerk, t->dt_ms);
        s_slave.scurve_active    = true;
        s_slave.scurve_last_tick = now_ms();
        return;
    }

    /* PP（或 CSP 但没配曲线）：把最终目标交给驱动器，一次写完 */
    OD_RAM.x2000_controlword = (s_slave.target_mode == CIA402_MODE_CSP)
                             ? CIA402_CMD_ENABLE_OP    /* CSP 不用 bit4/5/6 */
                             : CIA402_CMD_POS_ABS;     /* PP：bit4 上升沿触发新目标 */
    OD_RAM.x2001_target_position = (uint32_t)position;
    if (s_co != NULL) {
        CO_TPDOsendRequest(&s_co->TPDO[0]);
    }
}

void cia402_set_position_rel(int32_t delta) {
    /* 相对定位：目标 = 当前实际位置 + 偏移。
     * 注意 cur 来自最近一帧 TPDO（10ms 前的值），电机在动时会有滞后；
     * 对静止起步的定位无所谓，连续相对定位则应当由上层维护目标累加。 */
    int32_t cur = (int32_t)OD_RAM.x2004_actual_position;
    cia402_set_position_abs(cur + delta);
}

bool_t cia402_position_reached(void) {
    /* 非位置模式：bit10 的语义是"到达转速"，别拿它当到位用 */
    if (s_slave.target_mode != CIA402_MODE_PROFILE_POS
        && s_slave.target_mode != CIA402_MODE_CSP) {
        return false;
    }
    /* 软件轨迹还在跑：CSP 下每个中间点都会让 bit10 抖动，先看自己发完没有 */
    if (s_slave.scurve_active) {
        return false;
    }
    /* 再看从站自己的到位位 —— 两个条件都要满足，
     * 否则会出现"点发完了但从站根本没动"却报到位的情况 */
    return (OD_RAM.x2003_statusword & CIA402_STS_TARGET_REACHED) ? true : false;
}

uint16_t cia402_get_statusword(void) {
    return OD_RAM.x2003_statusword;
}

int32_t cia402_get_actual_velocity(void) {
    /* pps → rpm：rpm = pps * 60 / pulse_per_rev */
    uint32_t pps = OD_RAM.x2005_actual_velocity;
    return (int32_t)((int64_t)pps * 60 / s_cfg->pulse_per_rev);
}

int32_t cia402_get_actual_position(void) {
    return (int32_t)OD_RAM.x2004_actual_position;
}