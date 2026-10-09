/* APP/test/test.h — 两从站【同步】测试：先速度模式同步转，再位置模式同步转
 *
 * 两拍交替，转向相反，循环执行：
 *   拍1  VEL+60    速度模式（0x6060 = 3），两轴一起 +60 rpm（正转）
 *   拍2  POS-back  位置模式（0x6060 = 1），两轴一起往回走 1 圈（反转）
 * 两拍之间由 cia402_switch_mode 自动完成「停机 → 重配 PDO → 设新模式 →
 * 重发参数 → 重新使能」，这是已有路径，本文件只管等它走完。
 *
 * 每一拍都按同一套「同步」判据走：
 *
 *   阶段      动作                            此刻该看到
 *   STAGE1    只写轴1 的新目标，等 GAP        两轴都还是旧值
 *   STAGE2    只写轴2 的新目标，等 GAP        轴1 的新目标已上路 GAP ms，轴1 仍是旧值 ★
 *   SYNC      发 SYNC                         两轴在这一刻【同时】生效 ★
 *   RUN       保持 hold_ms，期间周期重发       两轴同步运行
 *
 * ★ 那两行是判据：STAGE2 打印里轴1 还是旧值 = 数据停在从站的同步缓冲区里、
 *   没写进对象字典；SYNC 之后两轴同时变 = 这一条 SYNC 把两轴一起落盘。
 *   反过来，若轴1 在 STAGE2 就已经是新值，说明同步接收没生效（从站 0x1400.02
 *   不是 1，或者驱动器压根不等 SYNC）。
 *   2026-10-09 实测：这两台 DM556-CAN 属于后者，收到就生效。判据先留着 ——
 *   它是「同步接收成不成立」的唯一直接证据，删了就再也看不见了。
 *
 * 【重发只发已经装订过的轴】周期重发是为了对付驱动器的 RPDO 超时保护
 * （长时间收不到 RPDO 会自己停机），而装订阶段恰恰是「某一轴长时间没有新目标」
 * 的那一段。但重发的【内容】必须是该轴已经装订下去的值：还没装订的轴一律不发，
 * 让它在总线上保持上一拍的状态。位置拍尤其不能乱发 —— 位置拍的暂存值是
 * 【绝对目标】，随手发一个 undefined 的值就是把轴叫到不知道哪里去。
 *
 * 【每次进速度拍之前先预演一轮「启动沿」】(T_IGNITE) 每次进 PV 的 RUNNING，
 * cia402.c 的 PV 启动 ①/②/③ 都还没走 —— 那三步自带 SYNC，会把同步缓冲区里的
 * 数据提前落盘，混在测量拍里就分不清是哪条 SYNC 落的盘。所以速度拍之前先用一个
 * 很小的转速把两轴的沿一起消化掉，再清 0 + 一条 SYNC 摆回已知状态，等停稳才开测。
 * 位置拍不需要（PV 启动沿只在速度模式走）。
 *
 * 数据流：
 *   速度拍  主站 TPDO1/2 (0x202/0x204) → 从站 RPDO1 : 控制字 0x6040 + 目标速度 0x60FF
 *           从站 TPDO1   (0x182/0x184) → 主站 RPDO1/2 : 状态字 0x6041 + 实际速度 0x606C
 *   位置拍  主站 TPDO1/2 (0x202/0x204) → 从站 RPDO1 : 目标位置 0x607A + 控制字 0x6040
 *           从站 TPDO1   (0x182/0x184) → 主站 RPDO1/2 : 状态字 0x6041 + 实际位置 0x6064
 *   两套映射由 user_pdo_cfg_slave.h 的 vel_cfg / pos_cfg 给出，切换模式时
 *   canopen_init 会把主站和从站两侧的映射一起重配。
 *
 * 判定依据：
 *   - 状态字 0x6041：bit2 = Operation Enabled，bit10 = 到位 / 到达转速
 *   - 速度拍看 OD_RAM 0x2005 / 0x2015（从站上报的实际速度 0x606C），单位 rpm
 *   - 位置拍看 OD_RAM 0x2004 / 0x2014（从站上报的实际位置 0x6064），单位脉冲
 *   过程数据恒为 0 而电机在转，说明从站 TPDO1 的映射没生效，不是电机没动。
 *
 * 本文件只做测试，随便改。
 */
#ifndef _APP_TEST_H_
#define _APP_TEST_H_

#include "slave_config.h"   /* SLAVE1/2_PULSE_PER_REV */
#include "cia402.h"
#include "canopen_init.h"   /* co_sync_send / co_init_get_state */
#include "canopen_master.h" /* 运行时探针用的 SDO 客户端闸门 */

/* ── 两拍的转向：必须相反 ─────────────────────────────────────────
 * 台架上如果看到两拍同向，翻【其中一个】的符号就行 —— 驱动器到电机的
 * 接线方向各机不同，没法在代码里一次定死。 */
#define TEST_RPM            60          /* 速度拍：+60 rpm（正转） */
#define TEST_POS_PULSES     (-50000)    /* 位置拍：相对当前实际位置走 −50000 脉冲
                                         * = 反转 1 圈（DM556 出厂 50000 脉冲/圈） */

#define TEST_RUN_MS         3000U       /* 每拍转起来保持多久（位置拍要够走完） */
#define TEST_STAGE_GAP_MS   200U        /* 装订间隔：轴1写目标→轴2写目标、轴2写目标→SYNC */
#define TEST_REFRESH_MS     100U        /* 周期重发暂存设定值的周期，见文件头 */
#define TEST_POLL_MS        250U        /* 运行中的采样/打印周期 */
#define TEST_WAIT_DBG_MS    1000U       /* 等使能 / 等模式切换时的诊断打印周期 */
#define TEST_SWITCH_TIMEOUT_MS  15000U  /* 模式切换（含 PDO 重配）的预算 */
#define TEST_SETTLE_MS      500U        /* 模式切换完成后等多久才开测：切到位置模式后
                                         * 从站的 TPDO1 要重新上报几帧 0x6064，主站的
                                         * 0x2004/0x2014 才从速度模式留下的旧值变成真位置。
                                         * 不等就装订，位置拍的绝对目标会算在错的基准上。 */

/* PV 启动沿预演（见文件头）。转速只要非零就行 —— 这一步是为了把那一次 bit8
 * 下降沿的机会花掉，不是为了让它转；用小值，免得预演本身把轴甩起来。 */
#define TEST_IGNITE_RPM     1           /* 预演转速(rpm) */
#define TEST_IGNITE_SETTLE_MS 400U      /* 预演停稳后再开始测量：清 0 之后轴还要减速，
                                         * 不等就进测量，第一拍的判据会读到预演残留转速 */
#define TEST_IGNITE_TIMEOUT_MS 2000U    /* 预演没走完也必须说话并往下走，不能把测试挂死 */

/* 位置拍「算它动了」的门槛（脉冲）。太小会被反馈噪声骗过，太大则 200ms 的
 * 装订间隔内看不出来。100/50000 圈 ≈ 0.72° */
#define TEST_POS_MOVED_EPS  100

/* 本测试固定用两个从站（cia402 的数组上限是 CIA402_MAX_SLAVES，不是从站数） */
#define TEST_SLAVE_COUNT    2U

#define TEST_LAG_NONE       0xFFFFFFFFu /* 「本拍还没测到」的哨兵值 */

/* ═══════════════════════════════════════════════════════════════
 *  拍表：两拍，转向相反，循环执行
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    const char*   name;
    cia402_mode_t mode;
    int32_t       rpm;      /* 速度拍的目标转速（位置拍不用） */
    int32_t       pulses;   /* 位置拍的相对位移，负=反转（速度拍不用） */
    uint32_t      hold_ms;  /* 保持时间 */
} test_beat_t;

static const test_beat_t s_beats[] = {
    { "VEL+60",   CIA402_MODE_PROFILE_VEL, TEST_RPM,       0,               TEST_RUN_MS },
    { "POS-back", CIA402_MODE_PROFILE_POS, 0,              TEST_POS_PULSES, TEST_RUN_MS },
};
#define TEST_BEAT_COUNT (sizeof(s_beats) / sizeof(s_beats[0]))

/** @brief 本拍是不是位置类（PP / CSP）。CSP 目前没用到，写成判断是为了不写死 */
static inline bool_t test_beat_is_pos(const test_beat_t* b)
{
    return (bool_t)(b->mode == CIA402_MODE_PROFILE_POS || b->mode == CIA402_MODE_CSP);
}

/* ═══════════════════════════════════════════════════════════════
 *  装订：把某一轴的新目标写下去，但【不发 SYNC】
 * ═══════════════════════════════════════════════════════════════ */

/* 每轴的暂存设定值 = 「本拍已经装订下去、但还没被 SYNC 落盘」的目标。
 * 速度拍是 rpm，位置拍是【绝对目标脉冲数】。 */
static int32_t s_stage_rpm[TEST_SLAVE_COUNT];
static int32_t s_stage_pos[TEST_SLAVE_COUNT];

/* 本拍哪些轴已经装订过（位 i = 轴 i）。周期重发只认它，见文件头。 */
static uint8_t s_staged_mask = 0U;
#define TEST_STAGED_ALL ((uint8_t)((1U << TEST_SLAVE_COUNT) - 1U))

/** @brief 装订第 i 轴：把本拍的新目标写下去（内部会请求本轴 TPDO，但没有 SYNC） */
static inline void test_stage_axis(uint8_t i, const test_beat_t* b)
{
    if (test_beat_is_pos(b)) {
        /* 相对【当前实际位置】偏移：负脉冲就是往回走，与速度拍的正转相反。
         * 绝对目标只算这一次就存下来 —— 重发时不能再读一次实际位置，那样目标
         * 会跟着实际位置往后爬，轴永远走不到头。 */
        s_stage_pos[i] = cia402_get_actual_position(i) + b->pulses;
        cia402_set_position_abs(i, s_stage_pos[i]);
    } else {
        s_stage_rpm[i] = b->rpm;
        cia402_set_velocity(i, s_stage_rpm[i]);
    }
    s_staged_mask |= (uint8_t)(1U << i);
}

/** @brief 周期重发【已经装订过】的轴的设定值。装订阶段和运行拍都调，理由见文件头 */
static inline void test_refresh_targets(bool_t pos_mode)
{
    for (uint8_t i = 0U; i < TEST_SLAVE_COUNT; i++) {
        if ((s_staged_mask & (uint8_t)(1U << i)) == 0U) {
            continue;   /* 还没装订，不许发 —— 发出去的就是上一拍的残留值 */
        }
        if (pos_mode) {
            cia402_set_position_abs(i, s_stage_pos[i]);
        } else {
            cia402_set_velocity(i, s_stage_rpm[i]);
        }
    }
}

/** @brief 开一拍：清空装订状态，然后装订轴1（轴2 等 STAGE2） */
static inline void test_beat_begin(const test_beat_t* b)
{
    s_staged_mask = 0U;
    test_stage_axis(0U, b);
}

/** @brief 进速度拍之前的启动沿预演开场：两轴给同一个很小的转速，让 ①/②/③ 一起走完 */
static inline void test_ignite_begin(void)
{
    for (uint8_t i = 0U; i < TEST_SLAVE_COUNT; i++) {
        s_stage_rpm[i] = TEST_IGNITE_RPM;
        cia402_set_velocity(i, s_stage_rpm[i]);
    }
    s_staged_mask = TEST_STAGED_ALL;    /* 预演期间两轴都按暂存值重发 */
}

/** @brief 打印一次装订动作，带上两轴此刻的实际值（判据就在这里） */
static inline void test_print_stage(const test_beat_t* b, uint8_t axis, uint32_t at_ms)
{
    if (test_beat_is_pos(b)) {
        printf("[T] %s: stage s%u -> %+ld pulses @+%lums | s1 pos=%+ld s2 pos=%+ld\r\n",
               b->name, (unsigned)(axis + 1U), (long)s_stage_pos[axis],
               (unsigned long)at_ms,
               (long)cia402_get_actual_position(0), (long)cia402_get_actual_position(1));
    } else {
        printf("[T] %s: stage s%u -> %+ld rpm @+%lums | s1 vel=%+ld s2 vel=%+ld\r\n",
               b->name, (unsigned)(axis + 1U), (long)s_stage_rpm[axis],
               (unsigned long)at_ms,
               (long)cia402_get_actual_velocity(0), (long)cia402_get_actual_velocity(1));
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  运行时 SDO 探针（只读诊断，定位完整块删掉）
 * ═══════════════════════════════════════════════════════════════
 * PDO 是单向的，帧发出去之后就再也看不到它了，而「轴不动」既可能是帧没发对，
 * 也可能是驱动器收到了却按它自己的意思解释 —— 看主站这边的状态量分不出来。
 * 探针直接问驱动器自己的视角，这是唯一能分开这两种情况的办法。
 *
 * 一轮只在每拍 SYNC 之后采一次：整拍里状态是不变的，采一次就够，
 * 日志也不会被刷屏。速度拍问 0x60FF / 0x606C，位置拍问 0x607A / 0x6064 ——
 * 问错模式的量只会读到「对象不存在」，白占一轮。
 *
 * 一次只挂一笔事务，被占着就下一轮再试（co_master_step_gate 就是为此设计的）。
 * 运行期 cia402 已经 RUNNING，不再占 SDO 客户端，两边不会打架。 */
typedef struct {
    uint8_t     node_id;
    uint16_t    index;
    uint8_t     sub;
    const char* name;
} test_probe_t;

#define TEST_PROBE_START_MS     50U     /* SYNC 之后多久开始采（等驱动器把缓存应用掉） */
#define TEST_PROBE_TIMEOUT_MS   300U    /* 单条读取的预算 */

/* 同样的对象问两台，读数不一样的那一项就是答案 */
static const test_probe_t s_probe_vel[] = {
    { SLAVE1_NODE_ID, 0x6061, 0, "op mode display"   },
    { SLAVE1_NODE_ID, 0x6040, 0, "controlword (sdo)" },
    { SLAVE1_NODE_ID, 0x603F, 0, "error code"        },
    { SLAVE1_NODE_ID, 0x1003, 1, "last error"        },
    { SLAVE1_NODE_ID, 0x6041, 0, "statusword (sdo)"  },
    { SLAVE1_NODE_ID, 0x60FF, 0, "target vel"        },
    { SLAVE1_NODE_ID, 0x606C, 0, "actual vel"        },
    { SLAVE1_NODE_ID, 0x1400, 2, "RPDO1 tx type"     },
    { SLAVE2_NODE_ID, 0x6061, 0, "op mode display"   },
    { SLAVE2_NODE_ID, 0x6040, 0, "controlword (sdo)" },
    { SLAVE2_NODE_ID, 0x603F, 0, "error code"        },
    { SLAVE2_NODE_ID, 0x1003, 1, "last error"        },
    { SLAVE2_NODE_ID, 0x6041, 0, "statusword (sdo)"  },
    { SLAVE2_NODE_ID, 0x60FF, 0, "target vel"        },
    { SLAVE2_NODE_ID, 0x606C, 0, "actual vel"        },
    { SLAVE2_NODE_ID, 0x1400, 2, "RPDO1 tx type"     },
};

static const test_probe_t s_probe_pos[] = {
    { SLAVE1_NODE_ID, 0x6061, 0, "op mode display"   },
    { SLAVE1_NODE_ID, 0x6040, 0, "controlword (sdo)" },
    { SLAVE1_NODE_ID, 0x603F, 0, "error code"        },
    { SLAVE1_NODE_ID, 0x1003, 1, "last error"        },
    { SLAVE1_NODE_ID, 0x6041, 0, "statusword (sdo)"  },
    { SLAVE1_NODE_ID, 0x607A, 0, "target pos"        },
    { SLAVE1_NODE_ID, 0x6064, 0, "actual pos"        },
    { SLAVE1_NODE_ID, 0x1400, 2, "RPDO1 tx type"     },
    { SLAVE2_NODE_ID, 0x6061, 0, "op mode display"   },
    { SLAVE2_NODE_ID, 0x6040, 0, "controlword (sdo)" },
    { SLAVE2_NODE_ID, 0x603F, 0, "error code"        },
    { SLAVE2_NODE_ID, 0x1003, 1, "last error"        },
    { SLAVE2_NODE_ID, 0x6041, 0, "statusword (sdo)"  },
    { SLAVE2_NODE_ID, 0x607A, 0, "target pos"        },
    { SLAVE2_NODE_ID, 0x6064, 0, "actual pos"        },
    { SLAVE2_NODE_ID, 0x1400, 2, "RPDO1 tx type"     },
};

#define TEST_PROBE_VEL_COUNT ((uint32_t)(sizeof(s_probe_vel) / sizeof(s_probe_vel[0])))
#define TEST_PROBE_POS_COUNT ((uint32_t)(sizeof(s_probe_pos) / sizeof(s_probe_pos[0])))

/* 探针状态。放在文件作用域，是为了让 test_probe_restart() 能在每拍开始时复位。
 * 本头文件只被 main.c 包含一次，s_probe_* 表也是这么放的。 */
static const test_probe_t* s_probe_tab   = s_probe_vel;
static uint32_t            s_probe_count = TEST_PROBE_VEL_COUNT;
static uint32_t s_probe_step     = 0;   /* 下一条要读的；刚读回来的是 step-1 */
static uint32_t s_probe_deadline = 0;
static uint8_t  s_probe_pending  = 0;   /* 1 = 这一拍还要采一轮 */
/* 上一笔事务发给谁。取结果必须用【同一个节点】去问闸门 —— 表里两个节点的条目
 * 是交替的，若拿下一个条目的节点去问，闸门会判「不是我的事务」拒收，这笔结果
 * 就一直挂到 reap() 的 5 秒超时回收，整轮跟着冻住。 */
static uint8_t  s_probe_node     = 0;

/** @brief 每拍进入 RUN（SYNC 已发）时调一次：按本拍的模式选表，开一轮新的采样 */
static inline void test_probe_restart(bool_t pos_mode)
{
    s_probe_tab     = pos_mode ? s_probe_pos : s_probe_vel;
    s_probe_count   = pos_mode ? TEST_PROBE_POS_COUNT : TEST_PROBE_VEL_COUNT;
    s_probe_step    = 0;
    s_probe_node    = 0;
    s_probe_pending = 1;
}

/** @brief 推进探针，非阻塞。整轮采完自动停，等下一拍 restart */
static inline void test_probe_tick(uint32_t nowTick, const char* phase, uint32_t elapsed)
{
    co_master_status_t st = { CO_MASTER_OK, 0 };
    const test_probe_t* cur  = &s_probe_tab[s_probe_step % s_probe_count];
    const test_probe_t* prev;

    if (!s_probe_pending || (elapsed < TEST_PROBE_START_MS)) {
        return;     /* 不在采样期，或 SYNC 刚发下去、驱动器还没来得及应用 */
    }
    if (s_probe_node == 0U) {
        s_probe_node = cur->node_id;    /* 首次进来先对齐（节点号不可能是 0） */
    }

    switch (co_master_step_gate(s_probe_node, s_probe_deadline, &st)) {

    case CO_SDO_STEP_WAIT:
        return;     /* 上一条还在跑（或别人占着），等 */

    case CO_SDO_STEP_DONE:
        if (s_probe_step > 0U) {
            /* 数据必须在取走结果之后【立刻】读：发起下一条就把缓冲覆盖了 */
            size_t   len = co_master_get_size();
            uint32_t v   = 0;
            if (len >= 4U) {
                (void)co_master_get_u32(&v);
            } else if (len >= 2U) {
                uint16_t t = 0;
                (void)co_master_get_u16(&t);
                v = t;
            } else {
                uint8_t t = 0;
                (void)co_master_get_u8(&t);
                v = t;
            }
            prev = &s_probe_tab[(s_probe_step - 1U) % s_probe_count];
            printf("[SDO] %s +%lums node=%u 0x%04X.%02X %-16s = %ld (0x%08lX)\r\n",
                   phase, (unsigned long)elapsed,
                   (unsigned)prev->node_id, (unsigned)prev->index, (unsigned)prev->sub,
                   prev->name, (long)(int32_t)v, (unsigned long)v);
        }
        break;

    case CO_SDO_STEP_FAIL:
        /* 原因（对象 + abort 码）闸门已经打印过了。驱动器 OD 里没有这个子索引
         * 本身就是结论，记下来继续问下一条。 */
        if (s_probe_step > 0U) {
            prev = &s_probe_tab[(s_probe_step - 1U) % s_probe_count];
            printf("[SDO] %s +%lums node=%u 0x%04X.%02X %-16s = (read failed)\r\n",
                   phase, (unsigned long)elapsed,
                   (unsigned)prev->node_id, (unsigned)prev->index,
                   (unsigned)prev->sub, prev->name);
        }
        break;

    case CO_SDO_STEP_NEXT:
    default:
        break;      /* 空闲，可以发下一条 */
    }

    if (s_probe_step >= s_probe_count) {
        s_probe_pending = 0;    /* 这一拍采完了 */
        return;
    }
    if (!co_master_start_read(cur->node_id, cur->index, cur->sub, TEST_PROBE_TIMEOUT_MS)) {
        printf("[SDO] %s +%lums node=%u 0x%04X.%02X read start failed: %s\r\n",
               phase, (unsigned long)elapsed,
               (unsigned)cur->node_id, (unsigned)cur->index, (unsigned)cur->sub,
               co_master_start_error_name(co_master_get_start_error()));
    }
    s_probe_step++;
    s_probe_node     = cur->node_id;    /* 取这笔结果要用它去问闸门 */
    s_probe_deadline = nowTick + TEST_PROBE_TIMEOUT_MS + 100U;
}

/* ═══════════════════════════════════════════════════════════════
 *  测试主循环：在 main 的 while 里每周期调用（名字沿用 main.c 的调用点）
 * ═══════════════════════════════════════════════════════════════ */
static inline void test_mode_switch(uint32_t nowTick)
{
    typedef enum {
        T_WAIT = 0,     /* 等两从站都 RUNNING */
        T_SWITCH_REQ,   /* 为本拍请求模式切换（已在目标模式则跳过） */
        T_SWITCH_WAIT,  /* 等两轴回到 RUNNING */
        T_SETTLE,       /* 切换完成后等反馈稳定，再开拍 */
        T_IGNITE,       /* 速度拍：PV 启动沿预演，见文件头 */
        T_STAGE1,       /* 轴1 的新目标已下发，等 GAP 再装订轴2 */
        T_STAGE2,       /* 轴2 的新目标已下发，等 GAP 后发 SYNC */
        T_RUN,          /* SYNC 已发，本拍进行中 */
    } test_state_t;

    static test_state_t s_state = T_WAIT;
    static uint8_t      s_beat  = 0;                    /* 当前第几拍 */
    static cia402_mode_t s_cur_mode = CIA402_MODE_PROFILE_VEL;  /* 本测试请求过的模式。
                                                         * 初值必须与 main.c 里
                                                         * motor_ctrl_set_config 的
                                                         * 初始模式一致，否则第一次
                                                         * 会白切一次（能跑，只多花两秒）*/
    static uint8_t      s_ignite_step = 0;              /* T_IGNITE: 0=等沿走完, 1=等停稳 */
    static uint32_t     s_t0    = 0;                    /* 本阶段 / RUN 的起点 */
    static uint32_t     s_dbg   = 0;                    /* 上次采样时刻 */
    static uint32_t     s_refresh = 0;                  /* 上次重发暂存值的时刻 */
    static int32_t      s_pos0[TEST_SLAVE_COUNT] = { 0, 0 };  /* 进 RUN 时的实际位置基准 */
    static uint32_t     s_lag[TEST_SLAVE_COUNT] = { TEST_LAG_NONE, TEST_LAG_NONE };
    static uint8_t      s_lag_done = 0;                 /* 本拍同步差已报过 */

    const test_beat_t* b = &s_beats[s_beat];
    bool_t   pos_mode = test_beat_is_pos(b);
    uint32_t elapsed;
    int32_t  v[TEST_SLAVE_COUNT];
    uint8_t  i;

    /* 周期重发已装订轴的暂存设定值（见文件头：停发会让轴因 RPDO 超时掉使能）。
     * 预演 / 装订 / 运行三段都要；等模式切换和等反馈稳定的两段必须停 ——
     * 那时控制字归 402 状态机独占，插一帧进去就是打架。 */
    if ((s_state == T_IGNITE || s_state == T_STAGE1
         || s_state == T_STAGE2 || s_state == T_RUN)
        && (nowTick - s_refresh) >= TEST_REFRESH_MS) {
        s_refresh = nowTick;
        test_refresh_targets(test_beat_is_pos(&s_beats[s_beat]));
    }

    switch (s_state) {

    case T_WAIT:
        if (cia402_get_state(0) == CIA402_RUNNING
            && cia402_get_state(1) == CIA402_RUNNING) {
            printf("[T] both slaves RUNNING, %u beats: %s -> %s\r\n",
                   (unsigned)TEST_BEAT_COUNT, s_beats[0].name, s_beats[1].name);
            s_beat  = 0;
            s_state = T_SWITCH_REQ;
        } else if ((nowTick - s_dbg) >= TEST_WAIT_DBG_MS) {
            /* 卡住的时候必须说话：串口一片安静看不出是在等哪一个、
             * 卡在初始化还是卡在 402。 */
            s_dbg = nowTick;
            printf("[T] waiting enable: s1 init=%d/0x%04X state=%d/0x%04X | "
                   "s2 init=%d/0x%04X state=%d/0x%04X\r\n",
                   (int)co_init_get_state(0), (unsigned)co_init_get_fault_code(0),
                   (int)cia402_get_state(0), (unsigned)cia402_get_fault_code(0),
                   (int)co_init_get_state(1), (unsigned)co_init_get_fault_code(1),
                   (int)cia402_get_state(1), (unsigned)cia402_get_fault_code(1));
        }
        break;

    case T_SWITCH_REQ:
        if (s_cur_mode == b->mode) {
            /* 已经在目标模式（开机第一拍就是这条路），不用切 */
            s_t0    = nowTick;
            s_state = T_SETTLE;
            break;
        }
        if (cia402_get_state(0) != CIA402_RUNNING
            || cia402_get_state(1) != CIA402_RUNNING) {
            /* 还得等（多半在故障重试）。切模式只能在 RUNNING 下调。 */
            if ((nowTick - s_dbg) >= TEST_WAIT_DBG_MS) {
                s_dbg = nowTick;
                printf("[T] %s: wait RUNNING before switch: s1=%d/0x%04X s2=%d/0x%04X\r\n",
                       b->name, (int)cia402_get_state(0), (unsigned)cia402_get_fault_code(0),
                       (int)cia402_get_state(1), (unsigned)cia402_get_fault_code(1));
            }
            break;
        }
        for (i = 0U; i < TEST_SLAVE_COUNT; i++) {
            (void)cia402_switch_mode(i, b->mode);
        }
        printf("[T] %s: mode switch requested -> 0x6060 = %d\r\n",
               b->name, (int)b->mode);
        s_staged_mask = 0U;     /* 切换期间谁都不许重发 */
        s_t0    = nowTick;
        s_dbg   = nowTick;
        s_state = T_SWITCH_WAIT;
        break;

    case T_SWITCH_WAIT:
        if (cia402_get_state(0) == CIA402_RUNNING
            && cia402_get_state(1) == CIA402_RUNNING) {
            s_cur_mode = b->mode;
            printf("[T] %s: mode switch done\r\n", b->name);
            s_t0    = nowTick;
            s_state = T_SETTLE;
            break;
        }
        if ((nowTick - s_t0) >= TEST_SWITCH_TIMEOUT_MS) {
            /* 说清楚卡在哪，然后回 T_SWITCH_REQ 重来，别把测试挂死 */
            printf("[T] %s: mode switch TIMEOUT: s1=%d/0x%04X s2=%d/0x%04X\r\n",
                   b->name, (int)cia402_get_state(0), (unsigned)cia402_get_fault_code(0),
                   (int)cia402_get_state(1), (unsigned)cia402_get_fault_code(1));
            s_t0    = nowTick;
            s_dbg   = nowTick;
            s_state = T_SWITCH_REQ;
            break;
        }
        if ((nowTick - s_dbg) >= TEST_WAIT_DBG_MS) {
            s_dbg = nowTick;
            printf("[T] %s: switching... s1=%d/0x%04X s2=%d/0x%04X\r\n",
                   b->name, (int)cia402_get_state(0), (unsigned)cia402_get_fault_code(0),
                   (int)cia402_get_state(1), (unsigned)cia402_get_fault_code(1));
        }
        break;

    case T_SETTLE:
        if ((nowTick - s_t0) < TEST_SETTLE_MS) {
            break;
        }
        if (pos_mode) {
            /* 位置拍的反馈（0x6064）现在才是真的，从这里开始装订 */
            test_beat_begin(b);
            test_print_stage(b, 0U, 0U);
            s_t0      = nowTick;
            s_refresh = nowTick;
            s_state   = T_STAGE1;
        } else {
            /* 速度拍先预演启动沿，见文件头 */
            test_ignite_begin();
            s_ignite_step = 0U;
            s_t0      = nowTick;
            s_refresh = nowTick;
            s_state   = T_IGNITE;
        }
        break;

    case T_IGNITE: {
        /* ── 预演阶段，不产生测量数据 ── */
        if (s_ignite_step == 0U) {
            bool_t ready = (bool_t)(cia402_pv_ready(0U) && cia402_pv_ready(1U));

            if (!ready && (nowTick - s_t0) < TEST_IGNITE_TIMEOUT_MS) {
                break;      /* 沿还在走（每步 20ms，六步 ≈120ms），等着 */
            }
            if (!ready) {
                /* 不能把测试挂死在这儿，但要把话说清楚：多半是某一轴掉出了
                 * RUNNING，或沿在第 2 步被撤回目标打断了。 */
                printf("[T] ignite TIMEOUT: s1 state=%d ready=%d | s2 state=%d ready=%d\r\n",
                       (int)cia402_get_state(0), (int)cia402_pv_ready(0U),
                       (int)cia402_get_state(1), (int)cia402_pv_ready(1U));
            }

            /* 两轴清 0 + 一条 SYNC：把两轴【一起】停住，摆回已知状态。
             * 清 0 也是 RPDO 数据，同样要等 SYNC 才落盘，所以 SYNC 必须在写完之后。 */
            for (i = 0U; i < TEST_SLAVE_COUNT; i++) {
                s_stage_rpm[i] = 0;
                cia402_set_velocity(i, 0);
            }
            co_sync_send();
            printf("[T] PV start edge %s, stop + SYNC, settle %ums\r\n",
                   ready ? "done on both axes" : "INCOMPLETE",
                   (unsigned)TEST_IGNITE_SETTLE_MS);
            s_ignite_step = 1U;
            s_t0      = nowTick;
            s_refresh = nowTick;
            break;
        }

        if ((nowTick - s_t0) < TEST_IGNITE_SETTLE_MS) {
            break;          /* 等轴真的停稳，别让预演残留的转速糊了第一拍的判据 */
        }

        /* ── 从这里开始才产生测量数据 ── */
        printf("[T] start %s beat\r\n", b->name);
        test_beat_begin(b);
        test_print_stage(b, 0U, 0U);
        s_t0      = nowTick;
        s_refresh = nowTick;
        s_state   = T_STAGE1;
        break;
    }

    case T_STAGE1:
        /* 等轴1 的新目标真的上了总线，再装订轴2。这段间隔就是「轴1 拿着新目标
         * 干等」的那段时间，也是判据的一半 —— 打印里它应该还是旧值。 */
        if ((nowTick - s_t0) >= TEST_STAGE_GAP_MS) {
            test_stage_axis(1U, b);
            test_print_stage(b, 1U, TEST_STAGE_GAP_MS);
            s_t0    = nowTick;
            s_state = T_STAGE2;
        }
        break;

    case T_STAGE2:
        if ((nowTick - s_t0) >= TEST_STAGE_GAP_MS) {
            /* 两轴的新目标都已经在总线上放了 GAP ms。这一刻读回来还该是旧值：
             * 从站把它们收在同步缓冲区里，还没写进自己的对象字典。 */
            if (pos_mode) {
                printf("[T] %s: SYNC @+%lums | before: s1 pos=%+ld s2 pos=%+ld\r\n",
                       b->name, (unsigned long)(TEST_STAGE_GAP_MS * 2U),
                       (long)cia402_get_actual_position(0),
                       (long)cia402_get_actual_position(1));
            } else {
                printf("[T] %s: SYNC @+%lums | before: s1 vel=%+ld s2 vel=%+ld\r\n",
                       b->name, (unsigned long)(TEST_STAGE_GAP_MS * 2U),
                       (long)cia402_get_actual_velocity(0),
                       (long)cia402_get_actual_velocity(1));
            }
            co_sync_send();
            s_t0       = nowTick;
            s_dbg      = nowTick;
            s_refresh  = nowTick;
            s_lag[0]   = s_lag[1] = TEST_LAG_NONE;
            s_lag_done = 0;
            for (i = 0U; i < TEST_SLAVE_COUNT; i++) {
                s_pos0[i] = cia402_get_actual_position(i);
            }
            printf("[T] %s: SYNC sent (both axes take effect now)\r\n", b->name);
            test_probe_restart(pos_mode);   /* 本拍采一轮驱动器内部参数 */
            s_state = T_RUN;
        }
        break;

    case T_RUN:
        elapsed = nowTick - s_t0;

        /* 本拍（SYNC 之后）问一遍驱动器自己的参数，一轮而已 */
        test_probe_tick(nowTick, b->name, elapsed);

        if ((nowTick - s_dbg) >= TEST_POLL_MS) {
            s_dbg = nowTick;
            v[0]  = cia402_get_actual_velocity(0);
            v[1]  = cia402_get_actual_velocity(1);
            if (pos_mode) {
                printf("[T] %s +%lums | s1 sts=0x%04X pos=%+ld/%+ld | "
                       "s2 sts=0x%04X pos=%+ld/%+ld\r\n",
                       b->name, (unsigned long)elapsed,
                       (unsigned)cia402_get_statusword(0),
                       (long)cia402_get_actual_position(0), (long)s_stage_pos[0],
                       (unsigned)cia402_get_statusword(1),
                       (long)cia402_get_actual_position(1), (long)s_stage_pos[1]);
            } else {
                printf("[T] %s +%lums | s1 sts=0x%04X vel=%+ld | s2 sts=0x%04X vel=%+ld\r\n",
                       b->name, (unsigned long)elapsed,
                       (unsigned)cia402_get_statusword(0), (long)v[0],
                       (unsigned)cia402_get_statusword(1), (long)v[1]);
            }

            /* 记下各轴第一次「动起来」的时刻，两者之差就是这条 SYNC 的实际
             * 同步误差。速度拍看转速离开 0，位置拍看位置离开基准超过门槛。 */
            for (i = 0U; i < TEST_SLAVE_COUNT; i++) {
                bool_t moved;
                if (s_lag[i] != TEST_LAG_NONE) {
                    continue;
                }
                if (pos_mode) {
                    int32_t d = cia402_get_actual_position(i) - s_pos0[i];
                    if (d < 0) { d = -d; }
                    moved = (bool_t)(d >= TEST_POS_MOVED_EPS);
                } else {
                    moved = (bool_t)(v[i] != 0);
                }
                if (moved) { s_lag[i] = elapsed; }
            }
            if (!s_lag_done
                && s_lag[0] != TEST_LAG_NONE && s_lag[1] != TEST_LAG_NONE) {
                s_lag_done = 1;
                printf("[T] %s: sync lag s1=%lums s2=%lums (diff %ldms)\r\n",
                       b->name,
                       (unsigned long)s_lag[0], (unsigned long)s_lag[1],
                       (long)((int32_t)s_lag[0] - (int32_t)s_lag[1]));
            }
        }

        if (elapsed >= b->hold_ms) {
            /* 本拍收尾。没测到期望结果的那一轴要明说 —— 过程数据恒 0 说不清是
             * 驱动器没收到 RPDO，还是从站 TPDO1 里根本没有那个对象。 */
            for (i = 0U; i < TEST_SLAVE_COUNT; i++) {
                if (s_lag[i] == TEST_LAG_NONE) {
                    printf("[T] %s: slave%u never moved (process data stayed at "
                           "the old value)\r\n", b->name, (unsigned)(i + 1U));
                }
            }
            if (pos_mode) {
                printf("[T] %s: done | target reached s1=%d s2=%d\r\n", b->name,
                       (int)cia402_position_reached(0), (int)cia402_position_reached(1));
            }

            /* 下一拍：先请求切模式，切完再开拍 */
            s_beat  = (uint8_t)((s_beat + 1U) % TEST_BEAT_COUNT);
            s_t0    = nowTick;
            s_dbg   = nowTick;
            s_refresh = nowTick;
            s_state = T_SWITCH_REQ;
        }
        break;
    }
}

#endif /* _APP_TEST_H_ */
