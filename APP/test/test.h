/* APP/test/test.h — 两从站【同步速度模式】测试（PV + 同步 RPDO）
 *
 * 数据流（速度模式，配置见 user_pdo_cfg_slave.h 的 *_vel_config）：
 *   主站 TPDO1/2 (0x202/0x204) → 从站 RPDO1 : 控制字 0x6040 + 目标速度 0x60FF
 *   从站 TPDO1   (0x182/0x184) → 主站 RPDO1/2 : 状态字 0x6041 + 实际速度 0x606C
 *
 * 为什么叫「同步」：
 *   _vel_config 里 use_sync = true，初始化时从站 RPDO1 的传输类型被写成 0x01
 *   （同步接收，见 canopen_init.c 的 SET_TX_TYPE 步骤）。0x01 的含义是：从站
 *   收到 RPDO 只把它缓存起来，收到 SYNC 才写进对象字典 —— 两轴因此会在同一条
 *   SYNC 上同时起步。
 *   当前 slave1 是 true、slave2 临时改成 false（0xFE 立即生效）做对照：
 *   「同步接收」已被证伪为 node4 不转的原因（改成立即生效后依旧不转），
 *   结论出来后再决定要不要改回。
 *
 * 每一拍的顺序不能颠倒：
 *   ① 写目标（cia402_set_velocity 内部会请求 TPDO）
 *   ② 等这两帧真的发上总线
 *   ③ 发 SYNC
 * 少了②，SYNC 可能先于 TPDO 出总线，从站在 SYNC 上用到的还是【上一拍】的缓存，
 * 表现为「起步慢一拍」。TEST_ARM_MS 就是留给②的时间。
 *
 * 判定依据：
 *   - 状态字 0x6041：bit2 = Operation Enabled，bit10 = 速度到达
 *   - 实际速度 0x606C：由从站 TPDO1 上报、主站 RPDO 收下，落在 OD_RAM
 *     0x2005 / 0x2015。它要是恒为 0 而电机在转，说明从站 TPDO1 里并没有
 *     0x606C（映射没生效），而不是电机没动。
 *
 * 另外带一段【运行时 SDO 探针】（往下翻，定位完可以整块删掉）：PDO 是单向的，
 * 帧发出去之后就再也看不到它了，而「电机不转」既可能是帧没发对，也可能是驱动器
 * 收到了却按它自己的意思解释 —— 看主站这边的状态量分不出来。探针直接问驱动器
 * 自己的视角（目标速度 / 实际速度 / 模式 / 限幅 / 齿轮比），这是唯一能分开这两种
 * 情况的办法。当前这项已经证明帧是对的，所以探针改成对照两台驱动器的内部参数。
 *
 * 拍表（见 s_test_phases）按「哪一轴动」排了对照：slave1 单独 → slave2 单独 →
 * 两轴同时 → 反转 → 给 slave2 一个远超上限的目标。一次运行就能回答
 * 「好轴在同一份代码下还转不转」（FWD-s1）和「坏轴到底执不执行设定值」（FAST-s2）。
 *
 * 本文件只做测试，随便改。
 */
#ifndef _APP_TEST_H_
#define _APP_TEST_H_

#include "slave_config.h"   /* SLAVE1/2_PULSE_PER_REV */
#include "cia402.h"
#include "canopen_init.h"   /* co_sync_send / co_init_get_state */
#include "canopen_master.h" /* 运行时探针用的 SDO 客户端闸门 */

/* ── 测试参数 ─────────────────────────────────────────────────────── */
#define TEST_RPM            60          /* 每拍目标转速(rpm)，负号 = 反转 */
#define TEST_RPM_FAST       600         /* 远超驱动器上限的目标，见 FAST-s2 */
#define TEST_RUN_MS         3000U       /* 正转/反转保持时间 */
#define TEST_STOP_MS        2000U       /* 两拍之间的停止时间 */
#define TEST_ARM_MS         50U         /* 写目标 → 发 SYNC 的间隔 */
#define TEST_REFRESH_MS     100U        /* 运行中重发目标的周期，见 test_write_targets */
#define TEST_POLL_MS        250U        /* 运行中的采样/打印周期 */
#define TEST_WAIT_DBG_MS    1000U       /* 等使能时的诊断打印周期 */

/* 本测试固定用两个从站（cia402 的数组上限是 CIA402_MAX_SLAVES，不是从站数） */
#define TEST_SLAVE_COUNT    2U

#define TEST_LAG_NONE       0xFFFFFFFFu /* 「本拍还没测到」的哨兵值 */

/* ── 一拍的内容：哪一轴动、转多快、转多久 ───────────────────────────
 * 轴掩码让「谁在动」成为每一拍的属性，于是一次运行里就能做齐对照：
 *   FWD-s1    已知能转的轴单独动 —— 同时也是「我改的东西有没有把好轴改坏」的对照
 *   FWD-s2    不转的轴单独动，条件与上一拍完全相同，只换了轴
 *   SYNC-both 两轴同时动（最初要的同步测试；也只有两轴都动才谈得上同步误差）
 *   FAST-s2   给 node4 一个远超驱动器上限的目标（50 万 pps）：它只要真在按 0x60FF
 *             执行，就必然被限幅后冲起来；纹丝不动 = 它根本没在执行这个量 */
#define TEST_AXIS_NONE 0x00U
#define TEST_AXIS_S1   0x01U
#define TEST_AXIS_S2   0x02U
#define TEST_AXIS_BOTH (TEST_AXIS_S1 | TEST_AXIS_S2)

typedef struct {
    const char* name;       /* 打印用 */
    int32_t     rpm;        /* 0 = 停 */
    uint32_t    hold_ms;    /* 保持时间 */
    uint8_t     axes;       /* 本拍要动哪些轴（位掩码），其余轴一律写 0 */
} test_phase_t;

static const test_phase_t s_test_phases[] = {
    { "STOP",       0,             TEST_STOP_MS, TEST_AXIS_NONE },
    { "FWD-s1",     TEST_RPM,      TEST_RUN_MS,  TEST_AXIS_S1   },
    { "STOP",       0,             TEST_STOP_MS, TEST_AXIS_NONE },
    { "FWD-s2",     TEST_RPM,      TEST_RUN_MS,  TEST_AXIS_S2   },
    { "STOP",       0,             TEST_STOP_MS, TEST_AXIS_NONE },
    { "SYNC-both",  TEST_RPM,      TEST_RUN_MS,  TEST_AXIS_BOTH },
    { "STOP",       0,             TEST_STOP_MS, TEST_AXIS_NONE },
    { "REV-s2",    -TEST_RPM,      TEST_RUN_MS,  TEST_AXIS_S2   },
    { "STOP",       0,             TEST_STOP_MS, TEST_AXIS_NONE },
    { "FAST-s2",    TEST_RPM_FAST, TEST_RUN_MS,  TEST_AXIS_S2   },
};
#define TEST_PHASE_COUNT (sizeof(s_test_phases) / sizeof(s_test_phases[0]))

/** @brief 本拍第 idx 轴要不要动 */
static inline uint8_t test_axis_in_phase(const test_phase_t* ph, uint8_t idx)
{
    return (uint8_t)((ph->axes >> idx) & 1U);
}

static inline const char* test_axis_name(uint8_t axes)
{
    switch (axes) {
    case TEST_AXIS_S1:   return "slave1(node2) only";
    case TEST_AXIS_S2:   return "slave2(node4) only";
    case TEST_AXIS_BOTH: return "slave1+slave2 (sync)";
    default:             return "none (stop)";
    }
}

/* ── 把本拍的目标速度写给各轴 ────────────────────────────────────────
 * cia402_set_velocity() 内部换算 rpm → 脉冲/秒，写控制字 0x000F + 目标速度，
 * 并请求各自的 TPDO。此时从站只是把它们缓存进同步缓冲区，还没生效。
 *
 * 本拍不动的轴写 0（而不是不管它）：它因此会一直收到控制字、保持
 * Operation Enabled，成为「同样使能、只有设定值不同」的对照。
 * 不管它的话它会因为收不到 RPDO 而退出使能 —— 日志里 slave1 掉到 0x0670
 * （Switch On Disabled）就是这么来的，两边状态不一样就没法比。
 *
 * 为什么要在本拍里每 TEST_REFRESH_MS 重发一次：
 *   主站 TPDO 的传输类型是 0xFE（映射数据变化时才发）+ 事件定时器 0，
 *   所以「一拍只写一次目标」等于整拍只上总线一帧 0x204。带 RPDO 超时保护
 *   的驱动器（node4 那台就是：设定值锁进了 OD、机器却停着、0x606C 恒 0、
 *   不报错、一直发热）收到一帧后几十毫秒就自己把轴停了。
 *   「单轴测试」分支里 node4 能转的那份 test 就是每 100ms 重发一次，这里
 *   用同一个周期，把「发送频率」这一个变量对齐后再比。
 */
static inline void test_write_targets(const test_phase_t* ph)
{
    for (uint8_t i = 0U; i < TEST_SLAVE_COUNT; i++) {
        cia402_set_velocity(i, test_axis_in_phase(ph, i) ? ph->rpm : 0);
    }
}

/** @brief 写目标 + 打印本拍内容（进拍时用；运行中的周期重发只调上面那个） */
static inline void test_arm_phase(const test_phase_t* ph)
{
    test_write_targets(ph);
    /* 两个轴的 pulse_per_rev 都是 SLAVE1_PULSE_PER_REV(50000)，换算只有一个数 */
    printf("[PV] %s: target %+ld rpm (%ld pps) queued on %s\r\n",
           ph->name, (long)ph->rpm,
           (long)((int64_t)ph->rpm * SLAVE1_PULSE_PER_REV / 60),
           test_axis_name(ph->axes));
}

/* ═══════════════════════════════════════════════════════════════
 *  运行时 SDO 探针（只读诊断，定位完整块删掉）
 * ═══════════════════════════════════════════════════════════════
 * 到上一轮为止，主站这一侧、以及驱动器所有标准配置对象，两台完全一样：
 *   - 0x204 在总线上，且驱动器【自己】的对象字典里 0x60FF 就是 50000
 *   - 控制字通路也通（收到 RPDO 的保持使能，收不到的那台掉到 0x0670）
 *   - 0x6061 模式显示两台都是 3（真正的速度模式）
 *   - 0x607F/0x6072/0x6073/0x6091 两台都【不存在】，没有可比的差别
 *   - 0x606C 是问驱动器本人读回来的 0，电机确实没转，不是反馈没回来
 * 所以这一轮不再比配置，改读「它自己怎么说」：错误码和错误历史。
 * 加上 0x6041 走 SDO 读一遍，跟 TPDO 收到的状态字对照，确认反馈通道没骗我们。
 *
 * 一轮（2 节点 × 8 条）只在每拍 SYNC 之后采一次：整拍里状态是不变的，
 * 采一次就够，日志也不会被刷屏。
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

/* slave1(node2) 是已知能转的，slave2(node4) 是不转的。同样的对象问两台，
 * 读数不一样的那一项就是答案。
 *
 * 本轮新增三条，专门盯「同步接收 RPDO」这个变量：
 *   - 0x6040 控制字：验证驱动器锁存的控制字真是 0x000F（而不是靠 SDO 使能
 *     撑出来的 Operation Enabled）；
 *   - 0x1400.02：验证 slave2_vel_config.use_sync=false 是否真的把从站 RPDO1
 *     的传输类型从 1 改回了 0xFE；
 *   - 0x1003.01：node4 状态字 bit4（电压使能）= 0，错误史里最近一条是什么。 */
static const test_probe_t s_test_probe[] = {
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
#define TEST_PROBE_COUNT (sizeof(s_test_probe) / sizeof(s_test_probe[0]))

/* 探针状态。放在文件作用域，是为了让 test_probe_restart() 能在每拍开始时复位。
 * 本头文件只被 main.c 包含一次，s_test_probe 表也是这么放的。 */
static uint32_t s_probe_step     = 0;   /* 下一条要读的；刚读回来的是 step-1 */
static uint32_t s_probe_deadline = 0;
static uint8_t  s_probe_pending  = 0;   /* 1 = 这一拍还要采一轮 */
/* 上一笔事务发给谁。取结果必须用【同一个节点】去问闸门 —— 表里两个节点的条目
 * 是交替的，若拿下一个条目的节点去问，闸门会判「不是我的事务」拒收，这笔结果
 * 就一直挂到 reap() 的 5 秒超时回收，整轮跟着冻住。 */
static uint8_t  s_probe_node     = 0;

/** @brief 每拍进入 RUN（SYNC 已发）时调一次：开一轮新的采样 */
static inline void test_probe_restart(void)
{
    s_probe_step    = 0;
    s_probe_node    = 0;
    s_probe_pending = 1;
}

/** @brief 推进探针，非阻塞。整轮采完自动停，等下一拍 restart */
static inline void test_probe_tick(uint32_t nowTick, const char* phase, uint32_t elapsed)
{
    co_master_status_t st = { CO_MASTER_OK, 0 };
    const test_probe_t* cur  = &s_test_probe[s_probe_step % TEST_PROBE_COUNT];
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
            prev = &s_test_probe[(s_probe_step - 1U) % TEST_PROBE_COUNT];
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
            prev = &s_test_probe[(s_probe_step - 1U) % TEST_PROBE_COUNT];
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

    if (s_probe_step >= TEST_PROBE_COUNT) {
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

/* ── 测试函数：在主循环每周期调用（名字沿用 main.c 的调用点） ─────── */
static inline void test_mode_switch(uint32_t nowTick)
{
    typedef enum {
        T_WAIT = 0,     /* 等两从站都 RUNNING（已经在速度模式） */
        T_ARM,          /* 目标已下发，等 TPDO 上总线 */
        T_RUN,          /* SYNC 已发，本拍进行中 */
    } test_state_t;

    static test_state_t s_state = T_WAIT;
    static uint8_t      s_phase = 0;                            /* 当前第几拍 */
    static uint32_t     s_t0    = 0;                            /* ARM 起点 / SYNC 时刻 */
    static uint32_t     s_dbg   = 0;                            /* 上次采样时刻 */
    static uint32_t     s_refresh = 0;                          /* 上次重发目标的时刻 */
    static uint32_t     s_lag[TEST_SLAVE_COUNT] = { TEST_LAG_NONE, TEST_LAG_NONE };
    static uint8_t      s_lag_done = 0;                         /* 本拍同步差已报过 */

    const test_phase_t* ph = &s_test_phases[s_phase];
    uint32_t elapsed;
    int32_t  v[TEST_SLAVE_COUNT];
    uint8_t  i;

    switch (s_state) {

    case T_WAIT:
        if (cia402_get_state(0) == CIA402_RUNNING
            && cia402_get_state(1) == CIA402_RUNNING) {
            printf("[PV] both slaves RUNNING (velocity mode), start sync velocity test\r\n");
            s_phase    = 0;
            s_lag_done = 0;
            s_lag[0]   = s_lag[1] = TEST_LAG_NONE;
            test_arm_phase(&s_test_phases[s_phase]);
            s_t0    = nowTick;
            s_state = T_ARM;
        } else if ((nowTick - s_dbg) >= TEST_WAIT_DBG_MS) {
            /* 卡住的时候必须说话：串口一片安静看不出是在等哪一个、
             * 卡在初始化还是卡在 402。 */
            s_dbg = nowTick;
            printf("[PV] waiting enable: s1 init=%d/0x%04X state=%d/0x%04X | "
                   "s2 init=%d/0x%04X state=%d/0x%04X\r\n",
                   (int)co_init_get_state(0), (unsigned)co_init_get_fault_code(0),
                   (int)cia402_get_state(0), (unsigned)cia402_get_fault_code(0),
                   (int)co_init_get_state(1), (unsigned)co_init_get_fault_code(1),
                   (int)cia402_get_state(1), (unsigned)cia402_get_fault_code(1));
        }
        break;

    case T_ARM:
        /* 等两个 TPDO 真的出总线，再发 SYNC（见文件头「顺序不能颠倒」） */
        if ((nowTick - s_t0) >= TEST_ARM_MS) {
            co_sync_send();
            s_t0       = nowTick;
            s_dbg      = nowTick;
            s_refresh  = nowTick;
            s_lag[0]   = s_lag[1] = TEST_LAG_NONE;
            s_lag_done = 0;
            printf("[PV] %s: SYNC sent -> %s take effect now\r\n",
                   ph->name, test_axis_name(ph->axes));
            test_probe_restart();   /* 本拍采一轮驱动器内部参数 */
            s_state = T_RUN;
        }
        break;

    case T_RUN:
        elapsed = nowTick - s_t0;

        /* 周期重发目标：让 0x202/0x204 在本拍里持续上总线，
         * 而不是「值变了才发一帧」（见 test_write_targets 的说明） */
        if ((nowTick - s_refresh) >= TEST_REFRESH_MS) {
            s_refresh = nowTick;
            test_write_targets(ph);
        }

        /* 本拍（SYNC 之后）问一遍驱动器自己的参数，一轮而已 */
        test_probe_tick(nowTick, ph->name, elapsed);

        if ((nowTick - s_dbg) >= TEST_POLL_MS) {
            s_dbg = nowTick;
            v[0]  = cia402_get_actual_velocity(0);
            v[1]  = cia402_get_actual_velocity(1);
            printf("[PV] %s +%lums | s1 sts=0x%04X vel=%+ld | s2 sts=0x%04X vel=%+ld\r\n",
                   ph->name, (unsigned long)elapsed,
                   (unsigned)cia402_get_statusword(0), (long)v[0],
                   (unsigned)cia402_get_statusword(1), (long)v[1]);

            /* 记下各【在动的】轴第一次「动起来」（转速拍）或「停稳」（停车拍）
             * 的时刻，两者之差就是这条 SYNC 的实际同步误差。本拍不动的轴不参与，
             * 所以单轴拍永远不会报这个差（它本来也没意义），只有 SYNC-both 才有。 */
            for (i = 0U; i < TEST_SLAVE_COUNT; i++) {
                if (!test_axis_in_phase(ph, i) || (s_lag[i] != TEST_LAG_NONE)) {
                    continue;
                }
                if (ph->rpm != 0) {
                    if (v[i] != 0) { s_lag[i] = elapsed; }
                } else {
                    if (v[i] == 0) { s_lag[i] = elapsed; }
                }
            }
            if (!s_lag_done
                && s_lag[0] != TEST_LAG_NONE && s_lag[1] != TEST_LAG_NONE) {
                s_lag_done = 1;
                printf("[PV] %s: sync lag s1=%lums s2=%lums (diff %ldms)\r\n",
                       ph->name,
                       (unsigned long)s_lag[0], (unsigned long)s_lag[1],
                       (long)((int32_t)s_lag[0] - (int32_t)s_lag[1]));
            }
        }

        if (elapsed >= ph->hold_ms) {
            /* 本拍收尾。没测到期望结果的那一轴要明说 —— vel 恒 0 说不清是
             * 驱动器没收到 RPDO，还是从站 TPDO1 里根本没有 0x606C。 */
            for (i = 0U; i < TEST_SLAVE_COUNT; i++) {
                if (!test_axis_in_phase(ph, i) || (s_lag[i] != TEST_LAG_NONE)) {
                    continue;
                }
                printf("[PV] %s: slave%u %s (feedback vel stayed %s)\r\n",
                       ph->name, (unsigned)(i + 1U),
                       (ph->rpm != 0) ? "never moved" : "never stopped",
                       (ph->rpm != 0) ? "0" : "non-zero");
            }

            s_phase = (uint8_t)((s_phase + 1U) % TEST_PHASE_COUNT);
            test_arm_phase(&s_test_phases[s_phase]);
            s_t0    = nowTick;
            s_state = T_ARM;
        }
        break;
    }
}

#endif /* _APP_TEST_H_ */
