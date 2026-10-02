/* APP/test.h — 模式切换测试：速度→位置→速度循环 */
#ifndef _APP_TEST_H_
#define _APP_TEST_H_

#include "slave_config.h"   /* 从站配置（业务层用，测试只是引用） */

/* ── 测试参数 ─────────────────────────────────────────────────────── */
#define TEST_VEL_RPM          500       /* 速度模式转速(rpm) */
#define TEST_VEL_DURATION_MS  5000U    /* 速度模式持续时间 */
#define TEST_VEL_SEND_MS      100U      /* 速度指令下发周期（不要每周期都发） */
#define TEST_POS_REV          25        /* 位置模式目标圈数 */
#define TEST_POS_HOLD_MS      5000U     /* 位置到达后保持时间 */

/* 位置模式用哪种 402 模式：
 *   CIA402_MODE_PROFILE_POS → 驱动器自己规划，主机只发最终目标（默认，先验证这条）
 *   CIA402_MODE_CSP         → 主机用 s_curve 逐点插值下发（要先给驱动器配好
 *                             0x60C2 插值周期=10ms，否则驱动器按自己的默认周期
 *                             插值，会持续滞后/超差报警）
 * 换 CSP 前请先确认 DM556-CAN 支持模式 8。 */
#define TEST_POS_MODE         CIA402_MODE_PROFILE_POS

/* ── 测试函数：在主循环每周期调用 ─────────────────────────────────── */
static inline void test_mode_switch(uint32_t nowTick)
{
    typedef enum { T_VEL = 0, T_POS, T_WAIT, T_HOLD } test_state_t;
    static test_state_t s_state = T_VEL;
    static uint32_t s_tick = 0;

    if (cia402_get_state() != CIA402_RUNNING) return;

    switch (s_state) {

    case T_VEL: {
        /* 按固定周期下发即可。以前每周期都调 cia402_set_velocity()，
         * 里面无条件 CO_TPDOsendRequest()，等于拿主循环频率(≈1ms)刷总线。 */
        static uint32_t s_velTick = 0;
        if (s_tick == 0) { s_tick = nowTick; s_velTick = 0; }
        if (s_velTick == 0 || (nowTick - s_velTick) >= TEST_VEL_SEND_MS) {
            s_velTick = nowTick;
            cia402_set_velocity(TEST_VEL_RPM);
        }
        if ((nowTick - s_tick) >= TEST_VEL_DURATION_MS) {
            printf("[TEST] velocity %urpm done, switch to position\r\n", TEST_VEL_RPM);
            if (cia402_switch_mode(TEST_POS_MODE)) {
                s_state = T_POS; s_tick = 0;
            }
        }
        break;
    }

    case T_POS:
        cia402_set_position_rel((int32_t)(TEST_POS_REV * SLAVE1_PULSE_PER_REV));
        printf("[TEST] position rel=%d (rev=%d) sent\r\n",
               (int)(TEST_POS_REV * SLAVE1_PULSE_PER_REV), TEST_POS_REV);
        s_state = T_WAIT;
        break;

    case T_WAIT: {
        static uint32_t s_dbgTick = 0;
        if ((nowTick - s_dbgTick) >= 1000u) {
            s_dbgTick = nowTick;
            printf("[POS] sts=0x%04X actPos=%ld\r\n",
                   cia402_get_statusword(), (long)cia402_get_actual_position());
        }
        if (cia402_position_reached()) {
            printf("[POS] target reached! actPos=%ld\r\n",
                   (long)cia402_get_actual_position());
            s_state = T_HOLD;
            s_tick = nowTick;
        }
        break;
    }

    case T_HOLD:
        if ((nowTick - s_tick) >= TEST_POS_HOLD_MS) {
            printf("[TEST] position reached, back to velocity\r\n");
            cia402_switch_mode(CIA402_MODE_PROFILE_VEL);
            s_state = T_VEL;
            s_tick = 0;
        }
        break;
    }
}

#endif /* _APP_TEST_H_ */