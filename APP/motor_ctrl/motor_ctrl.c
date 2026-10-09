#include "motor_ctrl.h"
#include "canopen_init.h"   /* co_init_start, co_init_process, co_init_done */
#include "cia402.h"         /* cia402_init, cia402_process */
#include "app_log.h"

#include <stdio.h>

/* ── 私有状态 ── */
static CO_t* s_co = NULL;
static const cia402_slave_config_t* s_cfgs[CIA402_MAX_SLAVES];
static cia402_mode_t s_initial_modes[CIA402_MAX_SLAVES];
static bool_t s_cia402_started[CIA402_MAX_SLAVES];
static bool_t s_slave_used[CIA402_MAX_SLAVES];

void motor_ctrl_set_config(uint8_t idx, CO_t* co, const cia402_slave_config_t* cfg, cia402_mode_t initial_mode)
{
    if (idx >= CIA402_MAX_SLAVES) return;
    s_co = co;
    s_cfgs[idx] = cfg;
    s_initial_modes[idx] = initial_mode;
    s_cia402_started[idx] = false;
    s_slave_used[idx] = true;

    /* 按初始模式选择 PDO 配置，启动 CANopen 通用初始化 */
    const co_pdo_config_t* pdo_cfg = (initial_mode == CIA402_MODE_PROFILE_VEL)
        ? cfg->vel_cfg
        : cfg->pos_cfg;
    co_init_start(idx, co, cfg->node_id, pdo_cfg);

    LOGI("motor_ctrl: idx=%u set config, node=%u, mode=%d\r\n", idx, cfg->node_id, initial_mode);
}

void motor_ctrl_process(void)
{
    for (uint8_t idx = 0; idx < CIA402_MAX_SLAVES; idx++) {
        if (!s_slave_used[idx]) continue;

        /* 阶段1：推进 CANopen 通用初始化 */
        co_init_process(idx);

        /* 阶段2：CANopen 初始化完成后启动 402 状态机
         *        此时心跳消费者已注册完毕，cia402_init 可正常获取 hb_idx */
        if (co_init_done(idx) && !s_cia402_started[idx]) {
            cia402_init(idx, s_co, s_cfgs[idx], s_initial_modes[idx]);
            s_cia402_started[idx] = true;
            LOGI("motor_ctrl: idx=%u CiA402 started\r\n", idx);
        }

        /* 阶段3：推进 402 状态机 */
        if (s_cia402_started[idx]) {
            cia402_process(idx);
        }
    }
}