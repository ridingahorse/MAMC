#include "motor_ctrl.h"
#include "canopen_init.h"   /* co_init_start, co_init_process, co_init_done */
#include "cia402.h"         /* cia402_init, cia402_process */

#include <stdio.h>

/* ── 私有状态 ── */
static CO_t* s_co = NULL;
static const cia402_slave_config_t* s_cfg = NULL;
static cia402_mode_t s_initial_mode = CIA402_MODE_PROFILE_VEL;
static bool_t s_cia402_started = false;

void motor_ctrl_set_config(CO_t* co, const cia402_slave_config_t* cfg, cia402_mode_t initial_mode)
{
    s_co = co;
    s_cfg = cfg;
    s_initial_mode = initial_mode;
    s_cia402_started = false;

    /* 按初始模式选择 PDO 配置，启动 CANopen 通用初始化 */
    const co_pdo_config_t* pdo_cfg = (initial_mode == CIA402_MODE_PROFILE_VEL)
        ? cfg->vel_cfg
        : cfg->pos_cfg;
    co_init_start(co, cfg->node_id, pdo_cfg);

    printf("motor_ctrl: set config, node=%u, mode=%d\r\n", cfg->node_id, initial_mode);
}

void motor_ctrl_process(void)
{
    /* 阶段1：推进 CANopen 通用初始化 */
    co_init_process();

    /* 阶段2：CANopen 初始化完成后启动 402 状态机
     *        此时心跳消费者已注册完毕，cia402_init 可正常获取 hb_idx */
    if (co_init_done() && !s_cia402_started) {
        cia402_init(s_co, s_cfg, s_initial_mode);
        s_cia402_started = true;
        printf("motor_ctrl: CiA402 started\r\n");
    }

    /* 阶段3：推进 402 状态机 */
    if (s_cia402_started) {
        cia402_process();
    }
}
