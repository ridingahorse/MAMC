#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "CANopen.h"
#include "cia402.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  一站式配置入口：保存配置并启动 CANopen 初始化。
 *         402 状态机会在 CANopen 初始化完成后由 motor_ctrl_process 自动启动。
 * @param  idx           从站索引（0 ~ CIA402_MAX_SLAVES-1）
 * @param  co            协议栈实例
 * @param  cfg           从站完整配置（硬件+模式+轨迹）
 * @param  initial_mode  初始运行模式
 */
void motor_ctrl_set_config(uint8_t idx, CO_t* co, const cia402_slave_config_t* cfg, cia402_mode_t initial_mode);

/**
 * @brief  周期推进：协调 canopen_init 与 cia402 的状态机。
 *         主循环每周期调用一次。
 */
void motor_ctrl_process(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CTRL_H */