/* APP/app_log.h — 串口日志开关
 *
 * 全工程的打印都从这里走。两件事：
 *
 *   1. 【正式运行】把 APP_LOG_COMPILE_LEVEL 改成 APP_LOG_NONE，低于该级的
 *      LOGx() 整条被编译掉 —— 不是运行时跳过，是连字符串常量都不进 flash。
 *      想保留故障提示就用 APP_LOG_ERR。
 *
 *   2. 【调试】运行期挡位 g_app_log_level（初值 = 编译期上界）可以在调试器里
 *      直接改写，或者调 app_log_set_level()，不用重新烧录就能把刷屏的那一段
 *      压下去。它只能往下压，不会超过编译期已经编进去的。
 *
 * 打印最终落到哪里由 framework/BSP/bsp_uart.c 决定（串口 / 同时镜像进 RAM
 * 环形缓冲）。本开关只管「这条打印要不要发出去」。
 *
 * 分级口径：
 *   LOGE  故障：SDO 失败、402 fault、从站掉线、配置发起失败
 *   LOGW  可疑但不致命：错误寄存器非零、模式切换被拒、超时后继续走
 *   LOGI  流程里程碑：初始化完成、模式切换、测试每拍的判据与结果
 *   LOGD  过程细节：逐步子步骤、周期轮询、探针逐条、帧级流水
 */
#ifndef _APP_APP_LOG_H_
#define _APP_APP_LOG_H_

#include <stdint.h>
#include <stdio.h>

#define APP_LOG_NONE 0   /**< 全关（正式运行） */
#define APP_LOG_ERR  1   /**< 只留故障 */
#define APP_LOG_INFO 2   /**< + 流程里程碑与测试判据 */
#define APP_LOG_DBG  3   /**< + 过程细节（开发默认） */

/* ── 编译期上界：决定哪些打印进固件 ──
 * 正式运行改这一行即可。低于此级的调用整条编译掉，零代码零耗时。 */
#define APP_LOG_COMPILE_LEVEL APP_LOG_DBG

/* ── 运行期挡位 ──
 * 初值 = APP_LOG_COMPILE_LEVEL。调试器里直接改写可让某段不刷屏，无需重烧。
 * 只降不升：编译期没编进去的，这里调高也出不来。 */
extern volatile uint8_t g_app_log_level;

/** @brief 设置运行期挡位（APP_LOG_NONE..APP_LOG_DBG），只降不升 */
void app_log_set_level(uint8_t level);

#if APP_LOG_COMPILE_LEVEL >= APP_LOG_ERR
#define LOGE(...) do { if (g_app_log_level >= APP_LOG_ERR)  { printf(__VA_ARGS__); } } while (0)
#else
#define LOGE(...) ((void)0)
#endif

#if APP_LOG_COMPILE_LEVEL >= APP_LOG_INFO
#define LOGW(...) do { if (g_app_log_level >= APP_LOG_INFO) { printf(__VA_ARGS__); } } while (0)
#define LOGI(...) do { if (g_app_log_level >= APP_LOG_INFO) { printf(__VA_ARGS__); } } while (0)
#else
#define LOGW(...) ((void)0)
#define LOGI(...) ((void)0)
#endif

#if APP_LOG_COMPILE_LEVEL >= APP_LOG_DBG
#define LOGD(...) do { if (g_app_log_level >= APP_LOG_DBG)  { printf(__VA_ARGS__); } } while (0)
#else
#define LOGD(...) ((void)0)
#endif

#endif /* _APP_APP_LOG_H_ */
