/**
  * @file    cia402.h
  * @brief   CiA 402 专属状态机 —— 从 OP 到 Operation enabled
  *
  * 前置条件：通用 CANopen 初始化（等PRE-OP、配心跳、配PDO、发NMT进OP）
  *           已由 canopen_init 模块完成，从站应进入或即将进入 OP。
  *
  * 流程：等OP → 设模式 → shutdown → switch on → enable op → RUNNING
  *
  * 未来扩展 cia502 / cia602 时，照此模板新建文件即可，不影响本文件。
  */
#ifndef CIA402_H
#define CIA402_H

#include "CANopen.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════
 *  主状态枚举
 * ═══════════════════════════════════════════════════════════════ */

typedef enum {
    CIA402_WAIT_OP     = 0,  /**< 等从站心跳确认进入 OP */
    CIA402_SET_MODE,         /**< 写 0x6060 设模式，读 0x6061 确认 */
    CIA402_SHUTDOWN,         /**< 写控制字 0x0006，等 Ready to switch on */
    CIA402_SWITCH_ON,        /**< 写控制字 0x0007，等 Switched on */
    CIA402_ENABLE_OP,        /**< 写控制字 0x000F，等 Operation enabled */
    CIA402_RUNNING,          /**< 初始化完成 */
    CIA402_FAULT,            /**< 故障 */
} cia402_state_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  超时与重试
  * ═══════════════════════════════════════════════════════════════ */
 
#define CIA402_SDO_TIMEOUT_MS      500U
#define CIA402_OP_TIMEOUT_MS      3000U
#define CIA402_STEP_TIMEOUT_MS    1000U
 
 /* ═══════════════════════════════════════════════════════════════
  *  CiA 402 控制字 / 状态字
  * ═══════════════════════════════════════════════════════════════ */
 
 #define CIA402_CMD_SHUTDOWN      0x0006U  /**< Switch on disabled → Ready to switch on */
 #define CIA402_CMD_SWITCH_ON     0x0007U  /**< Ready to switch on → Switched on */
 #define CIA402_CMD_ENABLE_OP     0x000FU  /**< Switched on → Operation enabled */
 
 #define CIA402_STS_MASK          0x006FU  /**< 屏蔽厂家位与警告位 */
 #define CIA402_STS_READY_SWITCH  0x0021U  /**< Ready to switch on */
 #define CIA402_STS_SWITCHED_ON   0x0023U  /**< Switched on */
 #define CIA402_STS_OP_ENABLED    0x0027U  /**< Operation enabled */
 
 /* ═══════════════════════════════════════════════════════════════
  *  运行模式
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef enum {
     CIA402_MODE_PROFILE_POS = 1,   /**< 轮廓位置 */
     CIA402_MODE_PROFILE_VEL = 3,   /**< 轮廓速度 */
     CIA402_MODE_TORQUE      = 4,   /**< 转矩 */
     CIA402_MODE_HOMING      = 6,   /**< 回零 */
     CIA402_MODE_CSP         = 8,   /**< 循环同步位置 */
 } cia402_mode_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  故障码
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef enum {
     CIA402_ERR_NONE          = 0x0000,
     CIA402_ERR_OP_TIMEOUT    = 0x0100,  /**< 等从站进 OP 超时 */
     CIA402_ERR_MODE_TIMEOUT  = 0x0200,  /**< 设模式 SDO 超时 */
     CIA402_ERR_MODE_SDO      = 0x0201,  /**< 设模式 SDO 失败 */
     CIA402_ERR_MODE_MISMATCH = 0x0202,  /**< 读回模式与目标不一致 */
     CIA402_ERR_TRANS_TIMEOUT = 0x0300,  /**< 402 状态转换超时 */
     CIA402_ERR_TRANS_SDO     = 0x0301,  /**< 402 转换写控制字/读状态字 SDO 失败 */
 } cia402_err_t;
 
 /* PDO 配置描述已移至 canopen_init.h，402 模块不再关心 PDO 配置 */
 
 /* ═══════════════════════════════════════════════════════════════
  *  从站实例（运行时全部状态）
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef struct {
    uint8_t  node_id;           /**< 从站节点号 */
    uint8_t  hb_idx;            /**< 心跳消费者 0x1016 表中的索引 */

    cia402_state_t state;       /**< 当前主状态 */
    cia402_state_t prev_state;  /**< 上一次状态，用于触发 enter */
    uint8_t  trans_step;        /**< 402 状态转换子步骤（0=写控制字, 1=读状态字） */

    uint32_t deadline;          /**< 当前阶段超时截止时刻（ms） */
    uint8_t  retry;             /**< 重试次数 */

    bool_t   fault;             /**< 是否故障 */
    uint16_t fault_code;        /**< 故障码 */

    cia402_mode_t target_mode;  /**< 期望运行模式 */
} cia402_slave_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  对外接口
  * ═══════════════════════════════════════════════════════════════ */
 
 /**
   * @brief  初始化，绑定协议栈与从站。只做数据结构初始化，不发起通信。
   *         前置条件：canopen_init 已完成（从站已进 OP 或即将进 OP）。
   * @param  co       协议栈实例
   * @param  nodeId   从站节点号
   * @param  mode     期望运行模式
   */
 void cia402_init(CO_t* co, uint8_t nodeId, cia402_mode_t mode);
 
 /**
   * @brief  状态机推进，主循环每周期调用，全程非阻塞
   */
 void cia402_process(void);
 
 /**
   * @brief  查询当前主状态
   */
 cia402_state_t cia402_get_state(void);
 
 /**
   * @brief  获取故障码
   */
 uint16_t cia402_get_fault_code(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* CIA402_H */