/**
  * @file    canopen_init.h
  * @brief   CANopen 通用初始化状态机（协议无关）
  *
  * 负责把从站从「上电」带到「OP 状态」。完成后协议专属状态机
  * （cia402 / 未来的 cia502、cia602 等）接管，互不影响。
  *
  * 流程：等PRE-OP → 读设备信息 → 查错误寄存器 → 配心跳 → 配PDO → 发NMT进OP → DONE
  */
 #ifndef CANOPEN_INIT_H
 #define CANOPEN_INIT_H
 
 #include "CANopen.h"
 #include <stdbool.h>
 #include <stdint.h>
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /* ═══════════════════════════════════════════════════════════════
  *  主状态
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef enum {
     CO_INIT_WAIT_PREOP    = 0,  /**< 等从站心跳变成 PRE-OP */
     CO_INIT_READ_IDENTITY,      /**< 读 0x1018 vendor ID */
     CO_INIT_READ_ERROR_REG,     /**< 读 0x1001 错误寄存器 */
     CO_INIT_CFG_HEARTBEAT,      /**< 写 0x1017 心跳生产者周期 */
     CO_INIT_CFG_PDO,            /**< 配置 PDO 通讯+映射 */
     CO_INIT_ENTER_OP,           /**< 发 NMT 进 OP，等心跳确认 */
    CO_INIT_DONE,               /**< 初始化完成（从站已在 OP） */
    CO_INIT_FAULT,              /**< 故障 */
    CO_INIT_PDO_RECONFIG,       /**< 运行时重配PDO：发NMT PRE-OP → 重配PDO → 进OP */
} co_init_state_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  超时
  * ═══════════════════════════════════════════════════════════════ */
 
 #define CO_INIT_PREOP_TIMEOUT_MS   5000U
 #define CO_INIT_SDO_TIMEOUT_MS      500U
 #define CO_INIT_PDO_TIMEOUT_MS     1000U
 #define CO_INIT_OP_TIMEOUT_MS      3000U
 
 /* ═══════════════════════════════════════════════════════════════
  *  PDO 配置描述（静态，多个从站可共享）
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef struct {
     uint16_t comm_idx;          /**< 通讯参数索引：0x1400+n / 0x1800+n */
     uint16_t map_idx;           /**< 映射参数索引：0x1600+n / 0x1A00+n */
     uint32_t cob_id;            /**< 实际 COB-ID（不含启用位 bit31） */
     uint8_t  tx_type;           /**< 传输类型 */
    uint16_t event_timer;       /**< TPDO事件定时器(ms)，从站定时上报，RPDO设0 */
    uint16_t inhibit_time;      /**< TPDO禁止时间(100μs单位)，0=不限制；防止事件风暴 */
    uint8_t  map_count;         /**< 映射条目数 */
     uint32_t map_entries[8];    /**< (index<<16 | subIndex<<8 | bitLength) 从站侧映射 */
 } co_pdo_channel_t;

/** 主站侧 PDO 映射（与从站配置分开，独立描述） */
typedef struct {
    uint16_t index;             /**< 主站映射参数索引：0x1600 / 0x1A00 */
    uint8_t  count;             /**< 映射条目数 */
    uint32_t entries[8];        /**< (index<<16 | subIndex<<8 | bitLength) 主站侧映射 */
} co_master_pdo_mapping_t;
 
 typedef struct {
     const co_pdo_channel_t* channels;        /**< 从站 PDO 通道配置 */
     uint8_t                  count;
     const co_master_pdo_mapping_t* master_mappings; /**< 主站 PDO 映射（NULL=不重配主站） */
     uint8_t                  master_mappings_count;
     /* ── 同步 PDO 配置 ──
      * use_sync=true 时：
      *   1. 主站启用 SYNC 生产者（0x1005 + 0x1006），周期性发 SYNC 帧
      *   2. 从站所有 RPDO 的传输类型被覆盖为 0x01（同步接收）
      *      —— 主站仍用 CO_TPDOsendRequest 发数据，从站收到后缓存，
      *         SYNC 到来时统一应用到对象字典，实现多轴同步。
      *   3. TPDO（从站→主站）传输类型不变，保持异步定时上报。
      * use_sync=false 时：全部沿用 channels 里的原始配置（异步）。 */
     bool_t   use_sync;          /**< true=从站同步接收RPDO, false=异步接收 */
     uint32_t sync_period_us;    /**< SYNC 周期（微秒），仅 use_sync=true 时有效，如 1000=1ms */
 } co_pdo_config_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  故障码
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef enum {
     CO_INIT_ERR_NONE            = 0x0000,
     CO_INIT_ERR_PREOP_TIMEOUT   = 0x0100,  /**< 等 PRE-OP 超时 */
     CO_INIT_ERR_IDENTITY_SDO    = 0x0200,  /**< 读设备信息 SDO 失败 */
     CO_INIT_ERR_ERRREG_SDO      = 0x0300,  /**< 读错误寄存器 SDO 失败 */
    CO_INIT_ERR_HB_SDO          = 0x0400,  /**< 配心跳 SDO 失败 */
     CO_INIT_ERR_PDO             = 0x0500,  /**< 配 PDO 失败 */
     CO_INIT_ERR_OP_TIMEOUT      = 0x0600,  /**< 等进 OP 超时 */
 } co_init_err_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  从站实例
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef struct {
     uint8_t  node_id;           /**< 从站节点号 */
     uint8_t  hb_idx;            /**< 心跳消费者 0x1016 表中的索引 */
 
     co_init_state_t state;      /**< 当前主状态 */
     co_init_state_t prev_state; /**< 上一次状态，用于触发 enter */
     uint8_t  sub_step;          /**< 状态内子步骤（如 SDO 写/读） */
 
     /* PDO 子状态机专用字段（独立于 sub_step，避免冲突） */
     uint8_t  pdo_step;          /**< PDO 配置步骤 */
     uint8_t  pdo_channel;       /**< 当前配置的 PDO 通道索引 */
     uint8_t  pdo_entry_idx;     /**< 当前通道内的映射条目索引 */
 
     uint32_t deadline;          /**< 当前阶段超时截止时刻 */
     uint8_t  retry;             /**< 重试次数 */
 
     bool_t   online;            /**< 是否上线（bootup/心跳置位） */
 
     bool_t   fault;             /**< 是否故障 */
     uint16_t fault_code;        /**< 故障码 */
 
     const co_pdo_config_t* pdo_cfg;  /**< PDO 配置表（NULL=用默认映射） */
 } co_init_slave_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  对外接口
  * ═══════════════════════════════════════════════════════════════ */
 
 /**
   * @brief  启动通用初始化，绑定协议栈与从站
   * @param  co       协议栈实例
   * @param  nodeId   从站节点号
   * @param  pdo_cfg  PDO 配置表（NULL=不配置，用从站默认映射）
   */
 void co_init_start(CO_t* co, uint8_t nodeId, const co_pdo_config_t* pdo_cfg);
 
 /**
   * @brief  状态机推进，主循环每周期调用
   */
 void co_init_process(void);
 
 /**
  * @brief  是否初始化完成（从站已进 OP）
  */
bool_t co_init_done(void);

/**
  * @brief  运行时重新配置 PDO 并重启从站。
  *         先发 NMT 把从站拉回 PRE-OP，再用新 PDO 配置重新走
  *         CFG_PDO → ENTER_OP 流程。用于切换运行模式（速度/位置）。
  * @pre    调用前 cia402 应已停机（disable operation），否则从站
  *         在 OP 下收到 PRE-OP 命令时 PDO 会被禁用，可能丢帧。
  * @param  new_cfg  新的 PDO 配置表
  */
void co_init_restart(const co_pdo_config_t* new_cfg);
 
 /**
   * @brief  查询当前状态
   */
 co_init_state_t co_init_get_state(void);
 
 /**
   * @brief  获取故障码
   */
 uint16_t co_init_get_fault_code(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* CANOPEN_INIT_H */