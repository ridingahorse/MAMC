/**
  * @file    pdo_cfg_slave1.h
  * @brief   从站 node_id=1 的 PDO 配置（速度模式）
  *
  * 数据流：
  *   主站 TPDO1(0x201) → 从站 RPDO1 : 控制字(0x6040) + 目标速度(0x60FF)
  *   主站 RPDO1(0x181) ← 从站 TPDO1 : 状态字(0x6041) + 实际速度(0x606C)
  *
  * 主站侧：transmissionType=0xFF，手动调 CO_TPDOsendRequest() 触发
  * 从站侧：transmissionType=0xFE，event_timer=10ms 定时上报
  *
  * 要求主站 OD 映射与此文件一致，否则 PDO 长度/内容对不上。
  */
 #ifndef PDO_CFG_SLAVE1_H
 #define PDO_CFG_SLAVE1_H
 
 #include "canopen_init.h"
 
 #define SLAVE1_NODE_ID      2U
 #define SLAVE1_TPDO_PERIOD  10U     /**< 从站定时上报周期(ms) */
 
 /** 构造映射条目: (index << 16 | sub << 8 | bitLength) */
 #define PDO_MAP(index, sub, bits) \
     (((uint32_t)(index) << 16) | ((uint32_t)(sub) << 8) | (uint32_t)(bits))
 
 /* ═══════════════════════════════════════════════════════════════
  *  从站 RPDO1：接收控制字 + 目标速度
  *  从站 OD: comm=0x1400 / map=0x1600
  * ═══════════════════════════════════════════════════════════════ */
 static const co_pdo_channel_t slave1_rpdo1 = {
     .comm_idx    = 0x1400,
     .map_idx     = 0x1600,
     .cob_id      = 0x200 + SLAVE1_NODE_ID,   /* 0x202 */
     .tx_type     = 0xFE,
     .event_timer = 0,                         /* RPDO 不需要定时器 */
     .map_count   = 2,
     .map_entries = {
         PDO_MAP(0x6040, 0, 16),  /* 控制字   0x6040.00 */
         PDO_MAP(0x60FF, 0, 32),  /* 目标速度 0x60FF.00 */
     },
 };
 
 /* ═══════════════════════════════════════════════════════════════
  *  从站 TPDO1：定时上报状态字 + 实际速度
  *  从站 OD: comm=0x1800 / map=0x1A00
  * ═══════════════════════════════════════════════════════════════ */
 static const co_pdo_channel_t slave1_tpdo1 = {
     .comm_idx    = 0x1800,
     .map_idx     = 0x1A00,
     .cob_id      = 0x180 + SLAVE1_NODE_ID,   /* 0x182 */
     .tx_type     = 0xFE,                      /* 事件驱动 */
     .event_timer = SLAVE1_TPDO_PERIOD,        /* 定时 10ms 上报 */
     .map_count   = 2,
     .map_entries = {
         PDO_MAP(0x6041, 0, 16),  /* 状态字   0x6041.00 */
         PDO_MAP(0x606C, 0, 32),  /* 实际速度 0x606C.00 */
     },
 };
 
 /* ── 汇总 ── */
 static const co_pdo_channel_t slave1_pdo_channels[] = {
     slave1_rpdo1,
     slave1_tpdo1,
 };
 
 static const co_pdo_config_t slave1_pdo_config = {
     .channels = slave1_pdo_channels,
     .count    = sizeof(slave1_pdo_channels) / sizeof(slave1_pdo_channels[0]),
 };
 
 #endif /* PDO_CFG_SLAVE1_H */