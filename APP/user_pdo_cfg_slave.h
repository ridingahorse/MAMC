/**
  * @file    user_pdo_cfg_slave.h
  * @brief   从站 node_id=2 的 PDO 配置（速度模式 / 位置模式）
  *
  * 速度模式数据流：
  *   主站 TPDO1(0x202) → 从站 RPDO1 : 控制字(0x6040) + 目标速度(0x60FF)
  *   主站 RPDO1(0x182) ← 从站 TPDO1 : 状态字(0x6041) + 实际速度(0x606C)
  *
  * 位置模式数据流：
  *   主站 TPDO1(0x202) → 从站 RPDO1 : 控制字(0x6040) + 目标位置(0x607A)
  *   主站 RPDO1(0x182) ← 从站 TPDO1 : 状态字(0x6041) + 实际位置(0x6064)
  *
  * 主站侧：transmissionType=0xFF，手动调 CO_TPDOsendRequest() 触发
  * 从站侧：transmissionType=0xFF，event_timer=10ms 定时上报（雷赛DM556需0xFF才使能event_timer）
  *
  * 不用 static const，运行时切换模式需要修改映射条目。
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
  *  速度模式 PDO 配置
  * ═══════════════════════════════════════════════════════════════ */
 
 static const co_pdo_channel_t slave1_vel_rpdo1 = {
     .comm_idx    = 0x1400,
     .map_idx     = 0x1600,
     .cob_id      = 0x200 + SLAVE1_NODE_ID,   /* 0x202 */
     .tx_type     = 0xFE,
     .event_timer = 0,
     .inhibit_time = 0,
     .map_count   = 2,
     .map_entries = {
         PDO_MAP(0x6040, 0, 16),  /* 控制字   0x6040.00 */
         PDO_MAP(0x60FF, 0, 32),  /* 目标速度 0x60FF.00 */
     },
 };
 
 static const co_pdo_channel_t slave1_vel_tpdo1 = {
    .comm_idx    = 0x1800,
    .map_idx     = 0x1A00,
    .cob_id      = 0x180 + SLAVE1_NODE_ID,   /* 0x182 */
    .tx_type     = 0xFF,
    .event_timer = SLAVE1_TPDO_PERIOD,
    .inhibit_time = 100,
    .map_count   = 2,
    .map_entries = {
        PDO_MAP(0x6041, 0, 16),  /* 状态字   0x6041.00 */
        PDO_MAP(0x606C, 0, 32),  /* 实际速度 0x606C.00 */
    },
};

/* 速度模式：主站侧 PDO 映射 */
static const co_master_pdo_mapping_t slave1_vel_master_mappings[] = {
    { .index = 0x1A00, .count = 2, .entries = {
        PDO_MAP(0x2000, 0, 16),  /* 主站 TPDO1: 控制字   0x2000.00 */
        PDO_MAP(0x2002, 0, 32),  /*              目标速度 0x2002.00 */
    }},
    { .index = 0x1600, .count = 2, .entries = {
        PDO_MAP(0x2003, 0, 16),  /* 主站 RPDO1: 状态字   0x2003.00 */
        PDO_MAP(0x2005, 0, 32),  /*              实际速度 0x2005.00 */
    }},
};

static const co_pdo_channel_t slave1_vel_channels[] = {
    slave1_vel_rpdo1,
    slave1_vel_tpdo1,
};

static const co_pdo_config_t slave1_vel_config = {
    .channels             = slave1_vel_channels,
    .count                = sizeof(slave1_vel_channels) / sizeof(slave1_vel_channels[0]),
    .master_mappings      = slave1_vel_master_mappings,
    .master_mappings_count = sizeof(slave1_vel_master_mappings) / sizeof(slave1_vel_master_mappings[0]),
    /* 同步 PDO 开关：false=异步（现状），true=主站发 SYNC，从站 RPDO 同步接收 */
    .use_sync             = false,
    .sync_period_us       = 0,
};
 
 /* ═══════════════════════════════════════════════════════════════
  *  位置模式 PDO 配置
  *  主站 OD 已存在：0x2001 target_position(INT32)、0x2004 actual_position(INT32)
  * ═══════════════════════════════════════════════════════════════ */
 
 static const co_pdo_channel_t slave1_pos_rpdo1 = {
     .comm_idx    = 0x1400,
     .map_idx     = 0x1600,
     .cob_id      = 0x200 + SLAVE1_NODE_ID,
     .tx_type     = 0xFE,
     .event_timer = 0,
     .inhibit_time = 0,
     .map_count   = 2,
     .map_entries = {
         PDO_MAP(0x6040, 0, 16),  /* 控制字     0x6040.00 */
         PDO_MAP(0x607A, 0, 32),  /* 目标位置   0x607A.00 */
     },
 };
 
 static const co_pdo_channel_t slave1_pos_tpdo1 = {
    .comm_idx    = 0x1800,
    .map_idx     = 0x1A00,
    .cob_id      = 0x180 + SLAVE1_NODE_ID,
    .tx_type     = 0xFF,
    .event_timer = SLAVE1_TPDO_PERIOD,
    .inhibit_time = 100,
    .map_count   = 2,
    .map_entries = {
        PDO_MAP(0x6041, 0, 16),  /* 状态字     0x6041.00 */
        PDO_MAP(0x6064, 0, 32),  /* 实际位置   0x6064.00 */
    },
};

/* 位置模式：主站侧 PDO 映射 */
static const co_master_pdo_mapping_t slave1_pos_master_mappings[] = {
    { .index = 0x1A00, .count = 2, .entries = {
        PDO_MAP(0x2000, 0, 16),  /* 主站 TPDO1: 控制字     0x2000.00 */
        PDO_MAP(0x2001, 0, 32),  /*              目标位置   0x2001.00 */
    }},
    { .index = 0x1600, .count = 2, .entries = {
        PDO_MAP(0x2003, 0, 16),  /* 主站 RPDO1: 状态字     0x2003.00 */
        PDO_MAP(0x2004, 0, 32),  /*              实际位置   0x2004.00 */
    }},
};

static const co_pdo_channel_t slave1_pos_channels[] = {
    slave1_pos_rpdo1,
    slave1_pos_tpdo1,
};

static const co_pdo_config_t slave1_pos_config = {
    .channels             = slave1_pos_channels,
    .count                = sizeof(slave1_pos_channels) / sizeof(slave1_pos_channels[0]),
    .master_mappings      = slave1_pos_master_mappings,
    .master_mappings_count = sizeof(slave1_pos_master_mappings) / sizeof(slave1_pos_master_mappings[0]),
    /* 同步 PDO 开关：false=异步（现状），true=主站发 SYNC，从站 RPDO 同步接收 */
    .use_sync             = false,
    .sync_period_us       = 0,
};
 
 #endif /* PDO_CFG_SLAVE1_H */