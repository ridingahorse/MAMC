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
    /* 同步 PDO：从站 RPDO 同步接收。sync_period_us=0 表示不自动发 SYNC，
     * 由应用层按需手动调用 CO_SYNCsend() 触发，实现精确的多轴同步时序。 */
    .use_sync             = true,
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
        PDO_MAP(0x607A, 0, 32),  /* 目标位置   0x607A.00 */
        PDO_MAP(0x6040, 0, 16),  /* 控制字     0x6040.00 */
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
        PDO_MAP(0x2001, 0, 32),  /* 主站 TPDO1: 目标位置   0x2001.00 */
        PDO_MAP(0x2000, 0, 16),  /*              控制字     0x2000.00 */
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
    /* 同步 PDO：手动 SYNC 模式，应用层按需调用 co_sync_send() 触发。
     * 位置模式必须为 true：RPDO 传输类型会被覆盖成 0x01（同步接收），
     * 从站收到目标位置后先缓存，等 SYNC 到齐再一起生效，多轴才会同时起步。
     * false 时传输类型保持 0xFE，谁先收到谁先动 —— 两轴会差一整个发送间隔。 */
    .use_sync             = true,
    .sync_period_us       = 0,
};

/* ═══════════════════════════════════════════════════════════════
 *  slave2 (node_id=4) PDO 配置
 *  主站 TPDO2(0x204) → 从站 RPDO1 : 控制字 + 目标
 *  主站 RPDO2(0x184) ← 从站 TPDO1 : 状态字 + 实际
 *  主站侧映射：0x1A01(TPDO2) / 0x1601(RPDO2)，OD 变量 0x2010~0x2015
 * ═══════════════════════════════════════════════════════════════ */

#define SLAVE2_NODE_ID      4U
#define SLAVE2_TPDO_PERIOD  10U

/* ── slave2 速度模式 ── */
static const co_pdo_channel_t slave2_vel_rpdo1 = {
    .comm_idx    = 0x1400,
    .map_idx     = 0x1600,
    .cob_id      = 0x200 + SLAVE2_NODE_ID,
    .tx_type     = 0xFE,
    .event_timer = 0,
    .inhibit_time = 0,
    .map_count   = 2,
    .map_entries = {
        PDO_MAP(0x6040, 0, 16),
        PDO_MAP(0x60FF, 0, 32),
    },
};

static const co_pdo_channel_t slave2_vel_tpdo1 = {
    .comm_idx    = 0x1800,
    .map_idx     = 0x1A00,
    .cob_id      = 0x180 + SLAVE2_NODE_ID,
    .tx_type     = 0xFF,
    .event_timer = SLAVE2_TPDO_PERIOD,
    .inhibit_time = 100,
    .map_count   = 2,
    .map_entries = {
        PDO_MAP(0x6041, 0, 16),
        PDO_MAP(0x606C, 0, 32),
    },
};

static const co_master_pdo_mapping_t slave2_vel_master_mappings[] = {
    { .index = 0x1A01, .count = 2, .entries = {
        PDO_MAP(0x2010, 0, 16),
        PDO_MAP(0x2012, 0, 32),
    }},
    { .index = 0x1601, .count = 2, .entries = {
        PDO_MAP(0x2013, 0, 16),
        PDO_MAP(0x2015, 0, 32),
    }},
};

static const co_pdo_channel_t slave2_vel_channels[] = {
    slave2_vel_rpdo1,
    slave2_vel_tpdo1,
};

static const co_pdo_config_t slave2_vel_config = {
    .channels             = slave2_vel_channels,
    .count                = sizeof(slave2_vel_channels) / sizeof(slave2_vel_channels[0]),
    .master_mappings      = slave2_vel_master_mappings,
    .master_mappings_count = sizeof(slave2_vel_master_mappings) / sizeof(slave2_vel_master_mappings[0]),
    /* 同步接收（与 slave1、以及位置模式一致）：主站随时发的数据，从站先收进
     * 缓存，等 SYNC 到了才统一写进自己的对象字典 —— 多轴「同时起步」靠的就是
     * 这一条，也是两轴同步接收这个目标的最终形态。
     *
     * 代价，动这里之前要知道：
     *   1. 【只发数据、不补 SYNC，从站那边不会生效】—— 缓存在下一条 SYNC 才落地。
     *      PV 启动沿那三帧每帧后面都跟了一条 SYNC 就是为了它（见 pv_start_tick）；
     *      应用层改目标速度也一样，改完要自己补 SYNC，否则两台轴各按各的节拍生效。
     *   2. 从站的 PDO 参数（含 0x1400.02）存在【驱动器里】，但它只是运行内存，
     *      断电就丢。2026-10-09 起 canopen_init 每次开机都强制重写一遍，所以
     *      改完只需重烧主站、重启即可，不必再给驱动器断电重上。
     *
     * 曾有一条错误记录说「node4 旧固件不吃同步接收」—— 真因是主站在「从站已在 OP」
     * 时把【主站自己的映射】也跳过了，帧根本没带内容出去（详见 canopen_init.c 的
     * CO_INIT_WAIT_PREOP）。 */
    .use_sync             = true,
    .sync_period_us       = 0,
};

/* ── slave2 位置模式 ── */
static const co_pdo_channel_t slave2_pos_rpdo1 = {
    .comm_idx    = 0x1400,
    .map_idx     = 0x1600,
    .cob_id      = 0x200 + SLAVE2_NODE_ID,
    .tx_type     = 0xFE,
    .event_timer = 0,
    .inhibit_time = 0,
    .map_count   = 2,
    .map_entries = {
        PDO_MAP(0x607A, 0, 32),
        PDO_MAP(0x6040, 0, 16),
    },
};

static const co_pdo_channel_t slave2_pos_tpdo1 = {
    .comm_idx    = 0x1800,
    .map_idx     = 0x1A00,
    .cob_id      = 0x180 + SLAVE2_NODE_ID,
    .tx_type     = 0xFF,
    .event_timer = SLAVE2_TPDO_PERIOD,
    .inhibit_time = 100,
    .map_count   = 2,
    .map_entries = {
        PDO_MAP(0x6041, 0, 16),
        PDO_MAP(0x6064, 0, 32),
    },
};

static const co_master_pdo_mapping_t slave2_pos_master_mappings[] = {
    { .index = 0x1A01, .count = 2, .entries = {
        PDO_MAP(0x2011, 0, 32),
        PDO_MAP(0x2010, 0, 16),
    }},
    { .index = 0x1601, .count = 2, .entries = {
        PDO_MAP(0x2013, 0, 16),
        PDO_MAP(0x2014, 0, 32),
    }},
};

static const co_pdo_channel_t slave2_pos_channels[] = {
    slave2_pos_rpdo1,
    slave2_pos_tpdo1,
};

static const co_pdo_config_t slave2_pos_config = {
    .channels             = slave2_pos_channels,
    .count                = sizeof(slave2_pos_channels) / sizeof(slave2_pos_channels[0]),
    .master_mappings      = slave2_pos_master_mappings,
    .master_mappings_count = sizeof(slave2_pos_master_mappings) / sizeof(slave2_pos_master_mappings[0]),
    /* 同 slave1_pos_config：位置模式必须同步接收，否则两轴各自起步 */
    .use_sync             = true,
    .sync_period_us       = 0,
};

#endif /* PDO_CFG_SLAVE1_H */