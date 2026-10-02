/**
  * @file    canopen_init.c
  * @brief   CANopen 通用初始化状态机实现
  */
 #include "canopen_init.h"

 #include "canopen_master.h"
 #include "OD.h"
 #include "301/CO_HBconsumer.h"
 #include "main.h"
 
 #include <stdio.h>
 #include <string.h>
 
 /* ═══════════════════════════════════════════════════════════════
  *  私有全局
  * ═══════════════════════════════════════════════════════════════ */
 
 static CO_t* s_co = NULL;
 static co_init_slave_t s_slave;
 
 /* ═══════════════════════════════════════════════════════════════
  *  工具
  * ═══════════════════════════════════════════════════════════════ */
 
 static uint32_t now_ms(void) {
     return HAL_GetTick();
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  bootup 回调（协议栈收到从站 bootup 帧时由 CO_process 调用）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void on_bootup(uint8_t nodeId, uint8_t idx, void* object) {
     (void)idx;
     (void)object;
     if (nodeId == s_slave.node_id) {
         s_slave.online = true;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  PDO 配置子状态机
  *
  *  配置顺序：禁用 → 清映射 → 写条目 → 设数量 → 传输类型 → 定时器 → 启用
  *  注意：每个步骤名的含义是「本步要发起的 SDO」，不是「上一步的 SDO」。
  *  返回：BUSY=进行中, DONE=全部完成, FAULT=失败
 * ═══════════════════════════════════════════════════════════════ */
 
/* ═══════════════════════════════════════════════════════════════
 *  主站 PDO 映射重配：把 cfg->master_mappings 写入主站 OD
 *  从站配置（channels）和主站映射（master_mappings）完全分开，
 *  仅由 co_pdo_config_t 关联。
 * ═══════════════════════════════════════════════════════════════ */
static void apply_master_pdo_mapping(const co_pdo_config_t* cfg)
{
    if (s_co == NULL || cfg == NULL || cfg->master_mappings == NULL) return;

    for (uint8_t i = 0; i < cfg->master_mappings_count; i++) {
        const co_master_pdo_mapping_t* m = &cfg->master_mappings[i];

        /* 通过 OD_find 找主站映射条目（0x1600 / 0x1A00） */
        OD_entry_t* master_map = OD_find(OD, m->index);
        if (master_map == NULL) {
            printf("CO_INIT: master map 0x%04X NOT FOUND\r\n", m->index);
            continue;
        }

        /* 1. 清映射：sub0 = 0（uint8_t） */
        OD_set_u8(master_map, 0, 0, true);
        /* 2. 写映射条目 sub1..subN（uint32_t） */
        for (uint8_t e = 0; e < m->count; e++) {
            OD_set_u32(master_map, (uint8_t)(e + 1), m->entries[e], true);
        }
        /* 3. 设映射数：sub0 = count（uint8_t） */
        OD_set_u8(master_map, 0, m->count, true);
    }

    /* 重新初始化 PDO，使新映射生效 */
    uint32_t errInfo = 0;
    CO_CANopenInitPDO(s_co, s_co->em, OD, s_co->NMT->nodeId, &errInfo);
}

 typedef enum {
     PDO_STEP_DISABLE     = 0,  /**< 写 map_idx  sub0 = 0（清映射） */
     PDO_STEP_CLEAR_MAP,         /**< 写 map_idx  sub1（第一条映射条目） */
     PDO_STEP_WRITE_ENTRY,       /**< 写 map_idx  sub2..subN（继续写条目） */
     PDO_STEP_SET_COUNT,         /**< 写 map_idx  sub0 = map_count */
     PDO_STEP_SET_TX_TYPE,       /**< 写 comm_idx sub2 = tx_type */
    PDO_STEP_SET_INHIBIT,       /**< 写 comm_idx sub3 = inhibit_time */
    PDO_STEP_SET_EVENT_TIMER,   /**< 写 comm_idx sub5 = event_timer */
    PDO_STEP_ENABLE,            /**< 写 comm_idx sub1 = cob_id（bit31=0，启用） */
     PDO_STEP_NEXT_CH            /**< 处理下一通道（禁用）或返回 DONE */
 } pdo_step_t;
 
 typedef enum {
     PDO_BUSY = 0,
     PDO_DONE,
     PDO_FAULT,
 } pdo_result_t;
 
 static pdo_result_t pdo_cfg_tick(co_init_slave_t* s) {
     const co_pdo_config_t* cfg = s->pdo_cfg;
 
     /* 没配 PDO 表，直接完成（用从站默认映射） */
     if (cfg == NULL || cfg->channels == NULL || cfg->count == 0) {
         return PDO_DONE;
     }
 
     const co_pdo_channel_t* ch = &cfg->channels[s->pdo_channel];
     uint32_t val32;
 
     co_master_poll();
 
     if (co_master_busy()) {
         if (now_ms() >= s->deadline) return PDO_FAULT;
         return PDO_BUSY;
     }
 
     if (co_master_get_result() != CO_MASTER_OK) {
         return PDO_FAULT;
     }
 
     /* 上一步 SDO 成功，执行下一步 */
     switch (s->pdo_step) {
 
     case PDO_STEP_DISABLE:
         /* 清映射 */
         val32 = 0;
         co_master_start_write(s->node_id, ch->map_idx, 0, &val32, 1, CO_INIT_PDO_TIMEOUT_MS);
         s->pdo_step = PDO_STEP_CLEAR_MAP;
         s->pdo_entry_idx = 0;
         break;
 
     case PDO_STEP_CLEAR_MAP:
         /* 清完，写第一条映射条目 */
         val32 = ch->map_entries[0];
         co_master_start_write(s->node_id, ch->map_idx, 1, &val32, 4, CO_INIT_PDO_TIMEOUT_MS);
         s->pdo_entry_idx = 1;
         s->pdo_step = PDO_STEP_WRITE_ENTRY;
         break;
 
     case PDO_STEP_WRITE_ENTRY:
         if (s->pdo_entry_idx < ch->map_count) {
             val32 = ch->map_entries[s->pdo_entry_idx];
             co_master_start_write(s->node_id, ch->map_idx,
                                   (uint8_t)(s->pdo_entry_idx + 1),
                                   &val32, 4, CO_INIT_PDO_TIMEOUT_MS);
             s->pdo_entry_idx++;
             /* 留在 WRITE_ENTRY 步骤 */
         } else {
             /* 所有条目写完，下步设映射条目数 */
             s->pdo_step = PDO_STEP_SET_COUNT;
         }
         break;
 
     case PDO_STEP_SET_COUNT:
        /* 写映射条目数 */
        val32 = ch->map_count;
        co_master_start_write(s->node_id, ch->map_idx, 0, &val32, 1, CO_INIT_PDO_TIMEOUT_MS);
        s->pdo_step = PDO_STEP_SET_TX_TYPE;
        break;

    case PDO_STEP_SET_TX_TYPE:
        /* 写传输类型。
         * 同步模式下，RPDO（主站→从站，comm_idx 0x1400~0x15FF）覆盖为 0x01：
         *   从站收到 RPDO 后先缓存，等 SYNC 到来时才写入对象字典，实现多轴同步。
         * TPDO（从站→主站，comm_idx 0x1800~0x19FF）保持原配置（异步定时上报）。 */
        val32 = ch->tx_type;
        if (cfg->use_sync && ch->comm_idx >= 0x1400u && ch->comm_idx < 0x1600u) {
            val32 = 0x01u;
        }
        co_master_start_write(s->node_id, ch->comm_idx, 2, &val32, 1, CO_INIT_PDO_TIMEOUT_MS);
        s->pdo_step = PDO_STEP_SET_INHIBIT;
        break;

    case PDO_STEP_SET_INHIBIT:
        /* 写禁止时间（sub3），防止事件驱动风暴 */
        val32 = ch->inhibit_time;
        co_master_start_write(s->node_id, ch->comm_idx, 3, &val32, 2, CO_INIT_PDO_TIMEOUT_MS);
        s->pdo_step = PDO_STEP_SET_EVENT_TIMER;
        break;

    case PDO_STEP_SET_EVENT_TIMER:
        /* 写事件定时器 */
        val32 = ch->event_timer;
        co_master_start_write(s->node_id, ch->comm_idx, 5, &val32, 2, CO_INIT_PDO_TIMEOUT_MS);
        s->pdo_step = PDO_STEP_ENABLE;
        break;

    case PDO_STEP_ENABLE:
        /* 启用 PDO（bit31=0） */
        val32 = ch->cob_id;
        co_master_start_write(s->node_id, ch->comm_idx, 1, &val32, 4, CO_INIT_PDO_TIMEOUT_MS);
        s->pdo_step = PDO_STEP_NEXT_CH;
        break;
 
     case PDO_STEP_NEXT_CH:
         /* 当前通道完成，下个通道（或全部完成） */
         s->pdo_channel++;
         if (s->pdo_channel >= cfg->count) {
             return PDO_DONE;
         }
         ch = &cfg->channels[s->pdo_channel];
         val32 = ch->cob_id | 0x80000000u;
         co_master_start_write(s->node_id, ch->comm_idx, 1, &val32, 4, CO_INIT_PDO_TIMEOUT_MS);
         s->pdo_step = PDO_STEP_DISABLE;
         s->pdo_entry_idx = 0;
         break;
     }
 
     s->deadline = now_ms() + CO_INIT_PDO_TIMEOUT_MS + 100;
     return PDO_BUSY;
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：enter（状态切换时执行一次）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_enter(co_init_slave_t* s) {
     uint16_t u16val;
 
     s->sub_step = 0;
     s->retry = 0;
 
     switch (s->state) {
     case CO_INIT_WAIT_PREOP:
         s->deadline = now_ms() + CO_INIT_PREOP_TIMEOUT_MS;
         break;
 
     case CO_INIT_READ_IDENTITY:
         co_master_start_read(s->node_id, 0x1018, 1, CO_INIT_SDO_TIMEOUT_MS);
         s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
         break;
 
     case CO_INIT_READ_ERROR_REG:
         co_master_start_read(s->node_id, 0x1001, 0, CO_INIT_SDO_TIMEOUT_MS);
         s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
         break;
 
     case CO_INIT_CFG_HEARTBEAT:
         u16val = 1000;   /* 心跳生产者周期 1000ms */
         co_master_start_write(s->node_id, 0x1017, 0, &u16val, 2, CO_INIT_SDO_TIMEOUT_MS);
         s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
         break;
 
     case CO_INIT_CFG_PDO: {
         s->pdo_channel = 0;
         s->pdo_entry_idx = 0;
         const co_pdo_config_t* cfg = s->pdo_cfg;
         if (cfg != NULL && cfg->use_sync) {
             /* 配置主站为 SYNC 生产者：
              *   0x1005 = 0x40000080  → COB-ID 0x80 + bit30(生产者使能)
              *   0x1019 = 0           → 不用同步计数器（SYNC 帧无数据）
              *   0x1006 = period_us   → SYNC 周期（微秒）
              * 顺序：1005 → 1019 → 1006。0x1019 的 OD 扩展要求 0x1006=0 时才能写。
              * 协议栈已开启 CO_CONFIG_FLAG_OD_DYNAMIC，写 OD 会触发 SYNC 重配置，
              * 之后 CO_process_SYNC() 按 0x1006 周期自动发送 SYNC 帧。 */
             OD_set_u32(OD_ENTRY_H1005, 0, 0x40000080u, false);
             OD_set_u8(OD_ENTRY_H1019, 0, 0u, false);
             OD_set_u32(OD_ENTRY_H1006, 0, cfg->sync_period_us, false);
             printf("CO_INIT: SYNC producer enabled, period=%lu us\r\n",
                    (unsigned long)cfg->sync_period_us);
         }
         if (cfg != NULL && cfg->channels != NULL && cfg->count > 0) {
             /* 发起第一步：禁用第一个通道 */
             const co_pdo_channel_t* ch = &cfg->channels[0];
             uint32_t v = ch->cob_id | 0x80000000u;
             co_master_start_write(s->node_id, ch->comm_idx, 1, &v, 4, CO_INIT_PDO_TIMEOUT_MS);
             s->pdo_step = PDO_STEP_DISABLE;
             s->deadline = now_ms() + CO_INIT_PDO_TIMEOUT_MS + 100;
         }
         break;
     }
 
     case CO_INIT_ENTER_OP:
        co_master_nmt_send(CO_NMT_ENTER_OPERATIONAL, s->node_id);
        s->deadline = now_ms() + CO_INIT_OP_TIMEOUT_MS;
        break;

    case CO_INIT_PDO_RECONFIG:
        co_master_nmt_send(CO_NMT_ENTER_PRE_OPERATIONAL, s->node_id);
        s->deadline = now_ms() + CO_INIT_OP_TIMEOUT_MS;
        break;
 
     case CO_INIT_DONE:
         printf("CO_INIT: node=%u 进入 OP，通用初始化完成\r\n", s->node_id);
         break;
 
     case CO_INIT_FAULT:
         printf("CO_INIT: node=%u 故障 code=0x%04X\r\n", s->node_id, s->fault_code);
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：tick（每周期执行）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_tick(co_init_slave_t* s) {
     CO_HBconsNode_t* node;
     uint8_t u8val;
 
     switch (s->state) {
 
     case CO_INIT_WAIT_PREOP: {
        if (s->hb_idx >= s_co->HBcons->numberOfMonitoredNodes) {
            /* 心跳消费者未配置，直接超时进故障 */
            s->fault = true;
            s->fault_code = CO_INIT_ERR_PREOP_TIMEOUT;
            s->state = CO_INIT_FAULT;
            break;
        }
        node = &s_co->HBcons->monitoredNodes[s->hb_idx];
        if (node->HBstate == CO_HBconsumer_ACTIVE) {
            s->online = true;   /* 心跳到了也算上线 */
        }
        if (s->online && node->NMTstate == CO_NMT_PRE_OPERATIONAL) {
            s->state = CO_INIT_READ_IDENTITY;
        } else if (s->online && node->NMTstate == CO_NMT_OPERATIONAL) {
            /* 从站已在 OP（上次配置过且掉电保存），跳过配置直接完成 */
            printf("CO_INIT: node=%u already in OP, skip config\r\n", s->node_id);
            s->state = CO_INIT_DONE;
        } else if (now_ms() >= s->deadline) {
            s->fault = true;
            s->fault_code = CO_INIT_ERR_PREOP_TIMEOUT;
            s->state = CO_INIT_FAULT;
        }
        break;
    }
 
     case CO_INIT_READ_IDENTITY:
         co_master_poll();
         if (co_master_busy()) {
             if (now_ms() >= s->deadline) {
                 s->fault = true; s->fault_code = CO_INIT_ERR_IDENTITY_SDO;
                 s->state = CO_INIT_FAULT;
             }
             break;
         }
         if (co_master_get_result() == CO_MASTER_OK) {
             uint32_t vendor_id;
             co_master_get_u32(&vendor_id);
             printf("CO_INIT: vendor ID = 0x%08lX\r\n", (unsigned long)vendor_id);
             s->state = CO_INIT_READ_ERROR_REG;
         } else {
             s->fault = true; s->fault_code = CO_INIT_ERR_IDENTITY_SDO;
             s->state = CO_INIT_FAULT;
         }
         break;
 
     case CO_INIT_READ_ERROR_REG:
         co_master_poll();
         if (co_master_busy()) {
             if (now_ms() >= s->deadline) {
                 s->fault = true; s->fault_code = CO_INIT_ERR_ERRREG_SDO;
                 s->state = CO_INIT_FAULT;
             }
             break;
         }
         if (co_master_get_result() == CO_MASTER_OK) {
            if (co_master_get_u8(&u8val)) {
                if (u8val != 0) {
                    /* 错误寄存器非零只警告，不终止。很多从站上电瞬间会置位，
                     * 实际不影响后续配置与运行。 */
                    printf("CO_INIT: node=%u 错误寄存器=0x%02X（警告，继续）\r\n", s->node_id, u8val);
                }
                s->state = CO_INIT_CFG_HEARTBEAT;
            } else {
                s->fault = true; s->fault_code = CO_INIT_ERR_ERRREG_SDO;
                s->state = CO_INIT_FAULT;
            }
        } else {
            s->fault = true; s->fault_code = CO_INIT_ERR_ERRREG_SDO;
            s->state = CO_INIT_FAULT;
        }
         break;
 
     case CO_INIT_CFG_HEARTBEAT:
         co_master_poll();
         if (co_master_busy()) {
             if (now_ms() >= s->deadline) {
                 s->fault = true; s->fault_code = CO_INIT_ERR_HB_SDO;
                 s->state = CO_INIT_FAULT;
             }
             break;
         }
         if (co_master_get_result() == CO_MASTER_OK) {
             s->state = CO_INIT_CFG_PDO;
         } else {
             s->fault = true; s->fault_code = CO_INIT_ERR_HB_SDO;
             s->state = CO_INIT_FAULT;
         }
         break;
 
     case CO_INIT_CFG_PDO: {
        pdo_result_t r = pdo_cfg_tick(s);
        if (r == PDO_DONE) {
            /* 从站 PDO 配置完成后，重配主站侧 PDO 映射 */
            apply_master_pdo_mapping(s->pdo_cfg);
            s->state = CO_INIT_ENTER_OP;
        } else if (r == PDO_FAULT) {
            s->fault = true; s->fault_code = CO_INIT_ERR_PDO;
            s->state = CO_INIT_FAULT;
        }
        break;
    }
 
     case CO_INIT_ENTER_OP:
        node = &s_co->HBcons->monitoredNodes[s->hb_idx];
        if (node->HBstate == CO_HBconsumer_ACTIVE
            && node->NMTstate == CO_NMT_OPERATIONAL) {
            s->state = CO_INIT_DONE;
        } else if (now_ms() >= s->deadline) {
            s->fault = true; s->fault_code = CO_INIT_ERR_OP_TIMEOUT;
            s->state = CO_INIT_FAULT;
        }
        break;

    case CO_INIT_PDO_RECONFIG:
        node = &s_co->HBcons->monitoredNodes[s->hb_idx];
        if (node->HBstate == CO_HBconsumer_ACTIVE
            && node->NMTstate == CO_NMT_PRE_OPERATIONAL) {
            /* 从站已回 PRE-OP，直接重新配 PDO（跳过读设备信息等） */
            s->state = CO_INIT_CFG_PDO;
        } else if (now_ms() >= s->deadline) {
            s->fault = true; s->fault_code = CO_INIT_ERR_OP_TIMEOUT;
            s->state = CO_INIT_FAULT;
        }
        break;
 
     case CO_INIT_DONE:
     case CO_INIT_FAULT:
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  状态机分发
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_dispatch(co_init_slave_t* s) {
     if (s->state != s->prev_state) {
         s->prev_state = s->state;
         fsm_enter(s);
     }
     fsm_tick(s);
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  对外接口
  * ═══════════════════════════════════════════════════════════════ */
 
 void co_init_start(CO_t* co, uint8_t nodeId, const co_pdo_config_t* pdo_cfg) {
     s_co = co;
     memset(&s_slave, 0, sizeof(s_slave));
     s_slave.node_id = nodeId;
     s_slave.pdo_cfg = pdo_cfg;
     s_slave.state = CO_INIT_WAIT_PREOP;
     s_slave.prev_state = (co_init_state_t)-1;
 
     /* 配置心跳消费者：OD 0x1016 写入 (nodeId<<16) | 1000ms */
    if (s_co != NULL && s_co->HBcons != NULL) {
        uint32_t entry = ((uint32_t)nodeId << 16) | 1000u;
        OD_set_u32(OD_ENTRY_H1016_consumerHeartbeatTime, 1, entry, false);

        int8_t idx = CO_HBconsumer_getIdxByNodeId(s_co->HBcons, nodeId);
        if (idx >= 0 && idx < s_co->HBcons->numberOfMonitoredNodes) {
            s_slave.hb_idx = (uint8_t)idx;
            CO_HBconsumer_initCallbackRemoteReset(s_co->HBcons, s_slave.hb_idx, NULL, on_bootup);
        } else {
            s_slave.hb_idx = 0xFF;  /* 标记无效 */
            printf("CO_INIT: ERROR heartbeat consumer not found for node=%u\r\n", nodeId);
        }
    } else {
        s_slave.hb_idx = 0xFF;
    }
 
     printf("CO_INIT: start node=%u\r\n", nodeId);
 }
 
 void co_init_process(void) {
    fsm_dispatch(&s_slave);
}

void co_init_restart(const co_pdo_config_t* new_cfg) {
    s_slave.pdo_cfg = new_cfg;
    s_slave.online = false;
    s_slave.fault = false;
    s_slave.fault_code = 0;
    s_slave.state = CO_INIT_PDO_RECONFIG;
    s_slave.prev_state = (co_init_state_t)-1;
    printf("CO_INIT: restart with new PDO config\r\n");
}
 
 bool_t co_init_done(void) {
     return s_slave.state == CO_INIT_DONE;
 }
 
 co_init_state_t co_init_get_state(void) {
     return s_slave.state;
 }
 
 uint16_t co_init_get_fault_code(void) {
     return s_slave.fault_code;
 }