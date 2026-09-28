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
 
 typedef enum {
     PDO_STEP_DISABLE     = 0,  /**< 写 map_idx  sub0 = 0（清映射） */
     PDO_STEP_CLEAR_MAP,         /**< 写 map_idx  sub1（第一条映射条目） */
     PDO_STEP_WRITE_ENTRY,       /**< 写 map_idx  sub2..subN（继续写条目） */
     PDO_STEP_SET_COUNT,         /**< 写 map_idx  sub0 = map_count */
     PDO_STEP_SET_TX_TYPE,       /**< 写 comm_idx sub2 = tx_type */
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
         /* 写传输类型 */
         val32 = ch->tx_type;
         co_master_start_write(s->node_id, ch->comm_idx, 2, &val32, 1, CO_INIT_PDO_TIMEOUT_MS);
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
 
     case CO_INIT_WAIT_PREOP:
         node = &s_co->HBcons->monitoredNodes[s->hb_idx];
         if (node->HBstate == CO_HBconsumer_ACTIVE) {
             s->online = true;   /* 心跳到了也算上线 */
         }
         if (s->online && node->NMTstate == CO_NMT_PRE_OPERATIONAL) {
             s->state = CO_INIT_READ_IDENTITY;
         } else if (now_ms() >= s->deadline) {
             s->fault = true;
             s->fault_code = CO_INIT_ERR_PREOP_TIMEOUT;
             s->state = CO_INIT_FAULT;
         }
         break;
 
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
             printf("CO_INIT: vendor ID = 0x%08X\r\n", vendor_id);
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
             if (co_master_get_u8(&u8val) && u8val == 0) {
                 s->state = CO_INIT_CFG_HEARTBEAT;
             } else {
                 s->fault = true; s->fault_code = CO_INIT_ERR_ERRREG_NONZERO;
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
 
         s_slave.hb_idx = (uint8_t)CO_HBconsumer_getIdxByNodeId(s_co->HBcons, nodeId);
         CO_HBconsumer_initCallbackRemoteReset(s_co->HBcons, s_slave.hb_idx, NULL, on_bootup);
     }
 
     printf("CO_INIT: start node=%u\r\n", nodeId);
 }
 
 void co_init_process(void) {
     fsm_dispatch(&s_slave);
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