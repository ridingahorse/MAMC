/**
  * @file    cia402.c
  * @brief   CiA 402 主站侧 —— 非阻塞初始化状态机实现
  *
  * 本模块只负责 CiA 402 专属部分：从站已进 OP 后，设模式 + 402 状态转换。
  * 通用 CANopen 初始化（等PRE-OP、配心跳、配PDO、发NMT进OP）由 canopen_init
  * 模块完成，本模块不关心。
  *
  * 流程：等OP → 设模式 → shutdown → switch_on → enable_op → RUNNING
  * 全程非阻塞，靠 cia402_process() 周期驱动。
  */
 #include "cia402.h"

 #include "canopen_master.h"
 #include "301/CO_HBconsumer.h" /* CO_HBconsNode_t, CO_HBconsumer_getIdxByNodeId */
 #include "main.h"         /* HAL_GetTick() */
 
 #include <stdio.h>
 #include <string.h>
 
 /* ═══════════════════════════════════════════════════════════════
  *  私有全局
  * ═══════════════════════════════════════════════════════════════ */
 
 static CO_t* s_co = NULL;
 static cia402_slave_t s_slave;
 
 /* ═══════════════════════════════════════════════════════════════
  *  工具函数
  * ═══════════════════════════════════════════════════════════════ */
 
 static uint32_t now_ms(void) {
     return HAL_GetTick();
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  402 状态转换通用处理（shutdown / switch_on / enable_op）
  *
  *  流程：写控制字 0x6040 → 读状态字 0x6041 确认目标状态
  *  返回：true=转换完成, false=进行中
  * ═══════════════════════════════════════════════════════════════ */
 
 static bool_t cia402_transition_tick(cia402_slave_t* s, uint16_t cmd, uint16_t target_status) {
     (void)cmd;
     co_master_poll();
 
     if (co_master_busy()) {
         if (now_ms() >= s->deadline) {
             s->fault = true;
             s->fault_code = CIA402_ERR_TRANS_TIMEOUT;
         }
         return false;
     }
 
     if (co_master_get_result() != CO_MASTER_OK) {
         s->fault = true;
         s->fault_code = CIA402_ERR_TRANS_SDO;
         return false;
     }
 
     if (s->trans_step == 0) {
         /* 控制字写完了，读状态字确认 */
         s->trans_step = 1;
         co_master_start_read(s->node_id, 0x6041, 0, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
     } else {
         /* 读到状态字了 */
         uint16_t status;
         if (co_master_get_u16(&status)) {
             if ((status & CIA402_STS_MASK) == target_status) {
                 return true;  /* 转换完成 */
             }
         }
         /* 还没到目标状态，再读一次 */
         co_master_start_read(s->node_id, 0x6041, 0, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
     }
 
     return false;
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：enter 函数（状态切换时执行一次）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_enter(cia402_slave_t* s) {
     uint8_t  u8val;
     uint16_t u16val;
 
     s->trans_step = 0;
     s->retry = 0;
 
     switch (s->state) {
     case CIA402_WAIT_OP:
         s->deadline = now_ms() + CIA402_OP_TIMEOUT_MS;
         break;
 
     case CIA402_SET_MODE:
         u8val = (uint8_t)s->target_mode;
         co_master_start_write(s->node_id, 0x6060, 0, &u8val, 1, CIA402_SDO_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
         break;
 
     case CIA402_SHUTDOWN:
         u16val = CIA402_CMD_SHUTDOWN;
         co_master_start_write(s->node_id, 0x6040, 0, &u16val, 2, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
         break;
 
     case CIA402_SWITCH_ON:
         u16val = CIA402_CMD_SWITCH_ON;
         co_master_start_write(s->node_id, 0x6040, 0, &u16val, 2, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
         break;
 
     case CIA402_ENABLE_OP:
         u16val = CIA402_CMD_ENABLE_OP;
         co_master_start_write(s->node_id, 0x6040, 0, &u16val, 2, CIA402_STEP_TIMEOUT_MS);
         s->deadline = now_ms() + CIA402_STEP_TIMEOUT_MS + 100;
         break;
 
     case CIA402_RUNNING:
         printf("CiA402: node=%u init done, RUNNING\r\n", s->node_id);
         break;
 
     case CIA402_FAULT:
         printf("CiA402: node=%u fault, code=0x%04X\r\n", s->node_id, s->fault_code);
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：tick 函数（每周期执行）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_tick(cia402_slave_t* s) {
     CO_HBconsNode_t* node;
 
     switch (s->state) {
 
     /* ── 等从站进 OP（由 canopen_init 发了 NMT 进 OP，这里等心跳确认） ── */
     case CIA402_WAIT_OP:
         node = &s_co->HBcons->monitoredNodes[s->hb_idx];
         if (node->HBstate == CO_HBconsumer_ACTIVE
             && node->NMTstate == CO_NMT_OPERATIONAL) {
             s->state = CIA402_SET_MODE;
         } else if (now_ms() >= s->deadline) {
             s->fault = true;
             s->fault_code = CIA402_ERR_OP_TIMEOUT;
             s->state = CIA402_FAULT;
         }
         break;
 
     /* ── 设置运行模式：写 0x6060，读 0x6061 确认 ── */
     case CIA402_SET_MODE:
         co_master_poll();
         if (co_master_busy()) {
             if (now_ms() >= s->deadline) {
                 s->fault = true; s->fault_code = CIA402_ERR_MODE_TIMEOUT;
                 s->state = CIA402_FAULT;
             }
             break;
         }
         if (co_master_get_result() != CO_MASTER_OK) {
             s->fault = true; s->fault_code = CIA402_ERR_MODE_SDO;
             s->state = CIA402_FAULT;
             break;
         }
         if (s->trans_step == 0) {
             /* 写完模式，读 0x6061 确认 */
             s->trans_step = 1;
             co_master_start_read(s->node_id, 0x6061, 0, CIA402_SDO_TIMEOUT_MS);
             s->deadline = now_ms() + CIA402_SDO_TIMEOUT_MS + 100;
         } else {
             uint8_t actual;
             if (co_master_get_u8(&actual) && actual == (uint8_t)s->target_mode) {
                 s->state = CIA402_SHUTDOWN;
             } else {
                 s->fault = true; s->fault_code = CIA402_ERR_MODE_MISMATCH;
                 s->state = CIA402_FAULT;
             }
         }
         break;
 
     /* ── shutdown ── */
     case CIA402_SHUTDOWN:
         if (cia402_transition_tick(s, CIA402_CMD_SHUTDOWN, CIA402_STS_READY_SWITCH)) {
             s->state = CIA402_SWITCH_ON;
         } else if (s->fault) {
             s->state = CIA402_FAULT;
         }
         break;
 
     /* ── switch on ── */
     case CIA402_SWITCH_ON:
         if (cia402_transition_tick(s, CIA402_CMD_SWITCH_ON, CIA402_STS_SWITCHED_ON)) {
             s->state = CIA402_ENABLE_OP;
         } else if (s->fault) {
             s->state = CIA402_FAULT;
         }
         break;
 
     /* ── enable operation ── */
     case CIA402_ENABLE_OP:
         if (cia402_transition_tick(s, CIA402_CMD_ENABLE_OP, CIA402_STS_OP_ENABLED)) {
             s->state = CIA402_RUNNING;
         } else if (s->fault) {
             s->state = CIA402_FAULT;
         }
         break;
 
     /* ── 运行中 / 故障 ── */
     case CIA402_RUNNING:
     case CIA402_FAULT:
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  状态机分发：状态切换时调 enter，每周期调 tick
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_dispatch(cia402_slave_t* s) {
     if (s->state != s->prev_state) {
         s->prev_state = s->state;
         fsm_enter(s);
     }
     fsm_tick(s);
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  对外接口
  * ═══════════════════════════════════════════════════════════════ */
 
 void cia402_init(CO_t* co, uint8_t nodeId, cia402_mode_t mode) {
     s_co = co;
     memset(&s_slave, 0, sizeof(s_slave));
     s_slave.node_id = nodeId;
     s_slave.target_mode = mode;
     s_slave.state = CIA402_WAIT_OP;
     s_slave.prev_state = (cia402_state_t)-1;  /* 强制第一次进 enter */
 
     /* 获取心跳消费者索引（心跳消费者由 canopen_init 配置） */
     if (s_co != NULL && s_co->HBcons != NULL) {
         s_slave.hb_idx = (uint8_t)CO_HBconsumer_getIdxByNodeId(s_co->HBcons, nodeId);
     }
 
     printf("CiA402: init, node=%u, mode=%d\r\n", nodeId, mode);
 }
 
 void cia402_process(void) {
     fsm_dispatch(&s_slave);
 }
 
 cia402_state_t cia402_get_state(void) {
     return s_slave.state;
 }
 
 uint16_t cia402_get_fault_code(void) {
     return s_slave.fault_code;
 }