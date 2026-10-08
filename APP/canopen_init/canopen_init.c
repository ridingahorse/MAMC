/**
  * @file    canopen_init.c
  * @brief   CANopen 通用初始化状态机实现
  */
 #include "canopen_init.h"

 #include "canopen_master.h"
#include "OD.h"
#include "301/CO_HBconsumer.h"
#include "301/CO_SYNC.h"
#include "main.h"
 
 #include <stdio.h>
 #include <string.h>
 
 /* ═══════════════════════════════════════════════════════════════
  *  私有全局
  * ═══════════════════════════════════════════════════════════════ */
 
 static CO_t* s_co = NULL;
static co_init_slave_t s_slaves[CO_INIT_MAX_SLAVES];
static uint8_t s_hb_next_sub = 1;          /**< 0x1016 下一个可用子索引（从 1 开始，sub0 是数量） */
static bool_t  s_sync_producer_configured = false;  /**< SYNC 生产者只需配置一次（总线级资源） */
 
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
    for (uint8_t i = 0; i < CO_INIT_MAX_SLAVES; i++) {
        if (s_slaves[i].node_id == nodeId && s_slaves[i].state != CO_INIT_DONE) {
            s_slaves[i].online = true;
            break;
        }
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

 /* 通道配置的步骤序列。注意它【对每个通道都一样】，第一个通道也不例外 ——
  * 序列本身没有「首个通道特例」，特例只存在于别处（见 PDO_STEP_DISABLE_COBID）。 */
 typedef enum {
     PDO_STEP_DISABLE_COBID = 0, /**< 写 comm_idx sub1 = cob_id | bit31（禁用本通道） */
     PDO_STEP_CLEAR_MAP,         /**< 写 map_idx  sub0 = 0（清映射） */
     PDO_STEP_WRITE_ENTRY,       /**< 写 map_idx  sub1..subN（按顺序写条目） */
     PDO_STEP_SET_COUNT,         /**< 写 map_idx  sub0 = map_count */
     PDO_STEP_SET_TX_TYPE,       /**< 写 comm_idx sub2 = tx_type */
    PDO_STEP_SET_INHIBIT,       /**< 写 comm_idx sub3 = inhibit_time（仅 TPDO） */
    PDO_STEP_SET_EVENT_TIMER,   /**< 写 comm_idx sub5 = event_timer（仅 TPDO） */
    PDO_STEP_ENABLE,            /**< 写 comm_idx sub1 = cob_id（bit31=0，启用） */
     PDO_STEP_NEXT_CH            /**< 换下一通道（或全部完成） */
 } pdo_step_t;
 
 typedef enum {
     PDO_BUSY = 0,
     PDO_DONE,
     PDO_FAULT,
 } pdo_result_t;
 
 /* 发起一步 SDO 写。闸门刚确认「轮到我、客户端空闲」，所以这里的失败都是
  * 真失败（setup/initiate 失败、参数非法），必须当场暴露出来 ——
  * 吞掉返回值等于把一个**没发出去**的步骤当成成功，状态机会带着残缺的
  * PDO 映射一路走到 OP，最后表现为「配好了、也 RUNNING，但电机不动」。 */
 static pdo_result_t pdo_issue(co_init_slave_t* s, uint16_t index, uint8_t sub,
                               uint32_t value, size_t size)
 {
     if (!co_master_start_write(s->node_id, index, sub, &value, size, CO_INIT_PDO_TIMEOUT_MS)) {
         printf("CO_INIT: node=%u SDO 发起失败 idx=0x%04X sub=%u: %s\r\n",
                (unsigned)s->node_id, (unsigned)index, (unsigned)sub,
                co_master_start_error_name(co_master_get_start_error()));
         return PDO_FAULT;
     }
     s->deadline = now_ms() + CO_INIT_PDO_TIMEOUT_MS + 100;
     return PDO_BUSY;
 }

 /* 这个通道是 RPDO（主站→从站）吗？用通讯参数索引判断 —— CiA 301 的布局是
  * 固定的：0x1400~0x15FF = RPDO，0x1800~0x19FF = TPDO。 */
 static bool_t is_rpdo_channel(const co_pdo_channel_t* ch) {
     return (ch->comm_idx >= 0x1400u) && (ch->comm_idx < 0x1600u);
 }

 /* ═══════════════════════════════════════════════════════════════
 *  PDO 参数回读 —— bring-up 诊断，只读
 * ═══════════════════════════════════════════════════════════════
 * 为什么值得单独走一趟：主站发出去的那一帧，从站是按【它自己的】PDO 映射
 * 来解释的，和我们配置里写了什么毫无关系；而驱动器把哪些 PDO 参数开放给
 * SDO 访问，也完全由它的 OD 决定。这两件事都猜不出来，只能问它。
 *
 * 只读，不改任何东西。读不到不算故障 —— 「这个子索引不给 SDO 访问」本身
 * 就是一条结论，记下来继续问下一个。定位完问题整块删掉即可。
 *
 * 打印串刻意写成 ASCII：串口终端的编码和源文件不一定一致，中文在日志里会
 * 变成乱码，而这一块是专门给人看数据的。 */
typedef struct {
    uint16_t    index;
    uint8_t     sub;
    const char* name;
} pdo_probe_t;

static const pdo_probe_t s_pdo_probe[] = {
    { 0x1400, 1, "RPDO1 COB-ID"       },
    { 0x1400, 2, "RPDO1 transmission" },
    { 0x1600, 0, "RPDO1 map count"    },
    { 0x1600, 1, "RPDO1 map1"         },
    { 0x1600, 2, "RPDO1 map2"         },
    { 0x1600, 3, "RPDO1 map3"         },
    { 0x1401, 1, "RPDO2 COB-ID"       },
    { 0x1601, 0, "RPDO2 map count"    },
    { 0x1601, 1, "RPDO2 map1"         },
    { 0x1601, 2, "RPDO2 map2"         },
    { 0x1800, 1, "TPDO1 COB-ID"       },
    { 0x1800, 2, "TPDO1 transmission" },
    { 0x1800, 5, "TPDO1 event timer"  },
    { 0x1A00, 0, "TPDO1 map count"    },
    { 0x1A00, 1, "TPDO1 map1"         },
    { 0x1A00, 2, "TPDO1 map2"         },
};

#define PDO_PROBE_COUNT (sizeof(s_pdo_probe) / sizeof(s_pdo_probe[0]))

/** @brief 逐条回读 s_pdo_probe，读完返回 PDO_DONE。复用 s->sub_step 当表下标 */
static pdo_result_t
pdo_probe_tick(co_init_slave_t* s) {
    co_master_status_t st = { CO_MASTER_OK, 0 };
    uint8_t i = s->sub_step;   /* 下一个要读的表项；刚读回来的是 i-1 */

    if (i == 0U) {
        printf("CO_INIT: node=%u ---- PDO parameter readback ----\r\n", (unsigned)s->node_id);
    }

    switch (co_master_step_gate(s->node_id, s->deadline, &st)) {
    case CO_SDO_STEP_WAIT:
        return PDO_BUSY;

    case CO_SDO_STEP_DONE:
        if (i > 0U) {
            /* 数据要在取走结果之后【立刻】读：下一次事务发起后接收缓冲就被覆盖了 */
            size_t   len = co_master_get_size();
            uint32_t v   = 0;
            if (len >= 4U) {
                (void)co_master_get_u32(&v);
            } else if (len >= 2U) {
                uint16_t t = 0;
                (void)co_master_get_u16(&t);
                v = t;
            } else {
                uint8_t t = 0;
                (void)co_master_get_u8(&t);
                v = t;
            }
            const pdo_probe_t* p = &s_pdo_probe[i - 1U];
            printf("CO_INIT: node=%u   0x%04X.%02X %-18s = 0x%08lX (%u byte)\r\n",
                   (unsigned)s->node_id, (unsigned)p->index, (unsigned)p->sub, p->name,
                   (unsigned long)v, (unsigned)len);
        }
        break;

    case CO_SDO_STEP_FAIL:
        /* 原因（对象 + abort 码）闸门已经打印过了。读不到不是故障，
         * "驱动器不开放这个子索引"正是我们要的信息。 */
        if (i > 0U) {
            const pdo_probe_t* p = &s_pdo_probe[i - 1U];
            printf("CO_INIT: node=%u   0x%04X.%02X %-18s = (read failed)\r\n",
                   (unsigned)s->node_id, (unsigned)p->index, (unsigned)p->sub, p->name);
        }
        break;

    case CO_SDO_STEP_NEXT:
    default:
        break;
    }

    if (i < PDO_PROBE_COUNT) {
        if (!co_master_start_read(s->node_id, s_pdo_probe[i].index, s_pdo_probe[i].sub,
                                  CO_INIT_SDO_TIMEOUT_MS)) {
            /* 发起都失败就跳过这一条，别让一次本地错误卡住整轮回读 */
            printf("CO_INIT: node=%u   0x%04X.%02X read start failed: %s\r\n",
                   (unsigned)s->node_id, (unsigned)s_pdo_probe[i].index,
                   (unsigned)s_pdo_probe[i].sub,
                   co_master_start_error_name(co_master_get_start_error()));
        }
        s->sub_step = (uint8_t)(i + 1U);
        s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
        return PDO_BUSY;
    }

    printf("CO_INIT: node=%u ---- readback end ----\r\n", (unsigned)s->node_id);
    return PDO_DONE;
}

static pdo_result_t pdo_cfg_tick(co_init_slave_t* s) {
     const co_pdo_config_t* cfg = s->pdo_cfg;

     /* 没配 PDO 表，直接完成（用从站默认映射） */
     if (cfg == NULL || cfg->channels == NULL || cfg->count == 0) {
         return PDO_DONE;
     }

     /* 一步 SDO 的闸门。被别人占着时只等、不计时 ——
      * 少一个掉线的从站把在线的从站一起拖成故障，这是多轴能独立工作的前提。 */
     switch (co_master_step_gate(s->node_id, s->deadline, NULL)) {
     case CO_SDO_STEP_WAIT:
         return PDO_BUSY;
     case CO_SDO_STEP_FAIL:
         return PDO_FAULT;
     case CO_SDO_STEP_NEXT:   /* 本通道还没发起过 */
     case CO_SDO_STEP_DONE:   /* 上一步成功 */
     default:
         break;
     }

     const co_pdo_channel_t* ch = &cfg->channels[s->pdo_channel];

     /* 每个分支：先把子步骤推进到「下一个要发的」，再发出去并返回结果 */
     switch (s->pdo_step) {

     case PDO_STEP_DISABLE_COBID:
         /* 改映射之前必须先禁用本通道的 COB-ID。CiA 301 规定 PDO 处于有效
          * （使能）状态时，它的映射参数是写保护的，驱动器会直接回
          * 0x06010002（试图写只读对象）—— 看起来像"驱动器不支持改映射"，
          * 其实只是少了这一步。
          *
          * 这一步【每个通道都要自己做】，不能指望别人替它做：旧代码只在
          * 换通道时替**下一个**通道禁用，于是通道 0（RPDO1）的 COB-ID 从来
          * 没人碰过 —— 而它默认就是有效的，0x1600 因此永远写不进去；通道 1
          * （TPDO1）每次都被前一步禁用过，0x1A00 就写得进去。同一个驱动器，
          * 一个能改一个不能改，差别只在这里。 */
         s->pdo_step = PDO_STEP_CLEAR_MAP;
         s->pdo_entry_idx = 0;
         return pdo_issue(s, ch->comm_idx, 1, ch->cob_id | 0x80000000u, 4);

     case PDO_STEP_CLEAR_MAP:
         /* 清映射 = 让驱动器进入「可编辑态」。CiA 301 规定映射条目只有在条目数
          * sub0 为 0 时才可写，sub0 = 0 就是那个开关。
          *
          * 少了这一步，紧随其后的每一笔条目写都会被驱动器拒掉，回
          * 0x06010000（不支持访问 —— 含义是"此刻不允许访问"，而不是
          * 0x06010002 的"这个对象只读"）。手册写着 0x1600 可读写，和它
          * "此刻不让你写"并不矛盾，差别就在这一步。 */
         s->pdo_entry_idx = 0;
         s->pdo_step = PDO_STEP_WRITE_ENTRY;
         return pdo_issue(s, ch->map_idx, 0, 0, 1);   /* 清映射 sub0 = 0 */

     case PDO_STEP_WRITE_ENTRY:
         if (s->pdo_entry_idx < ch->map_count) {
             uint8_t  sub = (uint8_t)(s->pdo_entry_idx + 1);
             uint32_t v   = ch->map_entries[s->pdo_entry_idx];
             s->pdo_entry_idx++;
             return pdo_issue(s, ch->map_idx, sub, v, 4);
         }
         s->pdo_step = PDO_STEP_SET_COUNT;
         return PDO_BUSY;   /* 条目写完，下个周期发映射条目数 */

     case PDO_STEP_SET_COUNT:
        s->pdo_step = PDO_STEP_SET_TX_TYPE;
        return pdo_issue(s, ch->map_idx, 0, ch->map_count, 1);

    case PDO_STEP_SET_TX_TYPE: {
        /* 同步模式下，RPDO（主站→从站，comm_idx 0x1400~0x15FF）覆盖为 0x01：
         *   从站收到 RPDO 后先缓存，等 SYNC 到来时才写入对象字典，实现多轴同步。
         * TPDO（从站→主站，comm_idx 0x1800~0x19FF）保持原配置（异步定时上报）。 */
        uint32_t v = ch->tx_type;
        if (cfg->use_sync && is_rpdo_channel(ch)) {
            v = 0x01u;
        }
        s->pdo_step = PDO_STEP_SET_INHIBIT;
        return pdo_issue(s, ch->comm_idx, 2, v, 1);
    }

    case PDO_STEP_SET_INHIBIT:
        s->pdo_step = PDO_STEP_SET_EVENT_TIMER;
        if (is_rpdo_channel(ch)) {
            return PDO_BUSY;    /* 见下方注释：RPDO 没有这个子索引，跳过 */
        }
        return pdo_issue(s, ch->comm_idx, 3, ch->inhibit_time, 2);

    case PDO_STEP_SET_EVENT_TIMER:
        s->pdo_step = PDO_STEP_ENABLE;
        /* 禁止时间（sub3）和事件定时器（sub5）是**发送方向专有**的概念：
         * 「多久之内不许再发」「定时发一次」。接收方向（RPDO）既不定时发送、
         * 也谈不上抑制，CiA 301 里这两项对 RPDO 无定义，多数驱动器就把它们
         * 实现成只读常量 —— 写进去会回 0x06010002（试图写只读对象）。
         * 对 RPDO 直接跳过：少两条没有意义的 SDO，初始化更快，
         * 也不会再因为一个"本来就写不进去、写了也没用"的对象判故障。 */
        if (is_rpdo_channel(ch)) {
            return PDO_BUSY;    /* 下个周期处理 ENABLE */
        }
        return pdo_issue(s, ch->comm_idx, 5, ch->event_timer, 2);

    case PDO_STEP_ENABLE:
        s->pdo_step = PDO_STEP_NEXT_CH;
        return pdo_issue(s, ch->comm_idx, 1, ch->cob_id, 4);   /* bit31=0，启用 */

     case PDO_STEP_NEXT_CH:
     default:
         /* 当前通道完成，换下一个（或全部完成）。这里【不碰总线】——
          * 下一通道的第一步是「禁用自己」，交给下一轮 pdo_cfg_tick 按统一
          * 序列发起，换通道这一步就不再需要知道"新通道要写什么"了。 */
         s->pdo_channel++;
         if (s->pdo_channel >= cfg->count) {
             return PDO_DONE;
         }
         s->pdo_step = PDO_STEP_DISABLE_COBID;
         s->pdo_entry_idx = 0;
         return PDO_BUSY;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：enter（状态切换时执行一次）
  * ═══════════════════════════════════════════════════════════════ */
 
 /*  enter 只负责摆状态（子步骤、超时），**不碰总线**。
  *  所有 SDO 一律由 fsm_tick 经 co_master_step_gate 发起 ——
  *  I/O 只有一个出口，才谈得上「轮到我了吗、刚才是成是败」有唯一答案。 */
 static void fsm_enter(co_init_slave_t* s) {
     s->sub_step = 0;
     /* 注意：这里【不清】s->retry。它记的是「这块从站重来过几次」，
      * 属于整个初始化流程的状态，不能每次进状态都被抹掉 ——
      * 抹掉就等于无限重试。只有真正成功了（CO_INIT_DONE）才清零。 */

     switch (s->state) {
     case CO_INIT_WAIT_PREOP:
         s->deadline = now_ms() + CO_INIT_PREOP_TIMEOUT_MS;
         break;

     case CO_INIT_READ_IDENTITY:
     case CO_INIT_READ_ERROR_REG:
         s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
         break;
 
     case CO_INIT_CFG_HEARTBEAT:
     case CO_INIT_PROBE_PDO:
         /* 只摆好状态，SDO 由 fsm_tick 经闸门统一发起 */
         s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
         break;

     case CO_INIT_CFG_PDO: {
         s->pdo_channel = 0;
         s->pdo_entry_idx = 0;
         const co_pdo_config_t* cfg = s->pdo_cfg;
         if (cfg != NULL && cfg->use_sync) {
            /* SYNC 生产者是总线级资源，只配置一次。
             * 所有从站应使用相同的 sync_period_us，以第一个配置为准。 */
            if (!s_sync_producer_configured) {
                OD_set_u32(OD_ENTRY_H1005, 0, 0x40000080u, false);
                OD_set_u8(OD_ENTRY_H1019, 0, 0u, false);
                OD_set_u32(OD_ENTRY_H1006, 0, cfg->sync_period_us, false);
                s_sync_producer_configured = true;
                printf("CO_INIT: SYNC producer enabled, period=%lu us\r\n",
                       (unsigned long)cfg->sync_period_us);
            }
        }
         /* 第一步（禁用第一个通道的 COB-ID）由 pdo_cfg_tick 的
          * PDO_STEP_DISABLE_COBID 分支发起 */
         s->pdo_step = PDO_STEP_DISABLE_COBID;
         s->deadline = now_ms() + CO_INIT_PDO_TIMEOUT_MS + 100;
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
         s->retry = 0;   /* 这次真的成了，重试计数归零 */
         printf("CO_INIT: node=%u 进入 OP，通用初始化完成\r\n", s->node_id);
         break;

     case CO_INIT_FAULT:
         /* 先歇一会儿再重来，别一进故障就立刻重试 —— 从站刚掉线时
          * 总线上可能还在收尾，马上重试只会再失败一次。 */
         s->deadline = now_ms() + CO_INIT_FAULT_RETRY_MS;
         printf("CO_INIT: node=%u 故障 code=0x%04X（已重试 %u/%u 次）\r\n",
                s->node_id, s->fault_code, (unsigned)s->retry, (unsigned)CO_INIT_FAULT_RETRY_MAX);
         break;
     }
 }
 
 /* ═══════════════════════════════════════════════════════════════
  *  大状态机：tick（每周期执行）
  * ═══════════════════════════════════════════════════════════════ */
 
 static void fsm_tick(co_init_slave_t* s) {
    CO_HBconsNode_t* node;
    uint8_t u8val;

    /* 这里不再需要「SDO 令牌」—— 串行化已经由 co_master_step_gate 的归属
     * 判断完成，而且是按节点判断的：别的从站在用 SDO 时本状态机只会
     * WAIT，各走各的，谁的事务超时就算在谁头上。 */

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
        /* bootup(CO_NMT_INITIALIZING) 也表示从站初始化完成、可接收 SDO。
         * 部分从站 0x1017 默认=0 不主动发心跳，若只等 PRE_OPERATIONAL 会死锁。 */
        if (s->online && (node->NMTstate == CO_NMT_PRE_OPERATIONAL
                          || node->NMTstate == CO_NMT_INITIALIZING)) {
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
         switch (co_master_step_gate(s->node_id, s->deadline, NULL)) {
         case CO_SDO_STEP_WAIT:
             break;
         case CO_SDO_STEP_FAIL:
             s->fault = true; s->fault_code = CO_INIT_ERR_IDENTITY_SDO;
             s->state = CO_INIT_FAULT;
             break;
         case CO_SDO_STEP_DONE: {
             uint32_t vendor_id;
             if (co_master_get_u32(&vendor_id)) {
                 printf("CO_INIT: node=%u vendor ID = 0x%08lX\r\n",
                        (unsigned)s->node_id, (unsigned long)vendor_id);
                 s->state = CO_INIT_READ_ERROR_REG;
             } else {
                 /* 读到的东西长度不对 —— 通信成功但内容不是我们要的 */
                 s->fault = true; s->fault_code = CO_INIT_ERR_IDENTITY_SDO;
                 s->state = CO_INIT_FAULT;
             }
             break;
         }
         case CO_SDO_STEP_NEXT:
         default:
             if (!co_master_start_read(s->node_id, 0x1018, 1, CO_INIT_SDO_TIMEOUT_MS)) {
                 printf("CO_INIT: node=%u 读 0x1018 发起失败: %s\r\n",
                        (unsigned)s->node_id,
                        co_master_start_error_name(co_master_get_start_error()));
                 s->fault = true; s->fault_code = CO_INIT_ERR_IDENTITY_SDO;
                 s->state = CO_INIT_FAULT;
                 break;
             }
             s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
             break;
         }
         break;

     case CO_INIT_READ_ERROR_REG:
         switch (co_master_step_gate(s->node_id, s->deadline, NULL)) {
         case CO_SDO_STEP_WAIT:
             break;
         case CO_SDO_STEP_FAIL:
             s->fault = true; s->fault_code = CO_INIT_ERR_ERRREG_SDO;
             s->state = CO_INIT_FAULT;
             break;
         case CO_SDO_STEP_DONE:
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
            break;
         case CO_SDO_STEP_NEXT:
         default:
             if (!co_master_start_read(s->node_id, 0x1001, 0, CO_INIT_SDO_TIMEOUT_MS)) {
                 printf("CO_INIT: node=%u 读 0x1001 发起失败: %s\r\n",
                        (unsigned)s->node_id,
                        co_master_start_error_name(co_master_get_start_error()));
                 s->fault = true; s->fault_code = CO_INIT_ERR_ERRREG_SDO;
                 s->state = CO_INIT_FAULT;
                 break;
             }
             s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
             break;
         }
         break;

     case CO_INIT_CFG_HEARTBEAT:
         switch (co_master_step_gate(s->node_id, s->deadline, NULL)) {
         case CO_SDO_STEP_WAIT:
             break;
         case CO_SDO_STEP_FAIL:
             s->fault = true; s->fault_code = CO_INIT_ERR_HB_SDO;
             s->state = CO_INIT_FAULT;
             break;
         case CO_SDO_STEP_DONE:
             s->state = CO_INIT_PROBE_PDO;   /* 先回读一遍驱动器自己的 PDO 参数 */
             break;
         case CO_SDO_STEP_NEXT:
         default: {
             uint16_t hb_ms = CO_INIT_HB_PRODUCER_MS;
             if (!co_master_start_write(s->node_id, 0x1017, 0, &hb_ms, 2, CO_INIT_SDO_TIMEOUT_MS)) {
                 printf("CO_INIT: node=%u 写 0x1017 发起失败: %s\r\n",
                        (unsigned)s->node_id,
                        co_master_start_error_name(co_master_get_start_error()));
                 s->fault = true; s->fault_code = CO_INIT_ERR_HB_SDO;
                 s->state = CO_INIT_FAULT;
                 break;
             }
             s->deadline = now_ms() + CO_INIT_SDO_TIMEOUT_MS + 100;
             break;
         }
         }
         break;
 
     case CO_INIT_PROBE_PDO:
        /* 只读诊断：结果只打印，不判故障 —— 读到什么都往下走，
         * 真正决定成败的是紧随其后的 CFG_PDO。 */
        if (pdo_probe_tick(s) == PDO_DONE) {
            s->state = CO_INIT_CFG_PDO;
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
         break;

     case CO_INIT_FAULT:
         /* 有限次重试：从头再走一遍初始化。retry 只在 CO_INIT_DONE 里清零，
          * 所以「重试 N 次都失败」会在这里收敛下来，不会无限刷。 */
         if (s->retry < CO_INIT_FAULT_RETRY_MAX && now_ms() >= s->deadline) {
             s->retry++;
             s->fault = false;
             s->fault_code = CO_INIT_ERR_NONE;
             s->online = false;      /* 重新等心跳，别拿上一次的上线状态当数 */
             printf("CO_INIT: node=%u 重试第 %u 次\r\n", (unsigned)s->node_id, (unsigned)s->retry);
             s->state = CO_INIT_WAIT_PREOP;
         }
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
 
 void co_init_start(uint8_t idx, CO_t* co, uint8_t nodeId, const co_pdo_config_t* pdo_cfg) {
    if (idx >= CO_INIT_MAX_SLAVES) return;
    s_co = co;
    co_init_slave_t* s = &s_slaves[idx];
    memset(s, 0, sizeof(*s));
    s->node_id = nodeId;
    s->pdo_cfg = pdo_cfg;
    s->state = CO_INIT_WAIT_PREOP;
    s->prev_state = (co_init_state_t)-1;

    /* 配置心跳消费者：OD 0x1016 写入 (nodeId<<16) | CO_INIT_HB_TIMEOUT_MS
     * 每个从站占一个子索引（sub1, sub2, ...），按注册顺序分配。 */
    if (s_co != NULL && s_co->HBcons != NULL) {
        uint8_t hb_sub = s_hb_next_sub;
        if (hb_sub >= s_co->HBcons->numberOfMonitoredNodes) {
            printf("CO_INIT: ERROR heartbeat table full for node=%u\r\n", nodeId);
            s->hb_idx = 0xFF;
        } else {
            uint32_t entry = ((uint32_t)nodeId << 16) | CO_INIT_HB_TIMEOUT_MS;
            OD_set_u32(OD_ENTRY_H1016_consumerHeartbeatTime, hb_sub, entry, false);
            s_hb_next_sub++;

            int8_t hbidx = CO_HBconsumer_getIdxByNodeId(s_co->HBcons, nodeId);
            if (hbidx >= 0 && hbidx < s_co->HBcons->numberOfMonitoredNodes) {
                s->hb_idx = (uint8_t)hbidx;
                CO_HBconsumer_initCallbackRemoteReset(s_co->HBcons, s->hb_idx, NULL, on_bootup);
            } else {
                s->hb_idx = 0xFF;
                printf("CO_INIT: ERROR heartbeat consumer not found for node=%u\r\n", nodeId);
            }
        }
    } else {
        s->hb_idx = 0xFF;
    }

    printf("CO_INIT: start idx=%u node=%u\r\n", idx, nodeId);
}

void co_init_process(uint8_t idx) {
    if (idx >= CO_INIT_MAX_SLAVES) return;
    fsm_dispatch(&s_slaves[idx]);
}

void co_init_restart(uint8_t idx, const co_pdo_config_t* new_cfg) {
    if (idx >= CO_INIT_MAX_SLAVES) return;
    co_init_slave_t* s = &s_slaves[idx];
    s->pdo_cfg = new_cfg;
    s->online = false;
    s->fault = false;
    s->fault_code = 0;
    s->retry = 0;   /* 上层主动要求重来，等于重新给一次重试预算 */
    s->state = CO_INIT_PDO_RECONFIG;
    s->prev_state = (co_init_state_t)-1;
    printf("CO_INIT: idx=%u restart with new PDO config\r\n", idx);
}

bool_t co_init_done(uint8_t idx) {
    if (idx >= CO_INIT_MAX_SLAVES) return false;
    return s_slaves[idx].state == CO_INIT_DONE;
}

co_init_state_t co_init_get_state(uint8_t idx) {
    if (idx >= CO_INIT_MAX_SLAVES) return CO_INIT_FAULT;
    return s_slaves[idx].state;
}

uint16_t co_init_get_fault_code(uint8_t idx) {
    if (idx >= CO_INIT_MAX_SLAVES) return 0;
    return s_slaves[idx].fault_code;
}

void co_sync_send(void) {
    if (s_co != NULL && s_co->SYNC != NULL) {
        CO_SYNCsend(s_co->SYNC);
    }
}