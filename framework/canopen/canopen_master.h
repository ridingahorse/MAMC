/**
  * @file    canopen_master.h
  * @brief   CANopen 主站侧的对象字典访问接口：SDO 客户端事务 + NMT 命令
  *
  * 上游的 SDO 客户端是【非阻塞】的：调用者发起后必须反复调
  * CO_SDOclientUpload()/Download()，直到它返回 <= 0 才算结束。本模块把这套
  * 「发起 → 轮询 → 取结果」的流程包成一次**事务（transaction）**。
  *
  * ── 事务有主：为什么 API 里到处是 nodeId ─────────────────────────────
  * 全工程只有【一个】SDO 客户端，却同时被多个状态机使用（每个从站的
  * canopen_init 一份、每个从站的 cia402 一份）。所以「客户端在忙」这句话
  * 本身是不完整的 —— 必须回答「在替谁忙」。本模块因此把归属（ownership）
  * 做进事务里：
  *
  *     co_master_owner_of(node)   客户端相对【这个节点】是什么状态
  *     co_master_take_result()    取走【属于这个节点】的结果，读一次就消费
  *     co_master_step_gate()      合成上面两件事 + 超时判断 = 「一步 SDO」
  *
  * 少了归属，任何一个从站的事务超时都会被别的从站读成「我失败了」——
  * 一个掉线的从站能把整条总线上的初始化全部拖垮。用状态机的调用方
  * 不要自己判断 busy/result，一律套 co_master_step_gate 的固定形状。
  *
  * 同一时刻只允许一次事务；忙期间再调 start_* 会直接返回 false。
  * start_* 的返回值**必须检查**：发起失败时客户端是空闲的，不检查的话，
  * 下一个周期看到的现象和「事务成功」完全一样，状态机会静默地往下走。
  *
  * ── 为什么不做成阻塞函数 ────────────────────────────────────────────
  * SDO 报文的**接收**在 CAN 中断里完成（CO_SDOclient_receive 是注册给驱动的
  * 回调），但**状态推进**要靠调用者周期性调 CO_SDOclientUpload/Download。
  * 如果在这里死等，主循环就停了，canopen_app_process() 不再执行 —— NMT、
  * SDO 服务器、心跳全都停摆，事务反而等不到结果。所以必须非阻塞。
  *
  * ── 层归属 ──────────────────────────────────────────────────────────
  * 这是 CANopen 协议栈的通用主站能力，不含任何业务语义，因此放在
  * framework/canopen/ 而不是 APP/（见 APP/README.md 的分层原则）。
  */
#ifndef CANOPEN_MASTER_H
#define CANOPEN_MASTER_H

#include "CANopen.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 一次 SDO 事务的最终结果 */
typedef enum {
    CO_MASTER_OK = 0,     /**< 成功结束 */
    CO_MASTER_ERR_ARG,    /**< 参数非法，或 SDO 客户端不可用、上一次事务还没结束 */
    CO_MASTER_ERR_ABORT,  /**< 从站返回了 SDO abort（错误码见 co_master_status_t.abort） */
    CO_MASTER_ERR_TIMEOUT /**< 从站不应答，超时（abort 码为 0x05040000） */
} co_master_result_t;

/**
  * @brief  SDO 客户端相对【某个节点】的占用状态
  * @note   存在的理由：客户端是全局单例，而调用方有多个。少了归属，
  *         「忙」就是一句无法回答的话 —— 谁都能把别人的事务当成自己的。
  */
typedef enum {
    CO_MASTER_FREE    = 0,  /**< 轮到你：可以发起新事务 */
    CO_MASTER_MINE    = 1,  /**< 你的事务正在总线上跑，等结果 */
    CO_MASTER_OTHER   = 2,  /**< 别人占着（事务在跑，或其结果还没被取走） */
    CO_MASTER_SETTLED = 3,  /**< 你的事务已结束，结果待取走（调 co_master_take_result） */
} co_master_owner_t;

/** @brief 一次事务的结果 + 从站给的 SDO abort 码（失败时才有意义） */
typedef struct {
    co_master_result_t result;
    uint32_t           abort;
} co_master_status_t;

/**
  * @brief  「一步 SDO」的推进结论，由 co_master_step_gate 返回
  * @note   NEXT 与 DONE 在多数调用点走同一条路（「去发起下一步」），之所以分开，
  *         是让需要区分「首次进入本状态」和「上一步刚成功」的状态机 —— 比如
  *         402 的「先写控制字、再反复读状态字」握手 —— 能分辨这两种情况。
  */
typedef enum {
    CO_SDO_STEP_WAIT = 0,  /**< 还没轮到我，或我的事务在跑且未超时：本次不推进 */
    CO_SDO_STEP_NEXT,      /**< 轮到我，且没有待取的结果：发起第一步 */
    CO_SDO_STEP_DONE,      /**< 上一步的结果已取走且成功：可以推进到下一步 */
    CO_SDO_STEP_FAIL,      /**< 我自己的事务失败了，或等我自己的事务等到超时 */
} co_sdo_step_t;

/**
  * @brief  co_master_start_read/write 返回 false 的原因
  * @note   有了它，发起失败就不再是静默的 —— 状态机死循环重试却说不出为什么，
  *         是 bring-up 阶段最难查的一类问题。
  */
typedef enum {
    CO_MASTER_START_OK = 0,       /**< 发起成功 */
    CO_MASTER_START_NOT_INIT,     /**< co_master_init 没调，协议栈没起来 */
    CO_MASTER_START_BUSY,         /**< 上一次事务还没结束 */
    CO_MASTER_START_BAD_NODE,     /**< 节点号非法，或等于本机节点号 */
    CO_MASTER_START_BAD_ARG,      /**< 数据为空或超长 */
    CO_MASTER_START_SETUP_FAIL,   /**< CO_SDOclient_setup 失败（CAN 缓冲区登记不上）*/
    CO_MASTER_START_INITIATE_FAIL /**< SDO 客户端拒绝发起（通道无效等）*/
} co_master_start_err_t;

/**
  * @brief  绑定协议栈对象，必须在 canopen_app_init() 成功之后调用一次
  * @param  co           协议栈对象（即 CANopenNodeSTM32.canOpenStack）
  * @param  localNodeId  本节点自己的节点号。用于拦住「SDO 目标就是自己」这种
  *                      配置错误 —— 本工程没开 CAN 回环，自己发给自己收不到应答
  */
void co_master_init(CO_t* co, uint8_t localNodeId);

/**
  * @brief  发起一次 SDO 上传（读远端对象）
  * @param  nodeId      目标从站节点号（1..127）
  * @param  index       对象索引
  * @param  subIndex    子索引
  * @param  timeout_ms  等待从站应答的超时，一般给 500~1000
  * @return true 已发起（接下来由 co_master_step_gate 推进）；false 没发起成，事务未占用
  * @note   读回的数据在事务结束后用 co_master_get_* 取
  */
bool_t co_master_start_read(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t timeout_ms);

/**
  * @brief  发起一次 SDO 下载（写远端对象）
  * @param  data 待写数据，长度即对象长度（本模块上限 CO_MASTER_MAX_DATA 字节）
  * @param  size 字节数；写 0x6040 控制字这类 2 字节对象就传 2
  * @return true 已发起；false 没发起成
  * @note   最大 4 字节的对象会走「快速传输」，一次报文写完；
  *         超过 4 字节自动走分段传输（CO_CONFIG_SDO_CLI_SEGMENTED 已开）
  */
bool_t co_master_start_write(uint8_t nodeId, uint16_t index, uint8_t subIndex, const void* data, size_t size,
                             uint16_t timeout_ms);

/**
  * @brief  查询 SDO 客户端相对某个节点的占用状态
  */
co_master_owner_t co_master_owner_of(uint8_t nodeId);

/**
  * @brief  取走属于 nodeId 的事务结果。**读一次就消费**，再读返回 false
  * @param  out  可为 NULL（只消费不关心内容）
  * @return true  out 已填好；false 当前没有属于该节点的待取结果
  * @note   读到的数据（co_master_get_u32/u16/u8）要在取走结果后【立刻】读 ——
  *         下一次事务发起后接收缓冲会被覆盖。
  */
bool_t co_master_take_result(uint8_t nodeId, co_master_status_t* out);

/**
  * @brief  统一的「一步 SDO」闸门：轮询 + 归属判断 + 超时判断 + 取结果
  *
  * 所有用 SDO 的状态机都套这一个形状，不要自己调 start_* / 判断忙碌：
  *
  *     switch (co_master_step_gate(s->node_id, s->deadline, &st)) {
  *     case CO_SDO_STEP_WAIT:  return;                 // 等，下个周期再看
  *     case CO_SDO_STEP_FAIL:  fault(...); return;     // st.abort 里有 abort 码
  *     case CO_SDO_STEP_DONE:  ...读数据、推进子步骤...; break;
  *     case CO_SDO_STEP_NEXT:  break;                  // 首次，直接发起
  *     }
  *     if (!co_master_start_write(...)) { fault(...); }   // 必须检查返回值
  *     s->deadline = now_ms() + 超时 + 余量;
  *
  * @param  nodeId      本状态机负责的从站
  * @param  deadline_ms 只对【我自己的】事务计时。被别人占着不算我的超时 ——
  *                     否则一个掉线的从站会让别的轴跟着超时判故障。
  * @param  out         失败时填入 abort 码；可传 NULL
  * @return 见 co_sdo_step_t
  * @note   FAIL 时本函数已经把结果打印出来了（节点 + 读/写 + 对象索引 + 原因 + abort 码）。
  *         事务失败只在这一个地方落日志，就不会再出现「轴不动、串口什么也没说」。
  */
co_sdo_step_t co_master_step_gate(uint8_t nodeId, uint32_t deadline_ms, co_master_status_t* out);

/** @brief 取最近一次 co_master_start_* 返回 false 的原因 */
co_master_start_err_t co_master_get_start_error(void);

/** @brief 发起失败原因转可读字符串，用于 printf */
const char* co_master_start_error_name(co_master_start_err_t err);

/** @brief 事务结果转可读字符串，用于 printf */
const char* co_master_result_name(co_master_result_t result);

/** @brief 取最近一次【读】事务收到的字节数 */
size_t co_master_get_size(void);

/**
  * @brief  把最近一次【读】的结果按小端解释成整数
  * @return true 成功；false 当前没有数据或长度不足
  */
bool_t co_master_get_u32(uint32_t* out);
bool_t co_master_get_u16(uint16_t* out);
bool_t co_master_get_u8(uint8_t* out);

/**
  * @brief  发送 NMT 主站命令（需要 CO_CONFIG_NMT_MASTER，本项目已开）
  * @param  command  CO_NMT_command_t，常用：
  *                  CO_NMT_ENTER_OPERATIONAL(1) 启站
  *                  CO_NMT_STOPPED(2)           停站
  *                  CO_NMT_PRE_OPERATIONAL(128) 退回预操作
  *                  CO_NMT_RESET_NODE(129)      复位节点
  *                  CO_NMT_RESET_COMMUNICATION(130) 复位通信
  * @param  nodeId   目标节点号；0 表示广播（所有从站）
  * @return true 已交给 CAN 发送队列；false 参数非法或发送缓冲满
  * @note   NMT 是【无应答】的单向报文，返回 true 只代表发出去了，
  *         不代表从站真的执行了 —— 要确认状态得读从站的 0x6041 之类
  */
bool_t co_master_nmt_send(CO_NMT_command_t command, uint8_t nodeId);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_MASTER_H */
