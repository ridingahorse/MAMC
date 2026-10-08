/**
  * @file    canopen_master.c
  * @brief   CANopen 主站侧 SDO 客户端事务与 NMT 命令的实现
  */
#include "canopen_master.h"

#include "main.h" /* HAL_GetTick() */

#include <stdio.h>
#include <string.h>

/* ── 私有状态 ─────────────────────────────────────────────────── */

/* 结果缓冲区大小。SDO 客户端内部 FIFO 是 CO_CONFIG_SDO_CLI_BUFFER_SIZE(32)
 * 字节，本模块按同样的量级开缓冲，够读 0x1018 这种 4×4 字节的数组；
 * 再长的对象（如 0x1003 错误历史、字符串）需要分块读，本模块不支持。 */
#define CO_MASTER_MAX_DATA 32U

/* 「孤儿事务」回收时限。正常运行时永远触发不到 —— 调用方每轮过闸门，
 * 发起和取结果是同一处代码，一个事务最多活一个主循环周期。
 * 它只兜一种情况：某个调用方发起了事务（或留下结果）之后就再也不推进了，
 * 那会让客户端被永久占住、其它从站全部饿死。宁可把这种事务按超时收掉，
 * 也不要让整条总线因为一个掉队的调用方而停摆。 */
#define CO_MASTER_ORPHAN_TIMEOUT_MS 5000U

typedef enum {
    XFER_IDLE = 0, /**< 没有事务在跑 */
    XFER_READ,     /**< SDO 上传（读远端）进行中 */
    XFER_WRITE     /**< SDO 下载（写远端）进行中 */
} xfer_dir_t;

static CO_t* s_co = NULL;
static uint8_t s_localNodeId = 0;

static xfer_dir_t s_dir = XFER_IDLE;
static uint32_t s_lastTick = 0;
static uint32_t s_abort = 0;
static co_master_result_t s_result = CO_MASTER_OK;

/* ── 事务归属 ─────────────────────────────────────────────────────
 * 客户端全局唯一，调用方却有多份（每个从站的 canopen_init、每个从站的
 * cia402）。s_owner 记录当前/最近一次事务替哪个节点跑的，s_result_ready
 * 表示结果还没被它的主人取走。这两者合起来才让每个调用方能正确回答
 * 「现在轮到我了吗」「刚才是成是败」。 */
static uint8_t  s_owner = 0;             /**< 0 = 无归属 */
static bool_t   s_result_ready = false;  /**< s_result / s_abort 待 s_owner 取走 */
static uint32_t s_settled_tick = 0;      /**< 结果就绪时刻，用于回收孤儿结果 */
static uint32_t s_xfer_tick = 0;         /**< 事务发起时刻，用于回收孤儿事务 */

static uint8_t s_buf[CO_MASTER_MAX_DATA];
static size_t s_len = 0;

/* 当前/刚结束的事务访问的是哪个对象、哪个方向。只为一个目的：失败时能说清
 * 「是哪一条 SDO 被拒了」。少了它，日志只能说到「从站拒绝了」——
 * 知道被拒，但不知道拒的是谁，等于还得再猜一轮。 */
static uint16_t s_index = 0;
static uint8_t s_subIndex = 0;
static xfer_dir_t s_lastDir = XFER_READ; /**< 刚结束的那笔事务的方向 */

static co_master_start_err_t s_startErr = CO_MASTER_START_OK;

/* 事务只在主循环上下文里推进（co_master_start_* 与 co_master_step_gate 都在
 * 主循环调用），CAN 中断只负责把报文塞进协议栈，不碰这些变量。 */

/* ── 内部工具 ─────────────────────────────────────────────────── */

/** @brief 结束当前事务：关掉 SDO 客户端、把结果记到归属者名下 */
static void
xfer_finish(co_master_result_t result, uint32_t abortCode) {
    /* 关客户端是必需的：CO_SDOclientClose() 会把状态置回 IDLE，而
     * CO_SDOclient_receive() 只在状态非 IDLE 时才收报文（见上游实现），
     * 所以这一步等于关掉该 SDO 通道的接收，避免关事务后还有报文写进来。 */
    CO_SDOclientClose(&s_co->SDOclient[0]);
    s_lastDir = s_dir;   /* 记下方向，s_dir 马上要清零 */
    s_dir = XFER_IDLE;
    s_result = result;
    s_abort = abortCode;
    /* 归属者【保留】—— 结果等着它来取。取走之前别人不能发起新事务，
     * 否则这个结果就被覆盖了，归属者会把「没发出去的步骤」当成成功。 */
    s_result_ready = true;
    s_settled_tick = HAL_GetTick();
}

/** @brief 回收没人管的事务状态，防止一个掉队的调用方占死客户端 */
static void
reap(void) {
    uint32_t now = HAL_GetTick();
    if (s_dir != XFER_IDLE) {
        if ((uint32_t)(now - s_xfer_tick) >= CO_MASTER_ORPHAN_TIMEOUT_MS) {
            xfer_finish(CO_MASTER_ERR_TIMEOUT, (uint32_t)CO_SDO_AB_TIMEOUT);
        }
    } else if (s_result_ready) {
        if ((uint32_t)(now - s_settled_tick) >= CO_MASTER_ORPHAN_TIMEOUT_MS) {
            s_result_ready = false;
        }
    }
}

/** @brief 发起事务前的公共检查与客户端配置 */
static bool_t
xfer_begin(uint8_t nodeId, xfer_dir_t dir) {
    if (s_co == NULL) {
        s_startErr = CO_MASTER_START_NOT_INIT;
        return false;
    }
    if (s_dir != XFER_IDLE) {
        s_startErr = CO_MASTER_START_BUSY;
        return false; /* 上一次事务还没结束 */
    }
    if (s_result_ready) {
        /* 上一条事务的结果还没被它的主人取走。正常路径下到不了这里
         * （调用方过闸门时会当场取走），到了说明有调用方绕过了闸门 ——
         * 拒绝比把别人待取的结果覆盖掉安全。 */
        s_startErr = CO_MASTER_START_BUSY;
        return false;
    }
    if ((nodeId == 0U) || (nodeId > 127U)) {
        s_startErr = CO_MASTER_START_BAD_NODE;
        return false; /* 节点号非法（0 是广播，不能用来做 SDO） */
    }
    if (nodeId == s_localNodeId) {
        s_startErr = CO_MASTER_START_BAD_NODE;
        /* 目标是自己。此时 SDO 请求发到自己头上、由本地 SDO 服务器处理，
         * 但本工程的 bxCAN 没开回环（见 can.c 的 MX_CAN1_Init），自己发的
         * 帧自己收不到，应答永远等不来。宁可在这里直接失败也不要空等超时。 */
        return false;
    }

    /* 按目标节点重配 SDO 客户端 CAN 通道：C2S = 0x600+nodeId，S2C = 0x580+nodeId。
     * 本工程未开 CO_CONFIG_FLAG_OD_DYNAMIC，所以 setup() 不做「参数没变就跳过」
     * 的优化，一定会重新登记 CAN 收/发缓冲 —— 这正是需要的：换轴读写时不会
     * 残留上一轴的接收缓冲。（顺带也说明 0x1280 不用配，见该函数实现。） */
    if (CO_SDOclient_setup(&s_co->SDOclient[0], (uint32_t)CO_CAN_ID_SDO_CLI + nodeId,
                           (uint32_t)CO_CAN_ID_SDO_SRV + nodeId, nodeId) != CO_SDO_RT_ok_communicationEnd) {
        s_startErr = CO_MASTER_START_SETUP_FAIL;
        return false;
    }

    /* 超时时间不在这里传：它属于「这次传输」的参数，由调用方在
     * CO_SDOclientUploadInitiate()/DownloadInitiate() 里给。 */

    s_dir = dir;
    s_owner = nodeId;        /* 这笔事务从这个节点名下走 */
    s_abort = 0;
    s_result = CO_MASTER_OK;
    s_len = 0;
    s_startErr = CO_MASTER_START_OK;
    s_lastTick = HAL_GetTick();
    s_xfer_tick = s_lastTick;
    return true;
}

/* ── 对外接口 ─────────────────────────────────────────────────── */

void
co_master_init(CO_t* co, uint8_t localNodeId) {
    s_co = co;
    s_localNodeId = localNodeId;
    s_dir = XFER_IDLE;
    s_result = CO_MASTER_OK;
    s_abort = 0;
    s_len = 0;
    s_owner = 0;
    s_result_ready = false;
    s_settled_tick = 0;
    s_xfer_tick = 0;
    s_index = 0;
    s_subIndex = 0;
    s_lastDir = XFER_READ;
}

bool_t
co_master_start_read(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint16_t timeout_ms) {
    if (!xfer_begin(nodeId, XFER_READ)) {
        return false;
    }

    if (CO_SDOclientUploadInitiate(&s_co->SDOclient[0], index, subIndex, timeout_ms, false)
        != CO_SDO_RT_ok_communicationEnd) {
        s_dir = XFER_IDLE;
        s_startErr = CO_MASTER_START_INITIATE_FAIL;
        return false;
    }

    s_index = index;
    s_subIndex = subIndex;
    return true;
}

bool_t
co_master_start_write(uint8_t nodeId, uint16_t index, uint8_t subIndex, const void* data, size_t size,
                      uint16_t timeout_ms) {
    if ((data == NULL) || (size == 0U) || (size > CO_MASTER_MAX_DATA)) {
        s_startErr = CO_MASTER_START_BAD_ARG;
        return false;
    }
    if (!xfer_begin(nodeId, XFER_WRITE)) {
        return false;
    }

    CO_SDOclient_t* sdo = &s_co->SDOclient[0];

    /* sizeIndicated 传真实长度：≤4 字节走快速传输（请求里带上长度和值，
     * 一条报文完事）；>4 字节自动转分段传输（CO_CONFIG_SDO_CLI_SEGMENTED 已开）。 */
    if (CO_SDOclientDownloadInitiate(sdo, index, subIndex, size, timeout_ms, false) != CO_SDO_RT_ok_communicationEnd) {
        s_dir = XFER_IDLE;
        s_startErr = CO_MASTER_START_INITIATE_FAIL;
        return false;
    }

    /* 数据一次全塞进客户端 FIFO。CO_MASTER_MAX_DATA(32) 与客户端 FIFO 同量级，
     * 所以这里必然装得下，Download() 的 bufferPartial 可以恒传 false。 */
    if (CO_SDOclientDownloadBufWrite(sdo, (const uint8_t*)data, size) != size) {
        xfer_finish(CO_MASTER_ERR_ARG, 0);
        return false;
    }

    s_index = index;
    s_subIndex = subIndex;
    return true;
}

/** @brief 推进当前事务，非阻塞。只由 co_master_step_gate 调用 ——
  *        轮询和取结果放在同一处，结果就不会被别人截走。 */
static void
master_poll(void) {
    if ((s_co == NULL) || (s_dir == XFER_IDLE)) {
        return;
    }

    /* 1ms 节流。SDO 超时计时器是按「两次调用的时间差」累加的，若同一毫秒内
     * 反复调用，传入的 timeDifference_us 恒为 0，从站不应答时就永远等不到超时，
     * 事务会卡死。这里保证每次传进去的时间差至少是 1ms。 */
    uint32_t now = HAL_GetTick();
    if (now == s_lastTick) {
        return;
    }
    uint32_t timeDiff_us = (now - s_lastTick) * 1000U;
    s_lastTick = now;

    CO_SDOclient_t* sdo = &s_co->SDOclient[0];
    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    CO_SDO_return_t ret;

    if (s_dir == XFER_WRITE) {
        ret = CO_SDOclientDownload(sdo, timeDiff_us, false, false, &abortCode, NULL, NULL);
    } else {
        ret = CO_SDOclientUpload(sdo, timeDiff_us, false, &abortCode, NULL, NULL, NULL);
    }

    if (ret > 0) {
        if (ret == CO_SDO_RT_uploadDataBufferFull) {
            /* 分段读长对象时 FIFO 满了，必须先取走数据才能继续，否则会一直卡在
             * 这个返回值上。取干净后直接 return，下一轮接着推进。 */
            s_len += CO_SDOclientUploadBufRead(sdo, &s_buf[s_len], sizeof(s_buf) - s_len);
            if (s_len >= sizeof(s_buf)) {
                /* 对象比本模块的缓冲还长，放弃。这不是通信故障，是能力上限。 */
                xfer_finish(CO_MASTER_ERR_ARG, 0);
            }
        }
        return; /* 事务进行中，等下一轮 */
    }

    if (ret < 0) {
        xfer_finish((abortCode == CO_SDO_AB_TIMEOUT) ? CO_MASTER_ERR_TIMEOUT : CO_MASTER_ERR_ABORT, (uint32_t)abortCode);
        return;
    }

    /* ret == 0：通信正常结束。读事务此时才把 FIFO 里剩余的数据取出来。 */
    if (s_dir == XFER_READ) {
        s_len += CO_SDOclientUploadBufRead(sdo, &s_buf[s_len], sizeof(s_buf) - s_len);
    }
    xfer_finish(CO_MASTER_OK, 0);
}

co_master_owner_t
co_master_owner_of(uint8_t nodeId) {
    reap();

    if (s_dir != XFER_IDLE) {
        /* 事务在跑：只有它的归属者该等它，别人看到的是「别人占着」 */
        return (s_owner == nodeId) ? CO_MASTER_MINE : CO_MASTER_OTHER;
    }
    if (s_result_ready) {
        return (s_owner == nodeId) ? CO_MASTER_SETTLED : CO_MASTER_OTHER;
    }
    return CO_MASTER_FREE;
}

bool_t
co_master_take_result(uint8_t nodeId, co_master_status_t* out) {
    if ((s_dir != XFER_IDLE) || !s_result_ready || (s_owner != nodeId)) {
        return false;
    }
    if (out != NULL) {
        out->result = s_result;
        out->abort  = s_abort;
    }
    s_result_ready = false;   /* 读一次就消费 */
    return true;
}

co_sdo_step_t
co_master_step_gate(uint8_t nodeId, uint32_t deadline_ms, co_master_status_t* out) {
    master_poll();

    /* 兜底：我自己的事务超过预算还没结束，强制按超时收尾。
     * SDO 客户端内部有自己的超时，这一道只是保证状态机在任何情况下都能往下走。 */
    if ((s_dir != XFER_IDLE) && (s_owner == nodeId)
        && ((int32_t)(HAL_GetTick() - deadline_ms) >= 0)) {
        xfer_finish(CO_MASTER_ERR_TIMEOUT, (uint32_t)CO_SDO_AB_TIMEOUT);
    }

    switch (co_master_owner_of(nodeId)) {
    case CO_MASTER_FREE:
        return CO_SDO_STEP_NEXT;

    case CO_MASTER_SETTLED: {
        co_master_status_t st = { CO_MASTER_OK, 0 };
        (void)co_master_take_result(nodeId, &st);
        if (out != NULL) {
            *out = st;
        }
        if (st.result != CO_MASTER_OK) {
            /* 事务失败只在这一个地方落日志，而且带上**对象**。
             * 少了这一段，失败会一路静默到「电机不转但串口什么都没说」；
             * 只写节点不写对象，则只能知道「被拒了」，还得再猜一轮是哪个。 */
            printf("[SDO] node=%u %s 0x%04X.%02X 失败: %s abort=0x%08lX\r\n",
                   (unsigned)nodeId,
                   (s_lastDir == XFER_WRITE) ? "写" : "读",
                   (unsigned)s_index, (unsigned)s_subIndex,
                   co_master_result_name(st.result), (unsigned long)st.abort);
            return CO_SDO_STEP_FAIL;
        }
        return CO_SDO_STEP_DONE;
    }

    case CO_MASTER_MINE:
    case CO_MASTER_OTHER:
    default:
        /* 等。注意这里不判 deadline —— 被别人占着不算我超时。 */
        return CO_SDO_STEP_WAIT;
    }
}

co_master_start_err_t
co_master_get_start_error(void) {
    return s_startErr;
}

const char*
co_master_start_error_name(co_master_start_err_t err) {
    switch (err) {
        case CO_MASTER_START_OK: return "OK";
        case CO_MASTER_START_NOT_INIT: return "协议栈未初始化";
        case CO_MASTER_START_BUSY: return "上一次事务未结束";
        case CO_MASTER_START_BAD_NODE: return "节点号非法（0/超范围/等于本机）";
        case CO_MASTER_START_BAD_ARG: return "数据为空或超长";
        case CO_MASTER_START_SETUP_FAIL: return "SDO 客户端配置失败";
        case CO_MASTER_START_INITIATE_FAIL: return "SDO 客户端拒绝发起";
        default: return "?";
    }
}

const char*
co_master_result_name(co_master_result_t result) {
    switch (result) {
        case CO_MASTER_OK:          return "成功";
        case CO_MASTER_ERR_ARG:     return "参数非法/客户端不可用";
        case CO_MASTER_ERR_ABORT:   return "从站拒绝（SDO abort）";
        case CO_MASTER_ERR_TIMEOUT: return "从站不应答（超时）";
        default:                    return "?";
    }
}

size_t
co_master_get_size(void) {
    return s_len;
}

bool_t
co_master_get_u32(uint32_t* out) {
    if ((out == NULL) || (s_len < 4U)) {
        return false;
    }
    /* SDO 线上是小端，CANopenNode 把收到的字节按原序放进缓冲区，
     * 所以这里直接按小端拼。 */
    *out = (uint32_t)s_buf[0] | ((uint32_t)s_buf[1] << 8) | ((uint32_t)s_buf[2] << 16) | ((uint32_t)s_buf[3] << 24);
    return true;
}

bool_t
co_master_get_u16(uint16_t* out) {
    if ((out == NULL) || (s_len < 2U)) {
        return false;
    }
    *out = (uint16_t)((uint16_t)s_buf[0] | ((uint16_t)s_buf[1] << 8));
    return true;
}

bool_t
co_master_get_u8(uint8_t* out) {
    if ((out == NULL) || (s_len < 1U)) {
        return false;
    }
    *out = s_buf[0];
    return true;
}

bool_t
co_master_nmt_send(CO_NMT_command_t command, uint8_t nodeId) {
    if (s_co == NULL) {
        return false;
    }
    return (CO_NMT_sendCommand(s_co->NMT, command, nodeId) == CO_ERROR_NO) ? true : false;
}
