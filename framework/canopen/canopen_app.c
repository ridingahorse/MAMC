/**
  * @file    canopen_app.c
  * @brief   CANopen 协议栈的启动与运行实现
  */
#include "canopen_app.h"
#include "CANopen.h"
#include "OD.h"
#include "main.h"
#include <stdio.h>

/* ── 私有状态 ─────────────────────────────────────────────────── */

static CO_t* CO = NULL;                  /* 协议栈对象，CO_new() 建出 */
static CANopenNodeSTM32* node = NULL;    /* 应用层持有的节点对象 */
static uint32_t time_old = 0;            /* 上次喂 CO_process() 的时刻 */

/**
  * @brief  协议栈是否处于可访问状态
  * @note   上游用 HAL_TIM_Base_Stop_IT() 停掉 1ms 中断来避开
  *         CO_RESET_COMM 期间「CO 已释放、尚未重建」的窗口。
  *         本工程 1ms 来自 SysTick，停不掉，改用这个标志做等价互斥：
  *         清 CO 之前置 false，重建完成后置 true。
  *         volatile：主循环与 SysTick 中断都会访问。
  */
static volatile bool_t running = false;

/* ── 初始化参数（沿用上游取值）──────────────────────────────────── */

/* 启动后进入 Operational。去掉 CO_NMT_ERR_ON_ERR_REG：
 * 主站错误寄存器可能因上电瞬时通信错误置位，不应因此阻止进 OP。 */
#define NMT_CONTROL                                                                                                    \
    (CO_NMT_STARTUP_TO_OPERATIONAL | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_TIME        500   /* 首次心跳延时 ms */
#define SDO_SRV_TIMEOUT_TIME 1000  /* SDO 服务器超时 ms（本节点作服务器时） */
#define SDO_CLI_TIMEOUT_TIME 500   /* SDO 客户端超时 ms（主站读写从站时） */
#define SDO_CLI_BLOCK        false /* 不用块传输，用分段传输 */
#define OD_STATUS_BITS       NULL

/* ── 实现 ─────────────────────────────────────────────────────── */

int
canopen_app_init(CANopenNodeSTM32* n) {
    node = n;

    /* 建协议栈对象。config 传 NULL 表示用 OD.h 里编译期定好的默认配置
     * （各对象实例数量由 OD_CNT_* 宏决定，SDO 客户端数量即 OD_CNT_SDO_CLI）。 */
    uint32_t heapUsed = 0;
    CO = CO_new(NULL, &heapUsed);
    if (CO == NULL) {
        printf("CANopen: CO_new failed\n");
        return 1;
    }
    printf("CANopen: allocated %u bytes for stack objects\n", (unsigned)heapUsed);
    node->canOpenStack = CO;

    return canopen_app_resetCommunication();
}

int
canopen_app_resetCommunication(void) {
    if (CO == NULL) {
        return 1;
    }

    /* 重建期间禁止 1ms 中断访问协议栈 */
    running = false;

    CO->CANmodule->CANnormal = false;
    CO_CANsetConfigurationMode((void*)node);   /* CAN 转入配置模式 */
    CO_CANmodule_disable(CO->CANmodule);

    /* 驱动会在这里回调 HWInitFunction()（即 MX_CAN1_Init）重新初始化外设，
     * 然后配置全通过滤器、启动 CAN、打开 RX 中断通知。bitrate 参数不使用 ——
     * 真实位时序由 can.c 的 MX_CAN1_Init 配置。 */
    CO_ReturnError_t err = CO_CANinit(CO, node, 0);
    if (err != CO_ERROR_NO) {
        printf("CANopen: CO_CANinit failed: %d\n", (int)err);
        return 2;
    }

    /* LSS 地址取自对象字典 0x1018（厂商 ID / 产品码 / 版本 / 序列号）。
     * LSS 从站能力保留：允许网络上的 LSS 主站为本节点分配节点号。 */
    CO_LSS_address_t lssAddress = {
        .identity = {.vendorID       = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                     .productCode    = OD_PERSIST_COMM.x1018_identity.productCode,
                     .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                     .serialNumber   = OD_PERSIST_COMM.x1018_identity.serialNumber}};
    err = CO_LSSinit(CO, &lssAddress, &node->desiredNodeID, &node->baudrate);
    if (err != CO_ERROR_NO) {
        printf("CANopen: CO_LSSinit failed: %d\n", (int)err);
        return 3;
    }
    node->activeNodeID = node->desiredNodeID;

    /* 初始化各协议模块：NMT / EMCY / SDO 服务器 / SDO 客户端 / 心跳 / PDO。
     * 具体编译进哪些模块由 CO_driver_custom.h 的配置开关决定。
     * 前两个参数是「替代 NMT / 替代 EMCY」，传 NULL 表示用协议栈内部创建的。 */
    uint32_t errInfo = 0;
    err = CO_CANopenInit(CO, NULL, NULL, OD, OD_STATUS_BITS, NMT_CONTROL, FIRST_HB_TIME, SDO_SRV_TIMEOUT_TIME,
                         SDO_CLI_TIMEOUT_TIME, SDO_CLI_BLOCK, node->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO) {
        printf("CANopen: CO_CANopenInit failed: %d (OD entry 0x%08X)\n", (int)err, (unsigned)errInfo);
        return 4;
    }

    err = CO_CANopenInitPDO(CO, CO->em, OD, node->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO) {
        printf("CANopen: CO_CANopenInitPDO failed: %d (OD entry 0x%08X)\n", (int)err, (unsigned)errInfo);
        return 5;
    }

    /* 到这里所有模块都已完成接收缓冲登记，可以放行中断了 */
    time_old = HAL_GetTick();
    running = true;

    /* CAN 真正上线（内部即 HAL_CAN_Start） */
    CO_CANsetNormalMode(CO->CANmodule);

    printf("CANopen: node %u up\n", (unsigned)node->activeNodeID);
    return 0;
}

void
canopen_app_process(void) {
    if (CO == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - time_old) == 0U) {
        return; /* 距上次不足 1ms，不重复处理 */
    }
    uint32_t timeDiff_us = (now - time_old) * 1000U;
    time_old = now;

    CO_NMT_reset_cmd_t reset_status = CO_process(CO, false, timeDiff_us, NULL);

    if (reset_status == CO_RESET_COMM) {
        /* 收到 NMT reset communication：销毁重建 */
        running = false;
        CO_CANsetConfigurationMode((void*)node);
        CO_delete(CO);
        CO = NULL;
        if (canopen_app_init(node) != 0) {
            printf("CANopen: re-init failed, node dead\n");
        }
    } else if (reset_status == CO_RESET_APP) {
        /* NMT reset application：整机复位 */
        HAL_NVIC_SystemReset();
    }
}

void
canopen_app_interrupt(void) {
    /* 守卫必须在 CO_LOCK_OD 之前：CO_LOCK_OD 内部是关中断，
     * 若在加锁后提前 return，中断会被永久关闭。 */
    if (!running || CO == NULL) {
        return;
    }

    CO_LOCK_OD(CO->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
        bool_t syncWas = false;
        uint32_t timeDiff_us = 1000U; /* 本函数由 1ms 时基调用 */

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, timeDiff_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(CO, syncWas, timeDiff_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(CO, syncWas, timeDiff_us, NULL);
#endif
    }
    CO_UNLOCK_OD(CO->CANmodule);
}