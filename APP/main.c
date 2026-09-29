/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "canopen_app.h"
#include "canopen_master.h"
#include "cia402.h"
#include "canopen_init.h"
#include "OD.h"
#include "user_pdo_cfg_slave.h"
#include "diag_log.h"
#include "ringbuf.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 注意命名：移植层的全局是 CANopenNodeSTM32* canopenNodeSTM32（指针），
 * 此处刻意用大写 O 的 canOpenNodeSTM32（结构体本体）以区分，避免符号冲突。
 * 与上游官方范例的命名保持一致。 */
CANopenNodeSTM32 canOpenNodeSTM32;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* 生产者在 CAN1_RX0 中断里（CO_CANrxCaptureHook），消费者是本文件的
 * while 主循环 —— 单生产者单消费者，ringbuf 内部无锁即可安全并发。 */
static ringbuffer_t      *g_canLog      = NULL;
static volatile uint32_t  g_canLogDrops = 0;  /* 装不下而整条丢弃的帧数 */
static uint32_t           g_dumpTick    = 0;  /* 上次输出时刻 */

/* ── CAN 帧日志 ─────────────────────────────────────────────────────────
 * 由 CO_driver_STM32.c 的 CO_CANrxCaptureHook 驱动。
 * 开关在 framework/canopen/port/CO_driver_STM32.c 顶部：
 *     #define CO_CAN_RX_CAPTURE_HOOK 1
 * 关掉时驱动侧不再回调，本段代码成为无人引用的死代码，
 * 会被链接器的 --gc-sections 裁掉。 */

/* 一条 CAN 帧的日志记录：ident + dlc + data。
 * 定长 11 字节 —— 保证它总被【整条】写入或【整条】丢弃，不会写一半，
 * 否则消费端按记录长度解析就会错位。 */
typedef struct {
    uint16_t ident;   /* 11 位标准 ID */
    uint8_t  dlc;     /* 0..8 */
    uint8_t  data[8]; /* 未用到的字节补 0 */
} can_frame_t;

static void can_log_dump(void)
{
    if (g_canLog == NULL) {
        return;
    }

    const size_t   pending = ringbuf_used(g_canLog) / sizeof(can_frame_t);
    const uint32_t drops   = g_canLogDrops;
    g_canLogDrops = 0;   /* 每轮清零，"丢帧数"指本轮周期内的 */

    if ((pending == 0u) && (drops == 0u)) {
        return;   /* 这一秒一帧都没收到，不输出，避免刷屏 */
    }

    printf("---- CAN log: %u frame(s)", (unsigned)pending);
    if (drops != 0u) {
        printf(", %u dropped (buffer full)", (unsigned)drops);
    }
    printf(" ----\r\n");

    for (size_t i = 0; i < pending; i++) {
        can_frame_t f;
        if (ringbuf_read(g_canLog, &f, sizeof(f)) != sizeof(f)) {
            break;
        }
        printf("  ID=0x%03X  DLC=%u  ", (unsigned)f.ident, (unsigned)f.dlc);
        for (uint8_t k = 0u; k < f.dlc; k++) {
            printf("%02X ", (unsigned)f.data[k]);
        }
        printf("\r\n");
    }
}

/**
  * @brief  接收帧旁路钩子 —— 覆盖 CO_driver_STM32.c 里的 __weak 默认实现
  * @note   在 CAN1_RX0 中断上下文被调用，只做「整条塞进缓冲区」这一件事，
  *         不做任何阻塞或耗时操作。
  */
void CO_CANrxCaptureHook(uint32_t ident, uint8_t dlc, const uint8_t *data)
{
  if ((g_canLog == NULL) || (data == NULL) || (dlc > 8u)) {
    return;
  }

  /* 空间不够容纳【整条】记录就整条丢弃，绝不写半条 —— 否则消费端会解析错位。
   * 单生产者（只在本中断里写），检查与写入之间不会被其它生产者插入；
   * 消费者只会让空间变大，所以这个检查是安全的。 */
  if (ringbuf_space(g_canLog) < sizeof(can_frame_t)) {
    g_canLogDrops++;
    return;
  }

  can_frame_t f = {0};
  f.ident = (uint16_t)(ident & 0x7FFu);
  f.dlc   = dlc;
  memcpy(f.data, data, dlc);

  (void)ringbuf_write(g_canLog, &f, sizeof(f));
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* RAM 日志缓冲必须先清、必须在任何 printf 之前。
   * 它替串口兜底：板上没接 USB-TTL 时，靠调试器读这块内存取日志
   * （framework/BSP/diag_log.h，读法见 tools/dump_ramlog.gdb）。 */
  diag_log_init();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_CAN1_Init();
  HAL_CAN_Start(&hcan1);

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  printf("Hello world :CANopenNode STM32F4xx Demo!\r\n");
  
  /* USER CODE BEGIN 2 */
  /* CAN 帧日志缓冲区。必须先于 canopen_app_init() 建好：
   * 接收是在 canopen_app_init() 内部的 CO_CANmodule_init() 配好滤波器
   * 之后才真正打开的，此后第一帧就可能触发钩子。
   * 512 字节 / 11 字节每条 ≈ 46 条记录。 */
  if (!ringbuf_init(&g_canLog)) {
    printf("CAN log: ringbuf_init failed\r\n");
  }

  /* CANopen 主站初始化。
   * 只需填这四项，其余（滤波器、HAL_CAN_Start、RX 中断通知）由 CANopen
   * 驱动在 canopen_app_init() 内部完成，这里不要重复调用。
   * 1ms 时基由 SysTick 提供，见 stm32f4xx_it.c 的 SysTick_Handler。 */
  canOpenNodeSTM32.CANHandle      = &hcan1;
  canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
  canOpenNodeSTM32.desiredNodeID  = 1;    /* 本节点占用的 CANopen 节点号 */
  canOpenNodeSTM32.baudrate       = 500;  /* kbit/s，需与 can.c 一致 */
  canopen_app_init(&canOpenNodeSTM32);

  /* 主站侧 SDO 事务层。第二个参数是本机节点号 —— 用来拦住「SDO 目标写成
   * 自己」这种配置错误（本工程没开 CAN 回环，自己发给自己收不到应答）。
   * 必须等 canopen_app_init() 成功、canOpenStack 非空之后再调。 */
  co_master_init(canOpenNodeSTM32.canOpenStack, canOpenNodeSTM32.activeNodeID);

  /* 通用 CANopen 初始化：等 PRE-OP → 配从站 PDO → 进 OP。
   * 完成后 cia402 接管完成模式配置与 402 状态转换。 */
  co_init_start(canOpenNodeSTM32.canOpenStack, SLAVE1_NODE_ID, &slave1_pdo_config);
  printf("CANopen init started: slave=%u, %u kbit/s\r\n",
         (unsigned)SLAVE1_NODE_ID, (unsigned)canOpenNodeSTM32.baudrate);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 调试：每 500ms 打印一次循环心跳，确认 while 在跑 */
    {
      static uint32_t s_loopTick = 0;
      uint32_t now = HAL_GetTick();
      if (now - s_loopTick >= 500u) {
        s_loopTick = now;
        printf("[LOOP] tick=%lu co_state=%d co_fault=0x%04X nmt=%d errReg=0x%02X\r\n", now,
               (int)co_init_get_state(),
               (unsigned)co_init_get_fault_code(),
               (int)canOpenNodeSTM32.canOpenStack->NMT->operatingState,
               OD_RAM.x1001_errorRegister);
      }
    }

    /* ── 阶段1：推进通用 CANopen 初始化 ── */
    canopen_app_process();
    co_init_process();

    /* ── 阶段2：通用初始化完成后启动并推进 402 ── */
    static bool_t g_cia402_started = false;
    if (co_init_done() && !g_cia402_started) {
      cia402_init(canOpenNodeSTM32.canOpenStack, SLAVE1_NODE_ID, CIA402_MODE_PROFILE_VEL);
      g_cia402_started = true;
      printf("CiA402 init started: mode=velocity\r\n");
    }
    if (g_cia402_started) {
      cia402_process();
    }

    uint32_t nowTick = HAL_GetTick();

    /* ── 10s 测试：手动触发 TPDO 发送，打印 RPDO 收到的反馈 ── */
    if (g_cia402_started && cia402_get_state() == CIA402_RUNNING) {
      static uint32_t g_testTick = 0;
      if ((nowTick - g_testTick) >= 10000u) {
        g_testTick = nowTick;
        /* 设置控制字(保持OP)和目标速度，然后触发TPDO发送 */
        OD_RAM.x2000_controlword = 0x000F;
        OD_RAM.x2002_target_velocity = 1000;
        CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[0]);
        printf("[10s TPDO] cw=0x%04X vel=%ld | sts=0x%04X actVel=%ld\r\n",
               OD_RAM.x2000_controlword, (int32_t)OD_RAM.x2002_target_velocity,
               OD_RAM.x2003_statusword,  (int32_t)OD_RAM.x2005_actual_velocity);
      }
    }

    /* ── 每秒 CAN 帧日志 ── */
    if ((nowTick - g_dumpTick) >= 1000u) {
      g_dumpTick = nowTick;
      can_log_dump();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */