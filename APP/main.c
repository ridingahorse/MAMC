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
#include "motor_ctrl.h"
#include "slave_config.h"
#include "OD.h"
#include "user_pdo_cfg_slave.h"
#include "diag_log.h"
#include "app_log.h"
#include "test.h"

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


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
  {

  /* USER CODE BEGIN 1 */

  /* RAM 日志缓冲必须先清、必须在任何打印之前。
   * 它替串口兜底：板上没接 USB-TTL 时，靠调试器读这块内存取日志
   * （framework/BSP/diag_log.h，读法见 tools/dump_ramlog.gdb）。
   * 注意串口本身的总开关在 APP/app_log.h，与这里无关。 */
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

  LOGI("Hello world :CANopenNode STM32F4xx Demo!\r\n");
  
  /* USER CODE BEGIN 2 */
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

  /* 一站式配置：启动 CANopen 初始化，402 状态机由 motor_ctrl_process 在合适时机自动启动 */
  motor_ctrl_set_config(0, canOpenNodeSTM32.canOpenStack, &slave1_config, CIA402_MODE_PROFILE_VEL);
  motor_ctrl_set_config(1, canOpenNodeSTM32.canOpenStack, &slave2_config, CIA402_MODE_PROFILE_VEL);
  LOGI("motor_ctrl config set: slave1=node%u, slave2=node%u, %u kbit/s\r\n",
         (unsigned)SLAVE1_NODE_ID, (unsigned)SLAVE2_NODE_ID, (unsigned)canOpenNodeSTM32.baudrate);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ── 推进电机控制状态机（内部协调 CANopen 初始化 + 402） ── */
    canopen_app_process();
    motor_ctrl_process();

    /* ── 模式切换测试 ── */
    uint32_t nowTick = HAL_GetTick();
    test_mode_switch(nowTick);
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