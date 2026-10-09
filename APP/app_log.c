/**
  * @file    app_log.c
  * @brief   串口日志开关的运行期挡位（见 app_log.h）
  */
#include "app_log.h"

volatile uint8_t g_app_log_level = APP_LOG_COMPILE_LEVEL;

void app_log_set_level(uint8_t level)
{
    g_app_log_level = (level > APP_LOG_COMPILE_LEVEL) ? APP_LOG_COMPILE_LEVEL : level;
}
