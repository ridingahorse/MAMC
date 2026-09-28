/**
  * @file    CO_driver_custom.h
  * @brief   CANopenNode 编译期功能裁剪（本项目 = 主站）
  *
  * 这是上游官方预留的配置覆盖钩子：CO_driver_target.h:51 在
  * `#ifdef CO_DRIVER_CUSTOM` 下 include 本文件。而各模块头文件
  * （CO_NMT_Heartbeat.h:29、CO_SDOclient.h:31、CO_HBconsumer.h:30）
  * 的默认值都写在 `#ifndef` 保护里，include 顺序是：
  *
  *   CANopen.h → 301/CO_driver.h → CO_config.h（只定义“位”宏）
  *                              → CO_driver_target.h → 本文件   ← 覆盖在这里生效
  *   之后才是 CO_NMT_Heartbeat.h / CO_SDOclient.h / ... 的 #ifndef 默认值
  *
  * 所以在这个文件里 #undef + #define 就能覆盖，**不需要改任何上游文件**。
  *
  * 生效前提：编译时定义 CO_DRIVER_CUSTOM（见 Makefile 的 C_DEFS）。
  */
#ifndef CO_DRIVER_CUSTOM_H
#define CO_DRIVER_CUSTOM_H

/* ─────────────────────────────────────────────────────────────
 * 本节点角色：CANopen 主站
 * ───────────────────────────────────────────────────────────── */

/* NMT：默认只有 CALLBACK_PRE | TIMERNEXT，那是【纯从站】的行为。
 * 加上 CO_CONFIG_NMT_MASTER 后才会编译出主站发送接口
 * CO_NMT_sendCommand()（CO_NMT_Heartbeat.h:268-285 内受该宏保护）。 */
#undef  CO_CONFIG_NMT
#define CO_CONFIG_NMT                                                                                                  \
    (CO_CONFIG_NMT_MASTER | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)

/* SDO 客户端：默认是 0（完全关闭）。主站必须打开才能读写从站的对象字典。
 * SEGMENTED 用于长度超过 4 字节的对象；不加则只能做快速传输（≤4 字节）。 */
#undef  CO_CONFIG_SDO_CLI
#define CO_CONFIG_SDO_CLI (CO_CONFIG_SDO_CLI_ENABLE | CO_CONFIG_SDO_CLI_SEGMENTED)

/* FIFO：SDO 客户端的分段传输依赖它 —— CO_SDOclient.c:31 有 #error 强制检查。
 * 不用块传输（CO_CONFIG_SDO_CLI_BLOCK=false），所以只需要 ENABLE，
 * 不需要 ALT_READ 与 CRC16_CCITT（那两个只在启用块传输时才被要求）。 */
#undef  CO_CONFIG_FIFO
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE)

/* 心跳消费：默认已含 ENABLE，但只有 CALLBACK_PRE（一个公共回调）。
 * 主站要按轴监测在线状态、分别处理超时，所以补上 CALLBACK_MULTI。
 * 注意 MULTI 与 CALLBACK_CHANGE 互斥（CO_config.h:168-170 有明确警告）。 */
#undef  CO_CONFIG_HB_CONS
#define CO_CONFIG_HB_CONS                                                                                              \
    (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_HB_CONS_CALLBACK_MULTI | CO_CONFIG_HB_CONS_QUERY_FUNCT \
      | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC \
     | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)

#endif /* CO_DRIVER_CUSTOM_H */