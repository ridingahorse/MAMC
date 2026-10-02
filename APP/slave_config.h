/**
  * @file    slave_config.h
  * @brief   从站完整配置（硬件参数 + 模式参数 + 轨迹参数）
  *
  * 业务层只需 include 本文件即可获得从站全部配置，不依赖 test.h。
  * PDO 通道描述（COB-ID、映射条目等）由 user_pdo_cfg_slave.h 提供，
  * 本文件只做聚合。
  */
 #ifndef SLAVE_CONFIG_H
 #define SLAVE_CONFIG_H
 
 #include "cia402.h"
 #include "user_pdo_cfg_slave.h"   /* SLAVE1_NODE_ID, slave1_vel_config, slave1_pos_config */
 
 /* ── 从站硬件参数 ──────────────────────────────────────────────── */
 #define SLAVE1_PULSE_PER_REV    50000U   /* DM556-CAN 0x2001 细分数（脉冲/圈） */
 
 /* ── 速度模式参数（加速度/减速度，0=不下发） ───────────────────── */
 static cia402_mode_params_t slave1_vel_params = {
     .acceleration     = 0,
     .deceleration     = 0,
     .profile_velocity = 0,   /* 速度模式不用 0x6081 */
     .max_profile_vel  = 0,
 };
 
 /* ── 位置模式参数（轮廓速度=100rpm 的 pps，加速度/减速度） ──────── */
 static cia402_mode_params_t slave1_pos_params = {
     .profile_velocity = (uint32_t)((uint64_t)100 * SLAVE1_PULSE_PER_REV / 60),
     .acceleration     = 0,
     .deceleration     = 0,
     .max_profile_vel  = 0,
 };
 
 /* ── 从站完整配置（硬件 + 模式 + 轨迹，一次性喂给 motor_ctrl_set_config） ── */
 static const cia402_slave_config_t slave1_config = {
     .node_id        = SLAVE1_NODE_ID,
     .pulse_per_rev  = SLAVE1_PULSE_PER_REV,
     .vel_cfg        = &slave1_vel_config,
     .pos_cfg        = &slave1_pos_config,
     .vel_params     = &slave1_vel_params,
     .pos_params     = &slave1_pos_params,
     /* traj 只在 CSP 模式下被使用（那时轨迹由主机生成）。
      * PP / PV 模式下驱动器用自己的 0x6081/0x6083/0x6084 规划，下面这组参数
      * 配了也不会生效 —— 所以现在跑 PP 时这些值只是给 CSP 备用。 */
     .traj = {
         .curve_type = CIA402_TRAJ_S_CURVE,
         .v_max      = (uint32_t)((uint64_t)100  * SLAVE1_PULSE_PER_REV / 60), /* 100rpm → pps */
         .a_max      = (uint32_t)((uint64_t)500  * SLAVE1_PULSE_PER_REV / 60), /* 500rpm/s → pps² */
         .jerk       = (uint32_t)((uint64_t)5000 * SLAVE1_PULSE_PER_REV / 60), /* 5000rpm/s² → pps³ */
         .dt_ms      = 10,
     },
 };
 
 #endif /* SLAVE_CONFIG_H */
 