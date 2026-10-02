/**
  * @file    cia402.h
  * @brief   CiA 402 专属状态机 —— 从 OP 到 Operation enabled
  *
  * 前置条件：通用 CANopen 初始化（等PRE-OP、配心跳、配PDO、发NMT进OP）
  *           已由 canopen_init 模块完成，从站应进入或即将进入 OP。
  *
  * 流程：等OP → 设模式 → shutdown → switch on → enable op → RUNNING
  *
  * 未来扩展 cia502 / cia602 时，照此模板新建文件即可，不影响本文件。
  */
#ifndef CIA402_H
#define CIA402_H

#include "CANopen.h"
#include "canopen_init.h"
#include "../trajectory/s_curve.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════
 *  主状态枚举
 * ═══════════════════════════════════════════════════════════════ */

typedef enum {
    CIA402_WAIT_OP     = 0,  /**< 等从站心跳确认进入 OP */
    CIA402_SET_MODE,         /**< 写 0x6060 设模式，读 0x6061 确认 */
    CIA402_SHUTDOWN,         /**< 写控制字 0x0006，等 Ready to switch on */
    CIA402_SWITCH_ON,        /**< 写控制字 0x0007，等 Switched on */
    CIA402_ENABLE_OP,        /**< 写控制字 0x000F，等 Operation enabled */
    CIA402_CFG_PARAMS,       /**< 下发该模式参数（加速度/减速度/轮廓速度等） */
    CIA402_RUNNING,          /**< 初始化完成 */
    CIA402_DISABLE_OP,       /**< 停机（切换模式前，写 0x0007） */
    CIA402_WAIT_RECONFIG,    /**< 等 canopen_init 重配 PDO 完成 */
    CIA402_FAULT,            /**< 故障 */
} cia402_state_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  超时与重试
  * ═══════════════════════════════════════════════════════════════ */
 
#define CIA402_SDO_TIMEOUT_MS      500U
#define CIA402_OP_TIMEOUT_MS      3000U
#define CIA402_STEP_TIMEOUT_MS    1000U
 
 /* ═══════════════════════════════════════════════════════════════
  *  CiA 402 控制字 / 状态字
  * ═══════════════════════════════════════════════════════════════ */
 
 #define CIA402_CMD_SHUTDOWN      0x0006U  /**< Switch on disabled → Ready to switch on（上电初始化用） */
 #define CIA402_CMD_SWITCH_ON     0x0007U  /**< Ready to switch on → Switched on */
 #define CIA402_CMD_ENABLE_OP     0x000FU  /**< Switched on → Operation enabled（电机运转） */
 #define CIA402_CMD_DISABLE_OP    0x0007U  /**< Operation enabled → Switched on（清bit3，停机但保持励磁） */
 
 #define CIA402_STS_MASK            0x006FU  /**< 屏蔽厂家位与警告位 */
 #define CIA402_STS_READY_SWITCH    0x0021U  /**< Ready to switch on */
 #define CIA402_STS_SWITCHED_ON     0x0023U  /**< Switched on */
 #define CIA402_STS_OP_ENABLED      0x0027U  /**< Operation enabled */
 #define CIA402_STS_TARGET_REACHED  0x0400U  /**< bit10：到位（位置模式）/ 到达转速（速度模式） */

 /* 位置模式控制字：0x0F 保持 OP + bit4 新目标点 + bit5 立即改变 + bit6=0 绝对位置。
  * 注意 bit4 是【上升沿】触发 —— 每下发一个新设定点都要重新给一次沿，
  * 所以它只适合"一条指令一个目标"的直发，不能拿来做逐点流式下发。 */
 #define CIA402_CMD_POS_ABS        0x003FU
 
 /* ═══════════════════════════════════════════════════════════════
  *  运行模式
  * ═══════════════════════════════════════════════════════════════ */
 
/* 运行模式决定了【谁来生成轨迹】，这是选模式时最该知道的事：
 *
 *   PROFILE_POS(1) / PROFILE_VEL(3)  轨迹归【驱动器】：
 *        主机只写一次最终目标（0x607A / 0x60FF），驱动器按自己的
 *        0x6081/0x6083/0x6084 规划。主机插值没有意义 —— 驱动器不接收
 *        中间点（PP 的新设定点靠控制字 bit4 上升沿触发），硬发只会被丢弃。
 *
 *   CSP(8)                           轨迹归【主机】：
 *        主机每周期下发一个中间点（0x607A），驱动器只负责跟随。
 *        软件 S 曲线只在这种模式下使用。
 */
 typedef enum {
     CIA402_MODE_PROFILE_POS = 1,   /**< 轮廓位置：驱动器规划，主机只发最终目标 */
     CIA402_MODE_PROFILE_VEL = 3,   /**< 轮廓速度：驱动器规划 */
     CIA402_MODE_TORQUE      = 4,   /**< 转矩 */
     CIA402_MODE_HOMING      = 6,   /**< 回零 */
     CIA402_MODE_CSP         = 8,   /**< 循环同步位置：主机逐点下发，软件轨迹用这个 */
 } cia402_mode_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  故障码
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef enum {
     CIA402_ERR_NONE          = 0x0000,
     CIA402_ERR_OP_TIMEOUT    = 0x0100,  /**< 等从站进 OP 超时 */
     CIA402_ERR_MODE_TIMEOUT  = 0x0200,  /**< 设模式 SDO 超时 */
     CIA402_ERR_MODE_SDO      = 0x0201,  /**< 设模式 SDO 失败 */
     CIA402_ERR_MODE_MISMATCH = 0x0202,  /**< 读回模式与目标不一致 */
     CIA402_ERR_TRANS_TIMEOUT = 0x0300,  /**< 402 状态转换超时 */
     CIA402_ERR_TRANS_SDO     = 0x0301,  /**< 402 转换写控制字/读状态字 SDO 失败 */
 } cia402_err_t;
 
 /* PDO 配置描述定义在 canopen_init.h，通过 cia402_slave_config_t 持有指针 */

/* ═══════════════════════════════════════════════════════════════
 *  轨迹规划配置
 * ═══════════════════════════════════════════════════════════════ */

/* 软件轨迹的算法选择。【只在 CSP 模式下生效】——
 * 这一个结构描述的是"主机拿到轨迹生成权之后怎么画曲线"，
 * 不参与"要不要主机生成轨迹"的判断（那个由 target_mode 决定）。
 * PP / PV 模式下本结构整体被忽略，配了也不会被调用。 */
typedef enum {
    CIA402_TRAJ_NONE    = 0,   /**< 不做软件曲线，目标直接下发 */
    CIA402_TRAJ_T_CURVE = 1,   /**< 梯形曲线（未实现，等同 NONE） */
    CIA402_TRAJ_S_CURVE = 2,   /**< 7段S曲线（软件插值） */
} cia402_traj_type_t;

typedef struct {
    cia402_traj_type_t curve_type;  /**< 曲线类型（仅 CSP 模式下有意义） */
    uint32_t v_max;                 /**< 最大速度（pps） */
    uint32_t a_max;                 /**< 最大加速度（pps²） */
    uint32_t jerk;                  /**< 加加速度（pps³） */
    uint32_t dt_ms;                 /**< 规划周期（ms），应接近 cia402_process 调用周期 */
} cia402_traj_config_t;


/* 模式参数（每种模式一份，上层在 switch_mode 前填好） */

typedef struct {
  uint32_t profile_velocity;  /**< 轮廓速度 0x6081，pps（位置模式用，0=不写） */
  uint32_t acceleration;      /**< 加速度   0x6083，user units/s²（0=不写） */
  uint32_t deceleration;      /**< 减速度   0x6084，user units/s²（0=不写） */
  uint32_t max_profile_vel;   /**< 最大轮廓速度 0x6082，pps（0=不写） */
} cia402_mode_params_t;

/** 从站静态配置：上层定义一次，传给 cia402_set_config */
typedef struct {
    uint8_t  node_id;            /**< 从站节点号 */
    uint32_t pulse_per_rev;      /**< 每圈脉冲数（rpm↔pps 换算用） */
    const co_pdo_config_t* vel_cfg;  /**< 速度模式 PDO 配置 */
    const co_pdo_config_t* pos_cfg;  /**< 位置模式 PDO 配置 */
    cia402_mode_params_t* vel_params;  /**< 速度模式参数（NULL=不配置） */
    cia402_mode_params_t* pos_params;  /**< 位置模式参数（NULL=不配置） */
    cia402_traj_config_t traj;   /**< 轨迹规划配置（位置模式用） */
} cia402_slave_config_t;

 /* ═══════════════════════════════════════════════════════════════
  *  从站实例（运行时全部状态）
  * ═══════════════════════════════════════════════════════════════ */
 
 typedef struct {
   uint8_t  node_id;           /**< 从站节点号 */
   uint8_t  hb_idx;            /**< 心跳消费者 0x1016 表中的索引 */

   cia402_state_t state;       /**< 当前主状态 */
   cia402_state_t prev_state;  /**< 上一次状态，用于触发 enter */
   uint8_t  trans_step;        /**< 402 状态转换子步骤（0=写控制字, 1=读状态字） */

   uint32_t deadline;          /**< 当前阶段超时截止时刻（ms） */
   uint8_t  retry;             /**< 重试次数 */

   bool_t   fault;             /**< 是否故障 */
   uint16_t fault_code;        /**< 故障码 */

   cia402_mode_t target_mode;  /**< 期望运行模式 */

   /* 模式切换专用 */
   cia402_mode_t pending_mode;       /**< 待切换的目标模式 */
   const co_pdo_config_t* pending_pdo_cfg;  /**< 待切换的 PDO 配置 */

   /* 参数配置子状态 */
   uint8_t  param_step;        /**< 参数配置当前步骤 */
   const cia402_mode_params_t* active_params;  /**< 当前模式参数指针 */

   /* 轨迹规划运行时状态（位置模式 + S 曲线时使用） */
   s_curve_t scurve;            /**< S 曲线规划器实例 */
   bool_t    scurve_active;     /**< 是否正在规划中 */
   uint32_t  scurve_last_tick;  /**< 上次轨迹步进时刻（ms） */
} cia402_slave_t;
 
 /* ═══════════════════════════════════════════════════════════════
  *  对外接口
  * ═══════════════════════════════════════════════════════════════ */
 
 /**
   * @brief  初始化，绑定协议栈与从站配置。只做数据结构初始化，不发起通信。
   *         前置条件：canopen_init 已完成（从站已进 OP 或即将进 OP），
   *         心跳消费者已注册。
   * @param  co       协议栈实例
   * @param  cfg      从站配置（node_id、pulse_per_rev、PDO/参数指针）
   * @param  mode     初始运行模式
   */
 void cia402_init(CO_t* co, const cia402_slave_config_t* cfg, cia402_mode_t mode);
 
 /**
   * @brief  状态机推进，主循环每周期调用，全程非阻塞
   */
 void cia402_process(void);
 
 /**
   * @brief  查询当前主状态
   */
 cia402_state_t cia402_get_state(void);
 
 /**
  * @brief  获取故障码
  */
uint16_t cia402_get_fault_code(void);

/**
  * @brief  请求切换运行模式（速度 ↔ 位置）。
  *         非阻塞，调用后状态机自动完成：停机 → 重配 PDO → 设新模式 → 下发参数 → 重新使能。
  *         只能在 CIA402_RUNNING 状态下调用。
  *         模式参数从 cia402_init 绑定的 cfg 中取（vel_params / pos_params），
  *         上层在调用前可直接修改 params 结构体内容。
  * @param  mode  目标模式（CIA402_MODE_PROFILE_VEL / CIA402_MODE_PROFILE_POS）
  * @return true=请求已接受, false=当前状态不允许切换
  */
bool_t cia402_switch_mode(cia402_mode_t mode);

/**
  * @brief  速度模式：设置目标速度并触发 PDO 发送。
  *         内部按 pulse_per_rev 把 rpm 换算成 pps，写控制字(保持OP) + 目标速度，触发 TPDO。
  * @param  rpm  目标速度（单位：转/分钟）
  * @pre    当前为 CIA402_RUNNING 且模式为 PROFILE_VEL
  */
void cia402_set_velocity(int32_t rpm);

/**
  * @brief  位置模式：设置绝对目标位置。
  *         语义固定为"最终要走到 position"，内部按当前运行模式自动选择下发方式：
  *           PROFILE_POS → 写一次 0x607A，由驱动器规划（主机不插值）
  *           CSP         → 启动软件 S 曲线，由 cia402_process 周期逐点下发
  *         调用方不需要知道当前是哪种模式。
  * @param  position  目标位置（单位：脉冲数）
  * @pre    当前为 CIA402_RUNNING 且模式为 PROFILE_POS 或 CSP
  */
void cia402_set_position_abs(int32_t position);

/**
  * @brief  位置模式：设置相对目标位置（在当前实际位置基础上偏移）并触发 PDO 发送。
  * @param  delta  相对偏移量（单位：脉冲数，正=正转，负=反转）
  * @pre    当前为 CIA402_RUNNING 且模式为 PROFILE_POS
  */
void cia402_set_position_rel(int32_t delta);

/**
  * @brief  位置是否已到达
  * @note   要求【两个条件同时满足】：
  *           1. 软件轨迹已跑完（CSP 模式下，中间点会让 bit10 反复抖动）
  *           2. 从站状态字 bit10（Target reached）为 1
  *         只判软件规划完成会误报 —— 那只能说明"点发完了"，
  *         不代表电机真的到位（比如从站根本没跟随）。
  * @retval 非位置模式（如速度模式）下恒返回 false
  */
bool_t cia402_position_reached(void);

/**
  * @brief  获取从站状态字（最近一次 TPDO 收到的值）
  */
uint16_t cia402_get_statusword(void);

/**
  * @brief  获取实际速度（速度模式下，最近一次 TPDO 收到的值）
  * @return 速度（单位：rpm，内部从 pps 换算）
  */
int32_t cia402_get_actual_velocity(void);

/**
  * @brief  获取实际位置（位置模式下，最近一次 TPDO 收到的值）
  */
int32_t cia402_get_actual_position(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* CIA402_H */