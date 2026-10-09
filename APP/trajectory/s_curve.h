/* APP/trajectory/s_curve.h — 7段式 S 曲线轨迹规划【接口保留，实现未提供】
 *
 * ⚠ 本文件目前只有接口和语义说明，没有实现（实现已于 2026-10-09 清理时删除，
 *   需要时从 git 历史取回 s_curve.c，或照下面的语义重新实现）。
 *   全工程没有任何代码调用 s_curve_* —— 它是给 CSP 模式（主机逐点下发）预留的。
 *
 * 它解决什么问题：CSP 模式下主机必须持续下发中间点，若直接给驱动器一个位置
 * 阶跃，电机会抖；S 曲线把"从 start 走到 target"摊成一条 jerk 连续的位置曲线，
 * 每个调用周期取一个点下发，运动平滑且不丢步。
 *
 * 段内位置是时间的三次多项式（jerk 恒定），因此不需要数值积分：
 *   p(t) = p0 + v0·t + a0·t²/2 + j·t³/6
 * 每个周期直接代入当前段起点状态求值，误差只有最后的四舍五入，
 * 不会有逐步累积的漂移。
 *
 * 内部一律按【正方向】规划（v0/a0/j 均为正方向的量），
 * 输出时再用 dir 映射回实际方向。
 *
 * 调用流程（三步，典型放在 cia402 的 RUNNING 周期里）：
 *   s_curve_init(&sc, target, start, v_max, a_max, jerk, dt_ms);  // 一次
 *   每周期: p = s_curve_step(&sc, 实际间隔ms);  下发 p 到 0x607A
 *   收尾:   if (s_curve_done(&sc)) 停止下发 / 转为保持最后一点
 */
#ifndef _APP_TRAJECTORY_S_CURVE_H_
#define _APP_TRAJECTORY_S_CURVE_H_

#include <stdint.h>
#include <stdbool.h>

/** 单次步进允许的最大时间跨度（ms）。
 *  主循环被 printf 等阻塞时，限制轨迹一次跳过的距离 —— 否则在 CSP 模式下
 *  会给驱动器一个过大的位置阶跃，表现为"抖一下"。放宽的这部分时间会被
 *  丢弃，轨迹总体耗时相应变长，但位置曲线始终连续。 */
#define SCURVE_MAX_STEP_MS   100U

/** S 曲线 7 个阶段（对应 seg[0..6]） */
typedef enum {
    SCURVE_P1 = 0,   /**< 加加速：j = +jerk, a: 0 → +a_max */
    SCURVE_P2,       /**< 恒加速：j = 0,     a = +a_max */
    SCURVE_P3,       /**< 减加速：j = -jerk, a: +a_max → 0 */
    SCURVE_P4,       /**< 匀速：  j = 0,     a = 0, v = v_peak */
    SCURVE_P5,       /**< 加减速：j = -jerk, a: 0 → -a_max */
    SCURVE_P6,       /**< 恒减速：j = 0,     a = -a_max */
    SCURVE_P7,       /**< 减减速：j = +jerk, a: -a_max → 0 */
} s_curve_state_t;

/** 单段参数：段起点的状态 + 本段 jerk + 本段时长 */
typedef struct {
    double   p0;       /**< 段起点位置（相对 start 的增量，脉冲） */
    double   v0;       /**< 段起点速度（脉冲/秒） */
    double   a0;       /**< 段起点加速度（脉冲/秒²） */
    double   j;        /**< 段 jerk（脉冲/秒³，带符号） */
    uint32_t dur_ms;   /**< 段时长（毫秒） */
} s_curve_seg_t;

/** S 曲线规划器实例 */
typedef struct {
    /* ── 输入参数（原样保存，不会被改写） ── */
    int32_t  target;      /**< 目标位置（脉冲，绝对值） */
    int32_t  start;       /**< 起始位置（脉冲） */
    uint32_t v_max;       /**< 最大速度（脉冲/秒） */
    uint32_t a_max;       /**< 最大加速度（脉冲/秒²） */
    uint32_t jerk;        /**< 加加速度（脉冲/秒³） */
    uint32_t dt_ms;       /**< 期望的下发周期（毫秒），仅作参考 */

    /* ── 规划结果（s_curve_init 一次算好） ── */
    double       dir;     /**< 方向：+1.0 / -1.0 */
    uint32_t     v_peak;  /**< 实际峰值速度；距离不足时 < v_max */
    s_curve_seg_t seg[7]; /**< 7 段参数 */

    /* ── 运行时状态 ── */
    uint8_t  idx;         /**< 当前段 0..6；>= 7 表示已完成 */
    uint32_t t_ms;        /**< 当前段已用时（毫秒） */
    int32_t  pos;         /**< 最近一次输出的位置（脉冲，绝对值） */
    bool     done;        /**< 是否已到达目标 */
} s_curve_t;

/**
 * @brief  初始化一次 S 曲线定位
 * @note   参数非法（v_max/a_max/jerk 有 0）或目标等于起点时，直接置为完成态，
 *         由调用方按"保持当前位置"处理。
 * @param  sc      规划器实例
 * @param  target  目标位置（脉冲，绝对值）
 * @param  start   起始位置（脉冲）
 * @param  v_max   最大速度（脉冲/秒）
 * @param  a_max   最大加速度（脉冲/秒²）
 * @param  jerk    加加速度（脉冲/秒³）
 * @param  dt_ms   期望的调用周期（毫秒）
 */
void s_curve_init(s_curve_t* sc, int32_t target, int32_t start,
                  uint32_t v_max, uint32_t a_max, uint32_t jerk, uint32_t dt_ms);

/**
 * @brief  按实际经过的时间推进轨迹，返回当前应下发的位置
 * @param  sc          规划器实例
 * @param  elapsed_ms  距上次调用实际经过的毫秒数（内部按 SCURVE_MAX_STEP_MS 截断）
 * @retval 当前位置（脉冲，绝对值），直接喂给 CIA402 的 0x607A
 */
int32_t s_curve_step(s_curve_t* sc, uint32_t elapsed_ms);

/** 是否已到达目标 */
bool s_curve_done(const s_curve_t* sc);

#endif /* _APP_TRAJECTORY_S_CURVE_H_ */
