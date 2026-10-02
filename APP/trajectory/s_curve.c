/* APP/trajectory/s_curve.c — 7段式 S 曲线实现（分段解析式求值） */
#include "s_curve.h"
#include <math.h>   /* sqrt */
#include <stdio.h>  /* printf */

/* 毫秒 → 秒 */
#define MS_TO_S(ms)   ((double)(ms) / 1000.0)
/* 秒 → 毫秒（四舍五入） */
#define S_TO_MS(s)    ((uint32_t)((s) * 1000.0 + 0.5))

/** 四舍五入到最近的整数（向零截断对负数会偏，这里显式处理） */
static int32_t round_signed(double x)
{
    return (int32_t)((x >= 0.0) ? (x + 0.5) : (x - 0.5));
}

void s_curve_init(s_curve_t* sc, int32_t target, int32_t start,
                  uint32_t v_max, uint32_t a_max, uint32_t jerk, uint32_t dt_ms)
{
    sc->target = target;
    sc->start  = start;
    sc->v_max  = v_max;
    sc->a_max  = a_max;
    sc->jerk   = jerk;
    sc->dt_ms  = dt_ms;

    sc->dir    = 1.0;
    sc->v_peak = 0;
    sc->idx    = 7;          /* 默认已是完成态 */
    sc->t_ms   = 0;
    sc->pos    = start;
    sc->done   = false;

    /* ── 参数合法性：任一为 0 都会除零，直接判定完成 ── */
    if (v_max == 0u || a_max == 0u || jerk == 0u) {
        printf("[SCURVE] 参数非法 v=%lu a=%lu j=%lu，放弃规划\r\n",
               (unsigned long)v_max, (unsigned long)a_max, (unsigned long)jerk);
        sc->done = true;
        sc->pos  = target;
        return;
    }

    int32_t delta = target - start;
    if (delta == 0) {
        sc->done = true;
        sc->pos  = target;
        return;
    }
    sc->dir = (delta > 0) ? 1.0 : -1.0;

    /* ── 规划（全程按正方向算） ──
     * T1=T3=T5=T7 = a_max / jerk           （jerk 段；距离过短时会一起缩短）
     * T2=T6 = v_peak/a_max - T1            （恒加速段，可为 0）
     * T4 = (总距离 - 2×加速段距离) / v_peak （匀速段，可为 0）
     */
    double total = (double)((delta > 0) ? delta : -delta);
    double T1    = (double)a_max / (double)jerk;

    /* 加速段距离 d_acc(v) = v²/(2·a_max) + v·a_max/(2·jerk)
     * 若 2·d_acc(v_max) > 总距离，说明距离不够加速到 v_max，需缩减：
     *   v²/a_max + v·a_max/jerk = 总距离
     *   A·v² + B·v + C = 0,  A = 1/a_max, B = a_max/jerk, C = -总距离
     */
    double d_acc_at_vmax = (double)v_max * (double)v_max / (2.0 * (double)a_max)
                         + (double)v_max * (double)a_max / (2.0 * (double)jerk);

    double v_peak;
    double T2;
    double T4;

    if (2.0 * d_acc_at_vmax <= total) {
        /* ① 距离够：加速到 v_max，中间有匀速段 */
        v_peak = (double)v_max;
        T2     = v_peak / (double)a_max - T1;              /* ≥ 0 */
        T4     = (total - 2.0 * d_acc_at_vmax) / v_peak;   /* ≥ 0 */
    } else {
        /* ② 距离不够加速到 v_max：先砍掉匀速段，按"加速+减速刚好填满全程"
         *    解峰值速度（A·v² + B·v + C = 0，见文件头推导） */
        double A    = 1.0 / (double)a_max;
        double B    = (double)a_max / (double)jerk;
        double C    = -total;
        double disc = B * B - 4.0 * A * C;
        v_peak = (disc > 0.0) ? (-B + sqrt(disc)) / (2.0 * A) : 0.0;
        T2     = v_peak / (double)a_max - T1;
        T4     = 0.0;

        if (T2 < 0.0) {
            /* ②b 距离短到连 a_max 都加不到（T2 求出来是负的）。
             *     此时 T1 也必须缩短 —— 加速度同样要削峰，否则光 jerk 段
             *     就冲出去 a_max·T1² 个脉冲（10 脉冲的目标能过冲 8000+）。
             *     曲线退化成 P1/P3/P5/P7 四段等长，无恒加速段、无匀速段：
             *       半程距离 = jerk·T1³  →  全程 = 2·jerk·T1³
             *     T1 = ∛(总距离 / (2·jerk)) */
            T1     = cbrt(total / (2.0 * (double)jerk));
            T2     = 0.0;
            T4     = 0.0;
            v_peak = (double)jerk * T1 * T1;   /* 峰值速度（峰值加速度为 jerk·T1） */
        }
    }

    sc->v_peak = (uint32_t)(v_peak + 0.5);

    uint32_t d1 = S_TO_MS(T1);
    uint32_t d2 = S_TO_MS(T2);
    uint32_t d4 = S_TO_MS(T4);

    /* ── 各段 jerk 与时长 ── */
    uint32_t dur[7] = { d1, d2, d1, d4, d1, d2, d1 };
    double   jj     = (double)jerk;
    double   jrk[7] = { +jj, 0.0, -jj, 0.0, -jj, 0.0, +jj };

    /* ── 逐段串接段起点状态：段末状态 = 段起点 + 解析式在 T 处的取值 ── */
    double p = 0.0, v = 0.0, a = 0.0;
    for (uint8_t i = 0u; i < 7u; i++) {
        sc->seg[i].p0     = p;
        sc->seg[i].v0     = v;
        sc->seg[i].a0     = a;
        sc->seg[i].j      = jrk[i];
        sc->seg[i].dur_ms = dur[i];

        double T = MS_TO_S(dur[i]);
        p += v * T + 0.5 * a * T * T + jrk[i] * T * T * T / 6.0;
        v += a * T + 0.5 * jrk[i] * T * T;
        a += jrk[i] * T;
    }

    sc->idx  = 0;
    sc->t_ms = 0;
    sc->done = false;
    sc->pos  = start;

    printf("[SCURVE] target=%ld start=%ld delta=%ld dir=%d v_peak=%lu\r\n",
           (long)target, (long)start, (long)delta, (sc->dir > 0.0) ? 1 : -1,
           (unsigned long)sc->v_peak);
    printf("[SCURVE] 各段时长(ms): %lu %lu %lu %lu %lu %lu %lu  (总 %lu)\r\n",
           (unsigned long)d1, (unsigned long)d2, (unsigned long)d1,
           (unsigned long)d4, (unsigned long)d1, (unsigned long)d2,
           (unsigned long)d1,
           (unsigned long)(4u * d1 + 2u * d2 + d4));
}

int32_t s_curve_step(s_curve_t* sc, uint32_t elapsed_ms)
{
    if (sc->done || sc->idx >= 7u) {
        return sc->pos;
    }

    /* 主循环卡顿时的保护：一次最多推进 SCURVE_MAX_STEP_MS */
    if (elapsed_ms > SCURVE_MAX_STEP_MS) {
        elapsed_ms = SCURVE_MAX_STEP_MS;
    }
    sc->t_ms += elapsed_ms;

    /* 跨段推进：把已经走完的段依次吃掉（时长为 0 的段在这里被直接跳过） */
    while (sc->idx < 7u && sc->t_ms >= sc->seg[sc->idx].dur_ms) {
        sc->t_ms -= sc->seg[sc->idx].dur_ms;
        sc->idx++;
    }

    if (sc->idx >= 7u) {
        /* 规划结束：对齐到目标并清零速度/加速度状态 */
        sc->done = true;
        sc->pos  = sc->target;
        return sc->pos;
    }

    const s_curve_seg_t* g = &sc->seg[sc->idx];
    double t  = MS_TO_S(sc->t_ms);
    double dp = g->p0
              + g->v0 * t
              + 0.5 * g->a0 * t * t
              + g->j * t * t * t / 6.0;

    sc->pos = sc->start + round_signed(sc->dir * dp);
    return sc->pos;
}

bool s_curve_done(const s_curve_t* sc)
{
    return sc->done;
}
