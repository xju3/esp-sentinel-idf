// ============================================================
// Implementation
// ============================================================
#include "logger.h"
#include "algo_rms.h"
#include "string.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "algo_baseline.h"

// esp-dsp: biquad 硬件加速 (ESP32-S3)
#include "dsps_biquad.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------- Welford (unchanged) ----------------

void algo_welford_init(algo_welford_t *ctx) {
    if (ctx) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

void algo_welford_update(algo_welford_t *ctx, float x, float y, float z) {
    if (!ctx) return;
    ctx->count++;

    double dx = (double)x - ctx->mean_x;
    ctx->mean_x += dx / ctx->count;
    ctx->m2_x += dx * ((double)x - ctx->mean_x);

    double dy = (double)y - ctx->mean_y;
    ctx->mean_y += dy / ctx->count;
    ctx->m2_y += dy * ((double)y - ctx->mean_y);

    double dz = (double)z - ctx->mean_z;
    ctx->mean_z += dz / ctx->count;
    ctx->m2_z += dz * ((double)z - ctx->mean_z);
}

void algo_welford_finish(const algo_welford_t *ctx,
                         const vib_baseline_t *baseline,
                         float *out_x, float *out_y, float *out_z)
{
    if (!ctx || !out_x || !out_y || !out_z) return;

    if (ctx->count == 0) {
        *out_x = *out_y = *out_z = 0.0f;
        return;
    }

    float var_x = (ctx->count > 1) ? (float)(ctx->m2_x / (ctx->count - 1)) : 0.0f;
    float var_y = (ctx->count > 1) ? (float)(ctx->m2_y / (ctx->count - 1)) : 0.0f;
    float var_z = (ctx->count > 1) ? (float)(ctx->m2_z / (ctx->count - 1)) : 0.0f;

    float base_mean_x = baseline ? baseline->x.val : 0.0f;
    float base_mean_y = baseline ? baseline->y.val : 0.0f;
    float base_mean_z = baseline ? baseline->z.val : 0.0f;

    float rms_x = sqrtf(fmaxf(0.0f, var_x + (float)pow(ctx->mean_x - base_mean_x, 2)));
    float rms_y = sqrtf(fmaxf(0.0f, var_y + (float)pow(ctx->mean_y - base_mean_y, 2)));
    float rms_z = sqrtf(fmaxf(0.0f, var_z + (float)pow(ctx->mean_z - base_mean_z, 2)));

    if (baseline) {
        *out_x = fmaxf(0.0f, rms_x - baseline->x.offset);
        *out_y = fmaxf(0.0f, rms_y - baseline->y.offset);
        *out_z = fmaxf(0.0f, rms_z - baseline->z.offset);
    } else {
        *out_x = rms_x;
        *out_y = rms_y;
        *out_z = rms_z;
    }
}

// ---------------- ISO10816 helpers ----------------

static inline void algo_biquad_reset(algo_biquad_t *q) {
    if (!q) return;
    q->w[0] = 0.0f;
    q->w[1] = 0.0f;
}

static inline float algo_biquad_process(algo_biquad_t *q, float x) {
    // 1-sample 处理封装：内部仍走 esp-dsp 的 DF-II 实现（可能走硬件优化路径）
    float y = 0.0f;
    (void)dsps_biquad_f32(&x, &y, 1, q->coef, q->w);
    return y;
}

// Butterworth 2nd-order biquad (RBJ cookbook style)
// type: 0=LP, 1=HP
static void algo_biquad_design_butter2(algo_biquad_t *q, int type, float fc, float fs) {
    if (!q || fs <= 0.0f) return;

    if (fc < 0.001f) fc = 0.001f;
    if (fc > 0.499f * fs) fc = 0.499f * fs;

    const float Q = 0.70710678118f;
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;
    if (type == 0) {
        // LP
        b0 = (1.0f - cosw0) * 0.5f;
        b1 = 1.0f - cosw0;
        b2 = (1.0f - cosw0) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 = 1.0f - alpha;
    } else {
        // HP
        b0 = (1.0f + cosw0) * 0.5f;
        b1 = -(1.0f + cosw0);
        b2 = (1.0f + cosw0) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 = 1.0f - alpha;
    }

    float inv_a0 = 1.0f / a0;

    // esp-dsp 需要的系数顺序：b0,b1,b2,a1,a2 (a0=1)
    q->coef[0] = b0 * inv_a0;
    q->coef[1] = b1 * inv_a0;
    q->coef[2] = b2 * inv_a0;
    q->coef[3] = a1 * inv_a0;
    q->coef[4] = a2 * inv_a0;

    // IMPORTANT: do not reset state here.
}

// 工业级确定性：每窗口先做一次 “均值相减” 作为强制去直流。
// 目的：把重力 1g 以及零偏从积分链路中彻底移除，否则积分会把 DC 无限放大。
static inline void iso10816_remove_mean_3axis(const float *ax, const float *ay, const float *az,
                                             int N,
                                             float *mx, float *my, float *mz)
{
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (int i = 0; i < N; ++i) {
        sx += ax[i];
        sy += ay[i];
        sz += az[i];
    }
    const double invN = 1.0 / (double)N;
    *mx = (float)(sx * invN);
    *my = (float)(sy * invN);
    *mz = (float)(sz * invN);
}

static void iso10816_design_if_needed(iso10816_state_t *st, float fs) {
    // Avoid coefficient churn on tiny fs jitter.
    float tol = (st->fs_tol_hz > 0.0f) ? st->fs_tol_hz : 50.0f;

    if (st->fs > 0.0f && fabsf(st->fs - fs) <= tol) {
        return;
    }

    const float fc_hp10 = 10.0f;
    const float fc_hp1  = 1.0f;

    float fc_lp = (st->fc_lp_hz > 0.0f) ? st->fc_lp_hz : 1000.0f;
    float max_lp = 0.45f * fs;
    if (fc_lp > max_lp) fc_lp = max_lp;

    algo_biquad_design_butter2(&st->hp10_x, 1, fc_hp10, fs);
    algo_biquad_design_butter2(&st->hp10_y, 1, fc_hp10, fs);
    algo_biquad_design_butter2(&st->hp10_z, 1, fc_hp10, fs);

    algo_biquad_design_butter2(&st->lp_x, 0, fc_lp, fs);
    algo_biquad_design_butter2(&st->lp_y, 0, fc_lp, fs);
    algo_biquad_design_butter2(&st->lp_z, 0, fc_lp, fs);

    algo_biquad_design_butter2(&st->hp1_vx, 1, fc_hp1, fs);
    algo_biquad_design_butter2(&st->hp1_vy, 1, fc_hp1, fs);
    algo_biquad_design_butter2(&st->hp1_vz, 1, fc_hp1, fs);

    // When coefficients are (re)designed, reset states to avoid old-state mismatch.
    algo_biquad_reset(&st->hp10_x); algo_biquad_reset(&st->hp10_y); algo_biquad_reset(&st->hp10_z);
    algo_biquad_reset(&st->lp_x);   algo_biquad_reset(&st->lp_y);   algo_biquad_reset(&st->lp_z);
    algo_biquad_reset(&st->hp1_vx); algo_biquad_reset(&st->hp1_vy); algo_biquad_reset(&st->hp1_vz);

    st->vx = st->vy = st->vz = 0.0f;
    st->fs = fs;
}

// ---------------- ISO10816 public ----------------

void iso10816_init(iso10816_state_t *st) {
    if (!st) return;
    memset(st, 0, sizeof(*st));

    // Defaults (you can override with setters)
    st->fc_lp_hz = 1000.0f;
    st->skip_seconds = 0.25f; // good for 1-second snapshots
    st->fs_tol_hz = 50.0f;
    st->leak_hz = 0.5f; // integrator bleed to suppress DC drift
}

void iso10816_set_fc_lp(iso10816_state_t *st, float fc_lp_hz) {
    if (!st) return;
    st->fc_lp_hz = fc_lp_hz;
}

void iso10816_set_skip_seconds(iso10816_state_t *st, float skip_seconds) {
    if (!st) return;
    st->skip_seconds = skip_seconds;
}

void iso10816_set_fs_tolerance(iso10816_state_t *st, float fs_tol_hz) {
    if (!st) return;
    st->fs_tol_hz = fs_tol_hz;
}

void iso10816_set_leak_hz(iso10816_state_t *st, float leak_hz) {
    if (!st) return;
    st->leak_hz = leak_hz;
}

void iso10816_compute(iso10816_state_t *st,
                      const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out)
{
    if (!st || !ax || !ay || !az || !out || N <= 0 || fs <= 0.0f) {
        return;
    }

    // 输出确定性：无论什么异常路径，先清零输出。
    out->vx_rms = out->vy_rms = out->vz_rms = out->v3d_rms = 0.0f;

    // Design (or reuse) coefficients
    iso10816_design_if_needed(st, fs);

    // (0) 每窗口独立：重置滤波器状态 + 积分器，避免“跨窗口记忆”导致确定性下降。
    // 你的业务模型是“每分钟抓 1 秒快照”，因此应把状态视作每窗口局部。
    // 若未来要做连续流式监测，可把下面 reset 移到外部，仅在需要时调用。
    algo_biquad_reset(&st->hp10_x); algo_biquad_reset(&st->hp10_y); algo_biquad_reset(&st->hp10_z);
    algo_biquad_reset(&st->lp_x);   algo_biquad_reset(&st->lp_y);   algo_biquad_reset(&st->lp_z);
    algo_biquad_reset(&st->hp1_vx); algo_biquad_reset(&st->hp1_vy); algo_biquad_reset(&st->hp1_vz);
    st->vx = st->vy = st->vz = 0.0f;

    const float dt = 1.0f / fs;
    const float g_to_ms2 = 9.80665f;
    const float leak = (st->leak_hz > 0.0f) ? expf(-2.0f * (float)M_PI * st->leak_hz * dt) : 1.0f;

    int skip = 0;
    if (st->skip_seconds > 0.0f) {
        skip = (int)lroundf(st->skip_seconds * fs);
        if (skip < 0) skip = 0;
        if (skip > N) skip = N;
    }

    // (1) 强制去直流：每窗口均值相减（直接去掉 1g + 零偏）
    // 说明：即使后面还有 10Hz HPF，这一步也必须做。
    // 原因：
    //  - IIR HPF 有启动瞬态/有限衰减；
    //  - 在极低频/数值误差下仍会有残余 DC，被积分无限放大；
    //  - 均值相减能把 DC 分量在数值上“硬清零”，确定性更强。
    float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
    iso10816_remove_mean_3axis(ax, ay, az, N, &mean_x, &mean_y, &mean_z);

    double sum_vx2 = 0.0, sum_vy2 = 0.0, sum_vz2 = 0.0;
    int used = 0;

    for (int i = 0; i < N; ++i) {
        // 先减去每窗口均值，再换算为 m/s^2
        float x = (ax[i] - mean_x) * g_to_ms2;
        float y = (ay[i] - mean_y) * g_to_ms2;
        float z = (az[i] - mean_z) * g_to_ms2;

        // Accel band-pass: 10Hz HP + LP
        x = algo_biquad_process(&st->hp10_x, x);
        y = algo_biquad_process(&st->hp10_y, y);
        z = algo_biquad_process(&st->hp10_z, z);

        x = algo_biquad_process(&st->lp_x, x);
        y = algo_biquad_process(&st->lp_y, y);
        z = algo_biquad_process(&st->lp_z, z);

        // Integrate to velocity (m/s)
        st->vx = st->vx * leak + x * dt;
        st->vy = st->vy * leak + y * dt;
        st->vz = st->vz * leak + z * dt;

        // Velocity HP at 1Hz
        float vx_hp = algo_biquad_process(&st->hp1_vx, st->vx);
        float vy_hp = algo_biquad_process(&st->hp1_vy, st->vy);
        float vz_hp = algo_biquad_process(&st->hp1_vz, st->vz);

        // Let filters settle before accumulating RMS
        if (i >= skip) {
            float vx_mmps = vx_hp * 1000.0f;
            float vy_mmps = vy_hp * 1000.0f;
            float vz_mmps = vz_hp * 1000.0f;

            sum_vx2 += (double)vx_mmps * vx_mmps;
            sum_vy2 += (double)vy_mmps * vy_mmps;
            sum_vz2 += (double)vz_mmps * vz_mmps;
            used++;
        }
    }

    if (used <= 0) {
        return;
    }

    out->vx_rms = (float)sqrt(sum_vx2 / used);
    out->vy_rms = (float)sqrt(sum_vy2 / used);
    out->vz_rms = (float)sqrt(sum_vz2 / used);

    out->v3d_rms = sqrtf(out->vx_rms * out->vx_rms +
                         out->vy_rms * out->vy_rms +
                         out->vz_rms * out->vz_rms);
}

/*
调用示例（与当前 monitor_task_loop 的用法一致）：

    iso10816_state_t st;
    iso10816_init(&st);
    // 可选：
    // iso10816_set_fc_lp(&st, 1000.0f);
    // iso10816_set_skip_seconds(&st, 0.25f);
    // iso10816_set_leak_hz(&st, 0.5f);

    iso10816_result_t res;
    iso10816_compute(&st, ax_g, ay_g, az_g, N, fs, &res);
    // res.vx_rms/res.vy_rms/res.vz_rms/res.v3d_rms 单位：mm/s RMS

关键链路说明（工业级确定性要点）：
  1) 每窗口均值相减：硬去 DC（含 1g 重力 + 三轴零偏），避免积分无限放大。
  2) 加速度带通：10Hz 高通 + 低通(fc_lp)，进一步抑制低频/高频非目标能量。
  3) 时域积分：v[n] = leak*v[n-1] + a[n]*dt，其中 leak = exp(-2*pi*leak_hz*dt)
     leak_hz 相当于给积分器加一个极低频“泄漏”，抑制漂移。
  4) 速度再高通(1Hz)：作为二次兜底，滤除残余极低频漂移。
*/
