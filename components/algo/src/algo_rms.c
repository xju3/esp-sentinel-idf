/*
 * algo_rms.c - Root Mean Square (RMS) Calculation
 *
 * Minimal RMS implementation used by任务级别的速度RMS计算。
 */

#include "algo_pdm.h"
#include "algo_rms.h"
#include <math.h>

float algo_calc_rms(const float *data, size_t count)
{
    if (data == NULL || count == 0)
    {
        return 0.0f;
    }

    // Kahan 求和，降低累积误差
    float sum_sq = 0.0f;
    float compensation = 0.0f;

    for (size_t i = 0; i < count; i++)
    {
        float val = data[i];
        float term = val * val - compensation;
        float temp = sum_sq + term;

        compensation = (temp - sum_sq) - term;
        sum_sq = temp;
    }

    return sqrtf(sum_sq / count);
}

/* ========================================================================= *
 * 速度 RMS 诊断（mm/s）无动态分配实现
 * ========================================================================= */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void algo_rms_diag_init(algo_rms_diag_state_t *st, float dt, float hp_fc_hz)
{
    if (!st || dt <= 0.0f || hp_fc_hz <= 0.0f)
        return;
    *st = (algo_rms_diag_state_t){
        .dt = dt,
        .hp_alpha = 0.0f,
        .count = 0,
        .vx_ms = 0.0f,
        .vy_ms = 0.0f,
        .vz_ms = 0.0f,
        .mean_ax = 0.0f,
        .mean_ay = 0.0f,
        .mean_az = 0.0f,
        .hp_vx_prev_in = 0.0f,
        .hp_vy_prev_in = 0.0f,
        .hp_vz_prev_in = 0.0f,
        .hp_vx_prev_out = 0.0f,
        .hp_vy_prev_out = 0.0f,
        .hp_vz_prev_out = 0.0f,
        .sum_vx2 = 0.0,
        .sum_vy2 = 0.0,
        .sum_vz2 = 0.0,
    };

    float rc = 1.0f / (2.0f * (float)M_PI * hp_fc_hz);
    st->hp_alpha = rc / (rc + dt);
}

void algo_rms_diag_update(algo_rms_diag_state_t *st,
                          const imu_raw_data_t *data, size_t count)
{
    if (!st || !data || count == 0)
        return;

    for (size_t i = 0; i < count; i++)
    {
        float ax_ms2 = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G * 9.80665f;
        float ay_ms2 = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G * 9.80665f;
        float az_ms2 = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G * 9.80665f;

        // Running mean for DC removal
        float n = (float)(st->count + 1);
        st->mean_ax += (ax_ms2 - st->mean_ax) / n;
        st->mean_ay += (ay_ms2 - st->mean_ay) / n;
        st->mean_az += (az_ms2 - st->mean_az) / n;

        ax_ms2 -= st->mean_ax;
        ay_ms2 -= st->mean_ay;
        az_ms2 -= st->mean_az;

        // Integrate to velocity (m/s)
        st->vx_ms += ax_ms2 * st->dt;
        st->vy_ms += ay_ms2 * st->dt;
        st->vz_ms += az_ms2 * st->dt;

        // 1st-order high-pass on velocity to suppress drift (default fc≈1Hz)
        float vx_hp = st->hp_alpha * (st->hp_vx_prev_out + st->vx_ms - st->hp_vx_prev_in);
        float vy_hp = st->hp_alpha * (st->hp_vy_prev_out + st->vy_ms - st->hp_vy_prev_in);
        float vz_hp = st->hp_alpha * (st->hp_vz_prev_out + st->vz_ms - st->hp_vz_prev_in);
        st->hp_vx_prev_in = st->vx_ms;
        st->hp_vy_prev_in = st->vy_ms;
        st->hp_vz_prev_in = st->vz_ms;
        st->hp_vx_prev_out = vx_hp;
        st->hp_vy_prev_out = vy_hp;
        st->hp_vz_prev_out = vz_hp;

        float vx_mmps = vx_hp * 1000.0f;
        float vy_mmps = vy_hp * 1000.0f;
        float vz_mmps = vz_hp * 1000.0f;

        st->sum_vx2 += (double)vx_mmps * (double)vx_mmps;
        st->sum_vy2 += (double)vy_mmps * (double)vy_mmps;
        st->sum_vz2 += (double)vz_mmps * (double)vz_mmps;
        st->count++;
    }
}

void algo_rms_diag_finish(const algo_rms_diag_state_t *st,
                          algo_rms_diag_result_t *out)
{
    if (!st || !out || st->count == 0)
        return;

    float rms_vx = (float)sqrt(st->sum_vx2 / (double)st->count);
    float rms_vy = (float)sqrt(st->sum_vy2 / (double)st->count);
    float rms_vz = (float)sqrt(st->sum_vz2 / (double)st->count);
    out->rms_vx = rms_vx;
    out->rms_vy = rms_vy;
    out->rms_vz = rms_vz;
    out->rms_v3d = sqrtf(rms_vx * rms_vx + rms_vy * rms_vy + rms_vz * rms_vz);
    out->count = st->count;
    out->fs_hz = (st->dt > 0.0f) ? (1.0f / st->dt) : 0.0f;
}
