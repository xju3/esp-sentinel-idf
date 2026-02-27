#include "task_rms.h"
#include "algo_pdm.h"
#include "algo_rms.h"
#include "config_manager.h"
#include "daq_icm_42688_p.h"
#include "drv_icm_42688_p.h"
#include "logger.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RMS_WINDOW_MS 1000
#define RMS_CHUNK_SAMPLES 128
#define G_TO_MS2 9.80665f

static float odr_to_fs_hz(icm_odr_t odr)
{
    return icm_odr_to_hz(odr);
}

static void rms_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    algo_rms_diag_state_t *st = (algo_rms_diag_state_t *)ctx;
    if (!st || !data)
        return;
    algo_rms_diag_update(st, data, count);
}

void run_rms_diagnosis(void)
{
    icm_cfg_t cfg = {0};
    cfg.odr = calculate_patrol_odr(g_user_config.rpm);
    cfg.fs = ICM_FS_16G;
    cfg.enable_wom = false;
    cfg.wom_thr_mg = 0;

    float fs_hz = odr_to_fs_hz(cfg.odr);
    if (fs_hz <= 0.0f)
    {
        LOG_ERRORF("Unsupported ODR: %d", cfg.odr);
        return;
    }

    float dt = 1.0f / fs_hz;

    algo_rms_diag_state_t diag_st;
    algo_rms_diag_init(&diag_st, dt, 1.0f); // fc ≈ 1 Hz high-pass on velocity

    esp_err_t err = daq_icm_42688_p_capture(&cfg,
                                            RMS_WINDOW_MS,
                                            rms_chunk_handler,
                                            &diag_st,
                                            RMS_CHUNK_SAMPLES,
                                            0);
    if (err != ESP_OK)
    {
        LOG_ERRORF("RMS capture failed: %s", esp_err_to_name(err));
        return;
    }

    if (diag_st.count == 0)
    {
        LOG_WARN("No samples captured for RMS diagnosis");
        return;
    }

    algo_rms_diag_result_t res = {.fs_hz = fs_hz};
    algo_rms_diag_finish(&diag_st, &res);

    LOG_INFOF("RMS velocity (mm/s): X=%.3f, Y=%.3f, Z=%.3f, 3D=%.3f, N=%d, fs=%.0fHz",
              res.rms_vx, res.rms_vy, res.rms_vz, res.rms_v3d, (int)res.count, fs_hz);
}
