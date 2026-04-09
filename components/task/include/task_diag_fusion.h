#ifndef TASK_DIAG_FUSION_H
#define TASK_DIAG_FUSION_H

#include "algo_envelope.h"
#include "algo_fft.h"
#include "algo_kurtosis.h"
#include "esp_err.h"
#include "task_stash.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t timestamp;
    uint32_t length;
    float sample_rate;
    patrol_peak_t peaks[PATROL_MAX_PEAKS];
} diag_fft_summary_t;

typedef struct
{
    int32_t timestamp;
    uint32_t length;
    float sample_rate;
    vib_kurtosis_t kurtosis;
    float max_kurtosis;
    bool threshold_exceeded;
} diag_kurtosis_summary_t;

typedef struct
{
    int32_t timestamp;
    uint32_t length;
    float sample_rate;
    envelope_report_t x_axis;
    envelope_report_t y_axis;
    envelope_report_t z_axis;
} diag_envelope_summary_t;

esp_err_t start_diag_fusion_task(void);
esp_err_t diag_fusion_submit_fft(const diag_fft_summary_t *summary);
esp_err_t diag_fusion_submit_kurtosis(const diag_kurtosis_summary_t *summary);
esp_err_t diag_fusion_submit_envelope(const diag_envelope_summary_t *summary);

#ifdef __cplusplus
}
#endif

#endif // TASK_DIAG_FUSION_H
