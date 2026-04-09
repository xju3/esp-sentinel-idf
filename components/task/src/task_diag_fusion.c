#include "task_diag_fusion.h"

#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "logger.h"

#include <math.h>
#include <string.h>

#define DIAG_FUSION_QUEUE_LEN 8
#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

typedef enum
{
    DIAG_FUSION_EVT_FFT = 0,
    DIAG_FUSION_EVT_KURTOSIS,
    DIAG_FUSION_EVT_ENVELOPE,
} diag_fusion_event_type_t;

typedef struct
{
    diag_fusion_event_type_t type;
    union
    {
        diag_fft_summary_t fft;
        diag_kurtosis_summary_t kurtosis;
        diag_envelope_summary_t envelope;
    } data;
} diag_fusion_event_t;

typedef struct
{
    bool has_fft;
    bool has_kurtosis;
    bool has_envelope;
    int32_t timestamp;
    uint32_t length;
    float sample_rate;
    diag_fft_summary_t fft;
    diag_kurtosis_summary_t kurtosis;
    diag_envelope_summary_t envelope;
} diag_fusion_state_t;

static QueueHandle_t s_diag_fusion_queue = NULL;

static bool same_window(int32_t ts_a, uint32_t len_a, float sr_a,
                        int32_t ts_b, uint32_t len_b, float sr_b)
{
    return ts_a == ts_b && len_a == len_b && fabsf(sr_a - sr_b) < 1.0f;
}

static float peak_score_local(const patrol_peak_t *peak)
{
    float score = peak->amp_x;
    if (peak->amp_y > score)
    {
        score = peak->amp_y;
    }
    if (peak->amp_z > score)
    {
        score = peak->amp_z;
    }
    return score;
}

static const char *pick_envelope_fault(const envelope_report_t *report, float *confidence, float *freq_hz)
{
    if (!report || !confidence || !freq_hz)
    {
        return "none";
    }

    const fault_stat_t *best = NULL;
    const char *label = "none";
    const fault_stat_t *candidates[4] = {
        &report->bpfo_stat,
        &report->bpfi_stat,
        &report->bs_stat,
        &report->ftf_stat,
    };
    const char *labels[4] = {"BPFO", "BPFI", "BS", "FTF"};

    for (int i = 0; i < 4; ++i)
    {
        if (!candidates[i]->detected)
        {
            continue;
        }
        if (best == NULL || candidates[i]->confidence > best->confidence)
        {
            best = candidates[i];
            label = labels[i];
        }
    }

    if (best == NULL)
    {
        *confidence = 0.0f;
        *freq_hz = 0.0f;
        return "none";
    }

    *confidence = best->confidence;
    *freq_hz = best->frequency_hz;
    return label;
}

static void reset_state(diag_fusion_state_t *state)
{
    memset(state, 0, sizeof(*state));
}

static void maybe_emit_summary(diag_fusion_state_t *state)
{
    if (!state->has_fft || !state->has_kurtosis || !state->has_envelope)
    {
        return;
    }

    int best_peak_idx = -1;
    float best_peak_score = 0.0f;
    for (int i = 0; i < PATROL_MAX_PEAKS; ++i)
    {
        const float score = peak_score_local(&state->fft.peaks[i]);
        if (score > best_peak_score)
        {
            best_peak_score = score;
            best_peak_idx = i;
        }
    }

    const char *axis_label = "X";
    const envelope_report_t *best_env = &state->envelope.x_axis;
    if (state->envelope.y_axis.peak_mag > best_env->peak_mag)
    {
        best_env = &state->envelope.y_axis;
        axis_label = "Y";
    }
    if (state->envelope.z_axis.peak_mag > best_env->peak_mag)
    {
        best_env = &state->envelope.z_axis;
        axis_label = "Z";
    }

    float env_conf = 0.0f;
    float env_freq = 0.0f;
    const char *env_fault = pick_envelope_fault(best_env, &env_conf, &env_freq);

    if (best_peak_idx >= 0)
    {
        const patrol_peak_t *peak = &state->fft.peaks[best_peak_idx];
        LOG_INFOF("Diagnosis Summary: ts=%ld len=%lu sr=%.1fHz fft_peak=%.2fHz amp=%.4f dom=%c max_kurt=%.2f env_axis=%s env_peak=%.2fHz/%.4f env_fault=%s@%.2fHz conf=%.2f",
                  (long)state->timestamp,
                  (unsigned long)state->length,
                  state->sample_rate,
                  peak->freq_hz,
                  best_peak_score,
                  "XYZ"[peak->dominant_axis],
                  state->kurtosis.max_kurtosis,
                  axis_label,
                  best_env->peak_freq_hz,
                  best_env->peak_mag,
                  env_fault,
                  env_freq,
                  env_conf);
    }
    else
    {
        LOG_INFOF("Diagnosis Summary: ts=%ld len=%lu sr=%.1fHz fft_peak=none max_kurt=%.2f env_axis=%s env_peak=%.2fHz/%.4f env_fault=%s@%.2fHz conf=%.2f",
                  (long)state->timestamp,
                  (unsigned long)state->length,
                  state->sample_rate,
                  state->kurtosis.max_kurtosis,
                  axis_label,
                  best_env->peak_freq_hz,
                  best_env->peak_mag,
                  env_fault,
                  env_freq,
                  env_conf);
    }

    reset_state(state);
}

static void diag_fusion_task_entry(void *arg)
{
    (void)arg;
    diag_fusion_event_t event;
    diag_fusion_state_t state = {0};

    while (1)
    {
        if (!xQueueReceive(s_diag_fusion_queue, &event, portMAX_DELAY))
        {
            continue;
        }

        int32_t timestamp = 0;
        uint32_t length = 0;
        float sample_rate = 0.0f;
        switch (event.type)
        {
            case DIAG_FUSION_EVT_FFT:
                timestamp = event.data.fft.timestamp;
                length = event.data.fft.length;
                sample_rate = event.data.fft.sample_rate;
                break;
            case DIAG_FUSION_EVT_KURTOSIS:
                timestamp = event.data.kurtosis.timestamp;
                length = event.data.kurtosis.length;
                sample_rate = event.data.kurtosis.sample_rate;
                break;
            case DIAG_FUSION_EVT_ENVELOPE:
                timestamp = event.data.envelope.timestamp;
                length = event.data.envelope.length;
                sample_rate = event.data.envelope.sample_rate;
                break;
            default:
                continue;
        }

        if ((state.has_fft || state.has_kurtosis || state.has_envelope) &&
            !same_window(state.timestamp, state.length, state.sample_rate, timestamp, length, sample_rate))
        {
            LOG_WARNF("Diagnosis fusion state reset: incomplete window ts=%ld len=%lu sr=%.1f",
                      (long)state.timestamp,
                      (unsigned long)state.length,
                      state.sample_rate);
            reset_state(&state);
        }

        state.timestamp = timestamp;
        state.length = length;
        state.sample_rate = sample_rate;

        switch (event.type)
        {
            case DIAG_FUSION_EVT_FFT:
                state.fft = event.data.fft;
                state.has_fft = true;
                break;
            case DIAG_FUSION_EVT_KURTOSIS:
                state.kurtosis = event.data.kurtosis;
                state.has_kurtosis = true;
                break;
            case DIAG_FUSION_EVT_ENVELOPE:
                state.envelope = event.data.envelope;
                state.has_envelope = true;
                break;
            default:
                break;
        }

        maybe_emit_summary(&state);
    }
}

static esp_err_t submit_event(const diag_fusion_event_t *event)
{
    if (s_diag_fusion_queue == NULL || event == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(s_diag_fusion_queue, event, 0) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t diag_fusion_submit_fft(const diag_fft_summary_t *summary)
{
    if (summary == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    diag_fusion_event_t event = {
        .type = DIAG_FUSION_EVT_FFT,
    };
    event.data.fft = *summary;
    return submit_event(&event);
}

esp_err_t diag_fusion_submit_kurtosis(const diag_kurtosis_summary_t *summary)
{
    if (summary == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    diag_fusion_event_t event = {
        .type = DIAG_FUSION_EVT_KURTOSIS,
    };
    event.data.kurtosis = *summary;
    return submit_event(&event);
}

esp_err_t diag_fusion_submit_envelope(const diag_envelope_summary_t *summary)
{
    if (summary == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    diag_fusion_event_t event = {
        .type = DIAG_FUSION_EVT_ENVELOPE,
    };
    event.data.envelope = *summary;
    return submit_event(&event);
}

esp_err_t start_diag_fusion_task(void)
{
    if (s_diag_fusion_queue == NULL)
    {
        s_diag_fusion_queue = xQueueCreateWithCaps(DIAG_FUSION_QUEUE_LEN, sizeof(diag_fusion_event_t), TASK_MEM_CAPS);
        if (s_diag_fusion_queue == NULL)
        {
            LOG_ERROR("Failed to create DIAG fusion queue");
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreateWithCaps(diag_fusion_task_entry, "diag_fusion_task", 4096, NULL, 3, NULL, TASK_MEM_CAPS) != pdPASS)
    {
        LOG_ERROR("Failed to create DIAG fusion task");
        return ESP_ERR_INVALID_STATE;
    }

    LOG_DEBUG("DIAG fusion task started");
    return ESP_OK;
}
