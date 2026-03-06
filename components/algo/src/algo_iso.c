#include "algo_iso.h"
#include <stddef.h>

// ISO 10816-3 Vibration Severity Thresholds (mm/s RMS) for machine classes I-IV
// Each row represents a machine class (I, II, III, IV)
// Each column represents a foundation type (Rigid, Flexible)
// The value is an array of 3 thresholds: [Zone A/B, Zone B/C, Zone C/D]
static const float iso_10816_thresholds[4][2][3] = {
    // Class I (Category 1)
    {
        {1.4f, 2.8f, 4.5f}, // Rigid Foundation
        {2.3f, 4.5f, 7.1f}  // Flexible Foundation
    },
    // Class II (Category 2)
    {
        {2.3f, 4.5f, 7.1f}, // Rigid Foundation
        {3.5f, 7.1f, 11.0f} // Flexible Foundation
    },
    // Class III (Category 3)
    {
        {3.5f, 7.1f, 11.0f}, // Rigid Foundation
        {5.6f, 11.0f, 18.0f} // Flexible Foundation
    },
    // Class IV (Category 4)
    {
        {5.6f, 11.0f, 18.0f}, // Rigid Foundation
        {9.0f, 18.0f, 28.0f}  // Flexible Foundation
    }
};

iso_alarm_status_t iso10816_check(float rms_mm_s, const iso_config_t *config)
{
    if (!config) return ISO_STATUS_INVALID_CONFIG;

    // Validate category (1-4) and foundation (1-2) indices
    int category_idx = config->category - 1;
    int foundation_idx = config->foundation - 1;

    if (category_idx < 0 || category_idx >= 4 || foundation_idx < 0 || foundation_idx >= 2) {
        return ISO_STATUS_INVALID_CONFIG;
    }

    const float* thresholds = iso_10816_thresholds[category_idx][foundation_idx];
    
    if (rms_mm_s <= thresholds[1]) { // Below B/C boundary
        return (rms_mm_s <= thresholds[0]) ? ISO_STATUS_GOOD : ISO_STATUS_SATISFACTORY;
    } else { // Above B/C boundary
        return (rms_mm_s <= thresholds[2]) ? ISO_STATUS_UNSATISFACTORY : ISO_STATUS_UNACCEPTABLE;
    }
}

const char* iso_status_to_string(iso_alarm_status_t status)
{
    switch (status) {
        case ISO_STATUS_GOOD:           return "Good (Zone A)";
        case ISO_STATUS_SATISFACTORY:   return "Satisfactory (Zone B)";
        case ISO_STATUS_UNSATISFACTORY: return "Unsatisfactory (Zone C)";
        case ISO_STATUS_UNACCEPTABLE:   return "Unacceptable (Zone D)";
        case ISO_STATUS_INVALID_CONFIG: return "Invalid ISO Config";
        default:                        return "Unknown Status";
    }
}