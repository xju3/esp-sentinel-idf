#ifndef ISO_CHECK_H
#define ISO_CHECK_H

#include "config_manager.h" // For iso_config_t

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ISO 10816 Vibration Severity Zones
 */
typedef enum {
    ISO_STATUS_GOOD,          // Zone A: Good, suitable for new machines.
    ISO_STATUS_SATISFACTORY,  // Zone B: Satisfactory, suitable for unrestricted long-term operation.
    ISO_STATUS_UNSATISFACTORY,// Zone C: Unsatisfactory, not suitable for long-term operation.
    ISO_STATUS_UNACCEPTABLE,  // Zone D: Unacceptable, damage may occur.
    ISO_STATUS_INVALID_CONFIG // Input configuration is not valid.
} iso_alarm_status_t;

/**
 * @brief Checks the vibration RMS value against ISO 10816-3 standard.
 * 
 * @param rms_mm_s The vibration velocity RMS value in mm/s.
 * @param config Pointer to the ISO configuration struct containing machine class and foundation type.
 * @return iso_alarm_status_t The corresponding vibration severity zone.
 */
iso_alarm_status_t iso10816_check(float rms_mm_s, const iso_config_t *config);

/**
 * @brief Checks the vibration RMS value against ISO 20816 standard.
 * 
 * @param rms_mm_s The vibration velocity RMS value in mm/s.
 * @param config Pointer to the ISO configuration struct.
 * @return iso_alarm_status_t The corresponding vibration severity zone.
 */
iso_alarm_status_t iso20816_check(float rms_mm_s, const iso_config_t *config);

/**
 * @brief Converts an ISO alarm status enum to a human-readable string.
 */
const char* iso_status_to_string(iso_alarm_status_t status);

#ifdef __cplusplus
}
#endif

#endif // ISO_CHECK_H