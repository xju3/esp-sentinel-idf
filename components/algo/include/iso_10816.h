#ifndef VIB_ALGO_ISO_10816_H
#define VIB_ALGO_ISO_10816_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Machine classes (ISO 10816-3 legacy thresholds)
typedef enum
{
    ISO_CLASS_I = 0,
    ISO_CLASS_II = 1,
    ISO_CLASS_III = 2,
    ISO_CLASS_IV = 3
} iso_machine_class_t;

typedef enum
{
    ISO_ZONE_A = 0,
    ISO_ZONE_B = 1,
    ISO_ZONE_C = 2,
    ISO_ZONE_D = 3
} iso_severity_zone_t;

iso_severity_zone_t get_iso_status(iso_machine_class_t machine_cls, float rms_velocity);
const char *get_zone_name(iso_severity_zone_t zone);

#ifdef __cplusplus
}
#endif

#endif // VIB_ALGO_ISO_10816_H
