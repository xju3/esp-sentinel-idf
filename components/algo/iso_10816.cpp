#include "iso_10816.h"

namespace Algo
{
// ISO 10816-3 legacy limits (mm/s)
static const float ISO_LIMITS[4][3] = {
    {0.71f, 1.12f, 2.80f}, // Class I
    {1.12f, 2.80f, 7.10f}, // Class II
    {1.80f, 4.50f, 11.2f}, // Class III
    {2.80f, 7.10f, 18.0f}  // Class IV
};

iso_severity_zone_t get_iso_status(iso_machine_class_t machine_cls, float rms)
{
    if (machine_cls > ISO_CLASS_IV)
        machine_cls = ISO_CLASS_II;

    if (rms < ISO_LIMITS[machine_cls][0])
        return ISO_ZONE_A;
    if (rms < ISO_LIMITS[machine_cls][1])
        return ISO_ZONE_B;
    if (rms < ISO_LIMITS[machine_cls][2])
        return ISO_ZONE_C;
    return ISO_ZONE_D;
}

const char *get_zone_name(iso_severity_zone_t zone)
{
    switch (zone)
    {
    case ISO_ZONE_A:
        return "EXCELLENT";
    case ISO_ZONE_B:
        return "GOOD";
    case ISO_ZONE_C:
        return "WARNING";
    case ISO_ZONE_D:
        return "DANGER";
    default:
        return "UNKNOWN";
    }
}
} // namespace Algo

