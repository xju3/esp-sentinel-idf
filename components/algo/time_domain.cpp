#include "vib_math.h"
#include <math.h>
#include <stddef.h>

namespace Algo
{

float calc_rms(const float *data, int len)
{
    if (len <= 0 || data == NULL)
        return 0.0f;

    float sum_sq = 0.0f;
    for (int i = 0; i < len; i++)
    {
        // Remove 1g gravity (assuming Z includes gravity)
        float ac = data[i] - 1.0f;
        sum_sq += ac * ac;
    }
    return sqrtf(sum_sq / (float)len);
}

// Simple mock status mapping; kept for backward compatibility
const char *get_iso_status(float rms)
{
    if (rms > 0.45f)
        return "DANGER";
    if (rms > 0.30f)
        return "WARNING";
    return "NORMAL";
}

} // namespace Algo

