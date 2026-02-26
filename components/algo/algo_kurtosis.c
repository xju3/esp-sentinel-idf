// features/vib_kurtosis.c

#include "algo_kurtosis.h"

vib_algo_err_t vib_kurtosis_window(const float *x, const float *y, const float *z,
                                   int N, vib_kurtosis_out_t *out)
{
    (void)x; (void)y; (void)z; (void)N;
    if (!out) return VIB_ALGO_BAD_ARGS;
    out->kx = out->ky = out->kz = out->k3d = 0.0f;
    return VIB_ALGO_UNSUPPORTED;
}
