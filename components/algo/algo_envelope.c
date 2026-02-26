// features/vib_envelope.c

#include "algo_envelope.h"

void vib_envelope_init(vib_envelope_state_t *st)
{
    if (!st) return;
    st->reserved = 0;
}

vib_algo_err_t vib_envelope_window(vib_envelope_state_t *st,
                                  const float *x, const float *y, const float *z,
                                  int N, float fs_hz,
                                  vib_envelope_out_t *out)
{
    (void)st; (void)x; (void)y; (void)z; (void)N; (void)fs_hz;
    if (!out) return VIB_ALGO_BAD_ARGS;
    out->reserved = 0.0f;
    return VIB_ALGO_UNSUPPORTED;
}
