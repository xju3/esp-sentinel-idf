#ifndef ALGO_WELFORD_H
#define ALGO_WELFORD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t n;
        double mean;
        double m2;
    } vib_welford_1d_t;

    typedef struct
    {
        vib_welford_1d_t x;
        vib_welford_1d_t y;
        vib_welford_1d_t z;
    } vib_welford_3d_t;

    static inline void vib_welford_1d_init(vib_welford_1d_t *s)
    {
        if (!s)
            return;
        s->n = 0;
        s->mean = 0.0;
        s->m2 = 0.0;
    }

    static inline void vib_welford_3d_init(vib_welford_3d_t *s)
    {
        if (!s)
            return;
        vib_welford_1d_init(&s->x);
        vib_welford_1d_init(&s->y);
        vib_welford_1d_init(&s->z);
    }

    static inline void vib_welford_1d_update(vib_welford_1d_t *s, float v)
    {
        if (!s)
            return;
        s->n++;
        double dv = (double)v - s->mean;
        s->mean += dv / (double)s->n;
        s->m2 += dv * ((double)v - s->mean);
    }

    static inline void vib_welford_3d_update(vib_welford_3d_t *s, float x, float y, float z)
    {
        if (!s)
            return;
        vib_welford_1d_update(&s->x, x);
        vib_welford_1d_update(&s->y, y);
        vib_welford_1d_update(&s->z, z);
    }

#ifdef __cplusplus
}
#endif

#endif // ALGO_WELFORD_H