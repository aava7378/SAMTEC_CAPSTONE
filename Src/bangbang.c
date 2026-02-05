/*
 * bangbang.c
 *
 *  Created on: Jan 25, 2026
 *      Author: aaron
 */

#include "bangbang.h"

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void bangbang_init(bangbang_t *bb, float hyst_C, float out_min, float out_max)
{
    bb->hyst_C = (hyst_C < 0.0f) ? -hyst_C : hyst_C;

    if (out_min <= out_max) {
        bb->out_min = out_min;
        bb->out_max = out_max;
    } else {
        bb->out_min = out_max;
        bb->out_max = out_min;
    }

    bb->u = bb->out_min;
    bb->initialized = 0;
}

float bangbang_step_heatonly(bangbang_t *bb, float setpoint, float measurement)
{
    // On first call, start OFF
    if (!bb->initialized) {
        bb->u = bb->out_min;
        bb->initialized = 1;
    }

    float on_thresh  = setpoint - bb->hyst_C;
    float off_thresh = setpoint + bb->hyst_C;

    // Heat-only logic:
    // - If too cold -> ON (max)
    // - If too hot  -> OFF (min)
    // - In between -> hold last output
    if (measurement <= on_thresh) {
        bb->u = bb->out_max;
    } else if (measurement >= off_thresh) {
        bb->u = bb->out_min;
    } // else: keep bb->u

    return clampf(bb->u, bb->out_min, bb->out_max);
}


