/*
 * bangbang.h
 *
 *  Created on: Jan 25, 2026
 *      Author: aaron
 */

#ifndef BANGBANG_H_
#define BANGBANG_H_

#include <stdint.h>

typedef struct {
    float out_min;
    float out_max;

    // Hysteresis around setpoint:
    // ON when measurement <= setpoint - hyst
    // OFF when measurement >= setpoint + hyst
    float hyst_C;

    // latched output (so we "hold last state" in the band)
    float u;
    uint8_t initialized;
} bangbang_t;

void bangbang_init(bangbang_t *bb, float hyst_C, float out_min, float out_max);

// Heat-only bang/bang with hysteresis latch
float bangbang_step_heatonly(bangbang_t *bb, float setpoint, float measurement);

#endif /* BANGBANG_H_ */
