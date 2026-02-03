/*
 * max1968_ctl.h
 *
 *  Created on: Jan 14, 2026
 *      Author: aaron
 */

#ifndef MAX1968_CTL_H_
#define MAX1968_CTL_H_

#include "mcp4728.h"

typedef struct {
    mcp4728_t *dac;
    mcp4728_channel_t ch;

    float rsense_ohms;   // your sense resistor
    float vref_v;        // MAX1968 REF ~ 1.50V
    float i_limit_a;     // clamp
} max1968_ctl_t;

void  max1968_ctl_init(max1968_ctl_t *m,
                       mcp4728_t *dac, mcp4728_channel_t ch,
                       float rsense_ohms, float i_limit_a);

// Convert desired ITEC (A) to VCTLI (V)
float max1968_i_to_vctli(const max1968_ctl_t *m, float i_cmd_a);

// Push command to DAC -> CTLI
int   max1968_set_itec_cmd_a(max1968_ctl_t *m, float i_cmd_a);

#endif /* MAX1968_CTL_H_ */
