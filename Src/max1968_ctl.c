/*
 * max1968_ctl.c
 *
 *  Created on: Jan 14, 2026
 *      Author: aaron
 */

#include "max1968_ctl.h"

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void max1968_ctl_init(max1968_ctl_t *m, mcp4728_t *dac, mcp4728_channel_t ch, float rsense_ohms, float i_limit_a)
{
    m->dac = dac;
    m->ch = ch;
    m->rsense_ohms = rsense_ohms;
    m->vref_v = 1.50f;        // VREF nominal :contentReference[oaicite:6]{index=6}
    m->i_limit_a = i_limit_a;
}

float max1968_i_to_vctli(const max1968_ctl_t *m, float i_cmd_a)
{
    // VCTLI = VREF + (10*RSENSE)*ITEC
    float i = clampf(i_cmd_a, -m->i_limit_a, m->i_limit_a);
    return m->vref_v + (10.0f * m->rsense_ohms * i);
}

int max1968_set_itec_cmd_a(max1968_ctl_t *m, float i_cmd_a)
{
    float vctli = max1968_i_to_vctli(m, i_cmd_a);
    return mcp4728_write_voltage(m->dac, m->ch, vctli);
}

