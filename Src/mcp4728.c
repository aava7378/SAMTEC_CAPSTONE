/*
 * mcp4728.c
 *
 *  Created on: Jan 14, 2026
 *      Author: aaron
 */

#include "mcp4728.h"
#include "i2c.h"

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void mcp4728_init(mcp4728_t *d, uint8_t i2c_addr, float vdd_v)
{
    d->i2c_addr = i2c_addr;
    d->vdd_v    = vdd_v;
}

/*
 * Standard MCP4728 “write DAC input register” format used by many drivers:
 *  [0] = 0x40 | (ch << 1) | UDAC(0)
 *  [1] = (VREF=VDD, PD=normal, GAIN=1x, D11..D8)
 *  [2] = (D7..D0)
 *
 * If your device doesn’t ACK, the command byte may differ on your variant;
 * we can adjust quickly based on what it does on the bus.
 */
int mcp4728_write_code(mcp4728_t *d, mcp4728_channel_t ch, uint16_t code)
{
    if (code > 4095u) code = 4095u;

    uint8_t cmd = (uint8_t)(0x40u | (((uint8_t)ch & 0x03u) << 1)); // UDAC=0

    // VREF=VDD, PD=00 normal, GAIN=1x, then upper data nibble
    uint8_t b1 = (uint8_t)((code >> 8) & 0x0Fu);
    uint8_t b2 = (uint8_t)(code & 0xFFu);

    uint8_t tx[3] = { cmd, b1, b2 };
    return i2c1_write_bytes(d->i2c_addr, tx, 3);
}

int mcp4728_write_voltage(mcp4728_t *d, mcp4728_channel_t ch, float volts)
{
    volts = clampf(volts, 0.0f, d->vdd_v);

    float code_f = (volts / d->vdd_v) * 4095.0f;
    uint16_t code = (uint16_t)(code_f + 0.5f);

    return mcp4728_write_code(d, ch, code);
}


