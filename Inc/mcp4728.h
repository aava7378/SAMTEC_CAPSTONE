/*
 * mcp4728.h
 *
 *  Created on: Jan 14, 2026
 *      Author: aaron
 */

#ifndef MCP4728_H_
#define MCP4728_H_

#include <stdint.h>

typedef enum {
    MCP4728_CH_A = 0,
    MCP4728_CH_B = 1,
    MCP4728_CH_C = 2,
    MCP4728_CH_D = 3
} mcp4728_channel_t;

typedef struct {
    uint8_t i2c_addr;   // usually 0x60 or 0x64
    float   vdd_v;      // 3.3 if powered from 3.3V
} mcp4728_t;

void mcp4728_init(mcp4728_t *d, uint8_t i2c_addr, float vdd_v);

// write raw 12-bit code (0..4095) to a channel
int  mcp4728_write_code(mcp4728_t *d, mcp4728_channel_t ch, uint16_t code);

// write voltage (0..VDD) to a channel
int  mcp4728_write_voltage(mcp4728_t *d, mcp4728_channel_t ch, float volts);

#endif /* MCP4728_H_ */
