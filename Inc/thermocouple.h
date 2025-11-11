/*
 * thermocouple.h
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */

#ifndef THERMOCOUPLE_H_
#define THERMOCOUPLE_H_

#pragma once
#include <stdint.h>

// Adjust once you calibrate your board:
// AD8495 typically ~5.0 mV/°C; output ≈ 1.25 V at 0 °C (depends on wiring/cold-junction).
#define TC_VREF_mV        3300    // VDDA
#define TC_ADC_BITS       12
#define AD8495_SLOPE_uV_C 5000    // 5.000 mV/°C
#define AD8495_OFFSET_mV  1250    // Vout at 0 °C (tune!)

void     tc_init(void);           // init ADC1 on PA1 (ADC1_IN1)
int32_t  tc_read_c_x10(void);     // temperature in 0.1 °C units


#endif /* THERMOCOUPLE_H_ */
