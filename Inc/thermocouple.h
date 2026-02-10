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

#define TC_VREF_mV        3300
#define TC_ADC_BITS       12
#define AD8495_SLOPE_uV_C 5000
#define AD8495_OFFSET_mV  1250

void     tc_init(void);
int32_t  tc_read_c_x10_ch(uint8_t adc1_channel);

int32_t  tc_read_c_x10(void);


#endif /* THERMOCOUPLE_H_ */
