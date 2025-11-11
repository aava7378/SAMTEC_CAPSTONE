/*
 * adc.h
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */

#ifndef ADC_H_
#define ADC_H_

#pragma once
#include "stm32f1xx.h"
void     adc1_init_single(uint8_t channel);  // e.g., 1 for PA1
uint16_t adc1_read_single(uint8_t channel);  // blocking single conversion

#endif /* ADC_H_ */
