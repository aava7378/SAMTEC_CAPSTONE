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
#include <stdint.h>

void     adc1_init_single(uint8_t channel);
uint16_t adc1_read_single(uint8_t channel);

void     adc2_init_single(uint8_t channel);
uint16_t adc2_read_single(uint8_t channel);

void     adc1_enable_gpio_analog(uint8_t channel);
void     adc2_enable_gpio_analog(uint8_t channel);

void     adc1_set_sample_time_max(uint8_t channel);
void     adc2_set_sample_time_max(uint8_t channel);

#endif /* ADC_H_ */
