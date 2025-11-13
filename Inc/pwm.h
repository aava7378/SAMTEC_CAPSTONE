/*
 * pwm.h
 *
 *  Created on: Nov 12, 2025
 *      Author: aaron
 */

#ifndef PWM_H_
#define PWM_H_

#include "stm32f1xx.h"
#include <stdint.h>

void pwm_tim1_ch1_init(uint32_t sysclk_hz, uint32_t pwm_hz);
void pwm_tim1_set(float duty);

#endif /* PWM_H_ */
