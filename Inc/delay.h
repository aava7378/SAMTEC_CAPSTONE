/*
 * delay.h
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

// delay.h
#pragma once
#include "stm32f1xx.h"

void systick_init(uint32_t sysclk_hz);
void delay_ms(uint32_t ms);


#endif /* INC_DELAY_H_ */
