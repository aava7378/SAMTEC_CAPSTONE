/*
 * rcc.h
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#pragma once
#include "stm32f1xx.h"
void clock_init(void);   // picks HSE@72 or HSI@64 based on USE_HSE

#endif /* INC_RCC_H_ */
