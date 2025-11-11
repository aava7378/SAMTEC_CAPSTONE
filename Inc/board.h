/*
 * board.h
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#pragma once
#include "stm32f1xx.h"

#define USE_HSE 1
#if USE_HSE
  #define SYSCLK_FREQ_HZ 72000000UL
#else
  #define SYSCLK_FREQ_HZ 64000000UL
#endif

#define LED_GPIO  GPIOA
#define LED_PIN   5
#define ENABLE_GPIOA()  (RCC->APB2ENR |= RCC_APB2ENR_IOPAEN)


#endif /* INC_BOARD_H_ */
