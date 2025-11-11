/*
 * gpio.h
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#pragma once
#include "stm32f1xx.h"

void gpio_init_output(GPIO_TypeDef *port, uint8_t pin);
void gpio_toggle(GPIO_TypeDef *port, uint8_t pin);


#endif /* INC_GPIO_H_ */
