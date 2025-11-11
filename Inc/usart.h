/*
 * usart.h
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */

#ifndef USART_H_
#define USART_H_

#pragma once
#include "stm32f1xx.h"

void usart2_init(uint32_t pclk1_hz, uint32_t baud);
void usart2_write_char(char c);
void usart2_write_str(const char *s);


#endif /* USART_H_ */
