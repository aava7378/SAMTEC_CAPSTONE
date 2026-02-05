/*
 * max1968_shdn.c
 *
 *  Created on: Jan 14, 2026
 *      Author: aaron
 */

#include "max1968_shdn.h"
#include "stm32f1xx.h"

#define SHDN_PORT GPIOA
#define SHDN_PIN  9u   // PA9 = Arduino D8

void max1968_shdn_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // PA9 as 2 MHz push-pull output
    // PA9 is in CRH (pins 8–15)
    uint32_t shift = (9u - 8u) * 4u;
    SHDN_PORT->CRH &= ~(0xFu << shift);
    SHDN_PORT->CRH |=  (0x2u << shift);   // MODE=10, CNF=00

    // Default OFF
    max1968_shdn_set(0);
}

void max1968_shdn_set(uint8_t en)
{
    if (en) {
        SHDN_PORT->BSRR = (1u << SHDN_PIN);
    } else {
        SHDN_PORT->BSRR = (1u << (SHDN_PIN + 16u));
    }
}


