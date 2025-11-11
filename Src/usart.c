/*
 * usart.c
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */
#include "usart.h"

static uint32_t usart_brr_times16(uint32_t pclk_hz, uint32_t baud)
{
    return (pclk_hz + (baud/2U)) / baud;
}

void usart2_init(uint32_t pclk1_hz, uint32_t baud)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->CRL &= ~(0xFU << (2U*4));
    GPIOA->CRL |=  (0x2U << (2U*4));
    GPIOA->CRL &= ~(0x3U << (2U*4 + 2));
    GPIOA->CRL |=  (0x2U << (2U*4 + 2));
    GPIOA->CRL &= ~(0xFU << (3U*4));
    GPIOA->CRL |=  (0x4U << (3U*4));

    // Baud
    USART2->BRR = usart_brr_times16(pclk1_hz, baud);

    // Enable TX, RX and USART
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void usart2_write_char(char c)
{
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = (uint16_t)c;
}

void usart2_write_str(const char *s)
{
    while (*s) usart2_write_char(*s++);
}


