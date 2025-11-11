/*
 * adc.c
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */
#include "adc.h"

void adc1_init_single(uint8_t channel)
{
    // GPIOA & ADC1 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    // Analog mode on PAx for selected channel
    if (channel <= 7) {
        // channels 0..7 are on CRL pins A0..A7
        uint32_t shift = (channel & 7U)*4U;
        GPIOA->CRL &= ~(0xFU << shift);  // MODE=00, CNF=00 (analog)
    } else {
        // not needed for this demo
    }

    // ADC clock ≤14 MHz: PCLK2/6 with PCLK2=72 MHz → 12 MHz
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    // Power-up sequence
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;              // wake
    for (volatile int i=0; i<10000; i++) __asm volatile ("nop");
    ADC1->CR2 |= ADC_CR2_ADON;             // enable

    // Calibration
    ADC1->CR2 |= ADC_CR2_RSTCAL; while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
    ADC1->CR2 |= ADC_CR2_CAL;    while (ADC1->CR2 & ADC_CR2_CAL)   {}

    // Long sample time for accuracy: 239.5 cycles
    if (channel <= 9) {
        uint32_t s = channel*3U;
        ADC1->SMPR2 &= ~(7U << s);
        ADC1->SMPR2 |=  (7U << s);
    }
}

uint16_t adc1_read_single(uint8_t channel)
{
    // Sequence length = 1
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3  = channel & 0x1FU;

    // Start conversion
    ADC1->CR2 |= ADC_CR2_ADON;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    return (uint16_t)ADC1->DR;
}


