/*
 * adc.c
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */
#include "adc.h"

void adc1_init_single(uint8_t channel)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    // Analog mode on PAx for selected channel
    if (channel <= 7) {
        uint32_t shift = (channel & 7U)*4U;
        GPIOA->CRL &= ~(0xFU << shift);  // MODE=00, CNF=00 (analog)
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    // power up sequence
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;
    for (volatile int i=0; i<10000; i++) __asm volatile ("nop");
    ADC1->CR2 |= ADC_CR2_ADON;

    // calibration
    ADC1->CR2 |= ADC_CR2_RSTCAL; while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
    ADC1->CR2 |= ADC_CR2_CAL;    while (ADC1->CR2 & ADC_CR2_CAL)   {}

    if (channel <= 9) {
        uint32_t s = channel*3U;
        ADC1->SMPR2 &= ~(7U << s);
        ADC1->SMPR2 |=  (7U << s);
    }
}

uint16_t adc1_read_single(uint8_t channel)
{
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3  = channel & 0x1FU;

    ADC1->CR2 |= ADC_CR2_ADON;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    return (uint16_t)ADC1->DR;
}

//void adc2_init_single(uint8_t channel)
//{
//    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC2EN;
//
//    if (channel <= 7)
//    {
//        uint32_t shift = (channel & 7U)*4U;
//        GPIOA->CRL &= ~(0xFU << shift);
//    }
//
//    RCC -> CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
//
//    ADC2 -> CR1 = 0;
//    ADC2 -> CR2 = ADC_CR2_ADON;
//    for (volatile int i=0; i<10000; i++) __asm volatile ("nop");
//    ADC2->CR2 |= ADC_CR2_ADON;
//
//    ADC2->CR2 |= ADC_CR2_RSTCAL; while (ADC2->CR2 & ADC_CR2_RSTCAL) {}
//    ADC2->CR2 |= ADC_CR2_CAL;    while (ADC2->CR2 & ADC_CR2_CAL)   {}
//
//    if (channel <= 9) {
//        uint32_t s = channel*3U;
//        ADC2 -> SMPR2 &= ~(7U << s);
//        ADC2 -> SMPR2 |=  (7U << s);
//    }
//}

//uint16_t adc2_read_single(uint8_t channel)
//{
//	ADC2->SQR1 &= ~ADC_SQR1_L;
//	ADC2->SQR3  = channel & 0x1FU;
//
//	ADC2->CR2 |= ADC_CR2_ADON;
//
//	while (!(ADC2->SR & ADC_SR_EOC)) {}
//
//	return (uint16_t)ADC2->DR;
//}

// adc.c
//uint16_t adc2_read_single(uint8_t channel)
//{
//	// 1. Configure the sequence for a single conversion
//	ADC2->SQR1 &= ~ADC_SQR1_L;      // L=0 for a sequence length of 1
//	ADC2->SQR3  = channel & 0x1FU;  // Set the first (and only) channel
//
//	// 2. Clear the EOC flag (Read SR before DR, or write 0 to clear in some MCUs,
//	//    but simply clearing it is safer practice before starting)
//    ADC2->SR &= ~ADC_SR_EOC;
//
//	// 3. Start the conversion using the Software Start bit
//	ADC2->CR2 |= ADC_CR2_SWSTART; // *** THIS IS THE CRITICAL CHANGE ***
//
//	// 4. Wait for End-Of-Conversion (EOC)
//	while (!(ADC2->SR & ADC_SR_EOC)) {}
//
//	// 5. Read and return the result (this implicitly clears the EOC flag)
//	return (uint16_t)ADC2->DR;
//}

void adc2_init_single(uint8_t channel)
{
    // Enable GPIOA and ADC2 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC2EN;

    // Put PAx into analog mode if channel is on GPIOA 0..7
    if (channel <= 7U)
    {
        uint32_t shift = (channel & 7U) * 4U;
        GPIOA->CRL &= ~(0xFU << shift);   // MODE=00, CNF=00 => analog
    }

    // Set ADC prescaler: PCLK2 / 6 (72 MHz / 6 = 12 MHz)
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    // Power-up sequence (like ADC1)
    ADC2->CR1 = 0;
    ADC2->CR2 = 0;

    // Turn ADC2 on (first ADON)
    ADC2->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 10000; i++) __asm volatile ("nop");

    // Calibration
    ADC2->CR2 |= ADC_CR2_RSTCAL;
    while (ADC2->CR2 & ADC_CR2_RSTCAL) { }

    ADC2->CR2 |= ADC_CR2_CAL;
    while (ADC2->CR2 & ADC_CR2_CAL) { }

    // Sample time for this channel: longest (for best accuracy/noise)
    if (channel <= 9U) {
        uint32_t s = channel * 3U;
        ADC2->SMPR2 &= ~(7U << s);
        ADC2->SMPR2 |=  (7U << s);  // 239.5 cycles
    }

    // Configure regular sequence: 1 conversion, this channel
    ADC2->SQR1 &= ~ADC_SQR1_L;              // L = 0 → 1 conversion
    ADC2->SQR3  = channel & 0x1FU;          // first (and only) channel

    // Enable continuous conversion mode
    ADC2->CR2 |= ADC_CR2_CONT;

    // Start continuous conversions:
    // Second write to ADON starts conversion when CONT=1
    ADC2->CR2 |= ADC_CR2_ADON;
}

// Continuous regular mode: just read the latest conversion
uint16_t adc2_read_single(uint8_t channel)
{
    (void)channel;  // channel is fixed by adc2_init_single

    // Wait for at least one conversion (EOC set)
    while (!(ADC2->SR & ADC_SR_EOC)) { }

    // Read and return the data (reading DR clears EOC)
    return (uint16_t)ADC2->DR;
}


