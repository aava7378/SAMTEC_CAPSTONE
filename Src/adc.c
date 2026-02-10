/*
 * adc.c
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */
//#include "adc.h"
//
//void adc1_init_single(uint8_t channel)
//{
//    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
//
//    // Analog mode on PAx for selected channel
//    if (channel <= 7) {
//        uint32_t shift = (channel & 7U)*4U;
//        GPIOA->CRL &= ~(0xFU << shift);  // MODE=00, CNF=00 (analog)
//    }
//
//    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
//
//    // power up sequence
//    ADC1->CR1 = 0;
//    ADC1->CR2 = ADC_CR2_ADON;
//    for (volatile int i=0; i<10000; i++) __asm volatile ("nop");
//    ADC1->CR2 |= ADC_CR2_ADON;
//
//    // calibration
//    ADC1->CR2 |= ADC_CR2_RSTCAL; while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
//    ADC1->CR2 |= ADC_CR2_CAL;    while (ADC1->CR2 & ADC_CR2_CAL)   {}
//
//    if (channel <= 9) {
//        uint32_t s = channel*3U;
//        ADC1->SMPR2 &= ~(7U << s);
//        ADC1->SMPR2 |=  (7U << s);
//    }
//}
//
//uint16_t adc1_read_single(uint8_t channel)
//{
//    ADC1->SQR1 &= ~ADC_SQR1_L;
//    ADC1->SQR3  = channel & 0x1FU;
//
//    ADC1->CR2 |= ADC_CR2_ADON;
//    while (!(ADC1->SR & ADC_SR_EOC)) {}
//    return (uint16_t)ADC1->DR;
//}
//
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
//
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

#include "adc.h"

static void adc_enable_gpio_analog(GPIO_TypeDef *gpio, uint8_t pin)
{
    // pin 0..7 => CRL, pin 8..15 => CRH
    if (pin <= 7) {
        uint32_t shift = pin * 4U;
        gpio->CRL &= ~(0xFU << shift);   // MODE=00, CNF=00 => analog
    } else {
        uint32_t shift = (pin - 8U) * 4U;
        gpio->CRH &= ~(0xFU << shift);
    }
}

static void adc_set_sample_time_max(ADC_TypeDef *adc, uint8_t channel)
{
    // 239.5 cycles = 0b111
    if (channel <= 9U) {
        uint32_t s = channel * 3U;
        adc->SMPR2 &= ~(7U << s);
        adc->SMPR2 |=  (7U << s);
    } else if (channel <= 17U) {
        uint32_t s = (channel - 10U) * 3U;
        adc->SMPR1 &= ~(7U << s);
        adc->SMPR1 |=  (7U << s);
    }
}

static void adc_common_init(ADC_TypeDef *adc)
{
    // ADC prescaler: PCLK2 / 6 (72 MHz / 6 = 12 MHz)
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    adc->CR1 = 0;

    // Use software start as trigger (STM32F1 typical pattern)
    // EXTSEL = 111 (SWSTART), EXTTRIG = 1
    adc->CR2 = 0;
    adc->CR2 |= ADC_CR2_EXTTRIG;
    adc->CR2 |= ADC_CR2_EXTSEL; // EXTSEL bits all 1s => SWSTART
    adc->CR2 |= ADC_CR2_ADON;

    for (volatile int i=0; i<10000; i++) __asm volatile ("nop");

    // Calibration
    adc->CR2 |= ADC_CR2_RSTCAL; while (adc->CR2 & ADC_CR2_RSTCAL) {}
    adc->CR2 |= ADC_CR2_CAL;    while (adc->CR2 & ADC_CR2_CAL)   {}
}

// ---------- Public helper APIs ----------
void adc1_enable_gpio_analog(uint8_t channel)
{
    // channels 0..7 => PA0..PA7
    if (channel <= 7U) adc_enable_gpio_analog(GPIOA, channel);
    // channels 8..9 => PB0..PB1
    else if (channel == 8U) adc_enable_gpio_analog(GPIOB, 0);
    else if (channel == 9U) adc_enable_gpio_analog(GPIOB, 1);
    // channels 10..15 => PC0..PC5
    else if (channel >= 10U && channel <= 15U) adc_enable_gpio_analog(GPIOC, channel - 10U);
}

void adc2_enable_gpio_analog(uint8_t channel)
{
    // same pin mapping as ADC1
    if (channel <= 7U) adc_enable_gpio_analog(GPIOA, channel);
    else if (channel == 8U) adc_enable_gpio_analog(GPIOB, 0);
    else if (channel == 9U) adc_enable_gpio_analog(GPIOB, 1);
    else if (channel >= 10U && channel <= 15U) adc_enable_gpio_analog(GPIOC, channel - 10U);
}

void adc1_set_sample_time_max(uint8_t channel) { adc_set_sample_time_max(ADC1, channel); }
void adc2_set_sample_time_max(uint8_t channel) { adc_set_sample_time_max(ADC2, channel); }

// ---------- ADC1 ----------
void adc1_init_single(uint8_t channel)
{
    // Enable clocks: GPIOA/B/C (safe) + ADC1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN;

    // Put the requested channel pin in analog mode + sample time
    adc1_enable_gpio_analog(channel);
    adc_common_init(ADC1);
    adc1_set_sample_time_max(channel);
}

uint16_t adc1_read_single(uint8_t channel)
{
    // 1 conversion
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3  = channel & 0x1FU;

    // clear EOC
    ADC1->SR &= ~ADC_SR_EOC;

    // start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC1->SR & ADC_SR_EOC)) {}
    return (uint16_t)ADC1->DR;
}

// ---------- ADC2 ----------
void adc2_init_single(uint8_t channel)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC2EN;

    adc2_enable_gpio_analog(channel);
    adc_common_init(ADC2);
    adc2_set_sample_time_max(channel);
}

uint16_t adc2_read_single(uint8_t channel)
{
    ADC2->SQR1 &= ~ADC_SQR1_L;
    ADC2->SQR3  = channel & 0x1FU;

    ADC2->SR &= ~ADC_SR_EOC;
    ADC2->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC2->SR & ADC_SR_EOC)) {}
    return (uint16_t)ADC2->DR;
}



