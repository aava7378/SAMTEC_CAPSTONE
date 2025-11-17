/*
 * pwm.c
 *
 *  Created on: Nov 12, 2025
 *      Author: aaron
 */
#include "pwm.h"

// TIM1_CH1 on PA8, 20 kHz
static uint16_t s_arr = 3599; // with PSC=0, ARR=3599 gives 20 kHz at 72 MHz

/**
 * @brief  Initialize TIM1 Channel 1 for hardware PWM output.
 *
 * This function configures Timer 1, Channel 1 to generate a hardware-based
 * PWM signal on d7. The PWM frequency and duty-cycle resolution
 * are set according to the system clock and desired output frequency.
 *
 * The resulting PWM output drive a Peltier module,
 * via the L298N enable pin (ENB) for speed/power control.
 */
void pwm_tim1_ch1_init(uint32_t sysclk_hz, uint32_t pwm_hz)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_TIM1EN;

    // MODE=11 (50MHz), CNF=10 (AF PP) => 0b1011 = 0xB
    GPIOA->CRH &= ~(0xFU << 0);
    GPIOA->CRH |=  (0xBU << 0);

    // PSC=0 for maximum resolution
    uint32_t ticks = (sysclk_hz / pwm_hz);
    uint32_t psc   = 0;
    uint32_t arr   = (ticks / (psc + 1U)) - 1U;
    if (arr > 0xFFFFU) {
        psc = (ticks / 0x10000U);
        if (psc > 0xFFFFU) psc = 0xFFFFU;
        arr = (ticks / (psc + 1U)) - 1U;
        if (arr > 0xFFFFU) arr = 0xFFFFU;
    }

    TIM1->PSC = (uint16_t)psc;
    TIM1->ARR = (uint16_t)arr;
    s_arr     = (uint16_t)arr;

    // CH1 PWM mode 1, preload enable
    TIM1->CCR1  = 0;
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S);
    TIM1->CCMR1 |=  (6U << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM1->CCMR1 |=  TIM_CCMR1_OC1PE;            // preload

    // Enable output on CH1
    TIM1->CCER &= ~TIM_CCER_CC1P;   // active high
    TIM1->CCER |=  TIM_CCER_CC1E;

    // Main Output Enable (advanced timer)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Auto-reload preload
    TIM1->CR1 |= TIM_CR1_ARPE;

    // Start
    TIM1->EGR  |= TIM_EGR_UG;
    TIM1->CR1  |= TIM_CR1_CEN;
}

/**
 * @brief  Set the PWM duty cycle for TIM1 Channel 1.
 *
 * This function updates the PWM output level by writing a new compare
 * value (CCR1) to Timer 1, Channel 1, which controls how long the
 * output stays HIGH in each PWM period.
 *
 * The duty cycle is a floating-point value between 0.0 and 1.0:
 * - `0.0f` → 0% ON time (always off)
 * - `0.5f` → 50% ON time
 * - `1.0f` → 100% ON time (always on)
 *
 * Any value outside this range is clamped.
 */
void pwm_tim1_set(float duty)
{
    if (duty < 0.0f)
    {
    	duty = 0.0f;
    }

    if (duty > 1.0f)
    {
    	duty = 1.0f;
    }

    uint32_t c = (uint32_t)((float)(s_arr + 1U) * duty + 0.5f);

    if (c > s_arr)
    {
    	c = s_arr;
    }

    TIM1->CCR1 = (uint16_t)c;
}


