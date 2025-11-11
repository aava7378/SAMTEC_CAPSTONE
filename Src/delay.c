#include "delay.h"

static volatile uint32_t tick_ms = 0;
void SysTick_Handler(void) { tick_ms++; }

void systick_init(uint32_t sysclk_hz)
{
    SysTick->LOAD = (sysclk_hz/1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(uint32_t ms)
{
    uint32_t t0 = tick_ms;
    while ((tick_ms - t0) < ms) { __NOP(); }
}
