#include "rcc.h"
#include "board.h"

static void clock_init_hse_72mhz(void)
{
    // HSE on
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    // AHB=1, APB2=1, APB1=2, PLL = HSE * 9 = 72 MHz
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 |
                   RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);

    RCC->CFGR |=  RCC_CFGR_HPRE_DIV1
                | RCC_CFGR_PPRE1_DIV2
                | RCC_CFGR_PPRE2_DIV1
                | RCC_CFGR_PLLMULL9;

    RCC->CFGR |= RCC_CFGR_PLLSRC;
    RCC->CR |= RCC_CR_PLLON;

    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;

    while (((RCC->CFGR >> 2) & 0x3) != 0x2) {}
}

static void clock_init_hsi_64mhz(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);

    RCC->CFGR |=  RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PLLMULL16;

    RCC->CFGR &= ~RCC_CFGR_PLLSRC;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while (((RCC->CFGR >> 2) & 0x3) != 0x2) {}
}

void clock_init(void)
{
#if USE_HSE
    clock_init_hse_72mhz();
#else
    clock_init_hsi_64mhz();
#endif
}
