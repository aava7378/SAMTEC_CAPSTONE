#include "gpio.h"

static inline volatile uint32_t* cr_reg(GPIO_TypeDef *port, uint8_t pin)
{
    return (pin < 8) ? &port->CRL : &port->CRH;
}

void gpio_init_output(GPIO_TypeDef *port, uint8_t pin)
{
    volatile uint32_t *cr = cr_reg(port, pin);
    uint32_t shift = (pin & 7U) * 4U;
    uint32_t v = *cr;

    v &= ~(0xFU << shift);
    v |=  (0x2U << shift);

    *cr = v;
}

void gpio_toggle(GPIO_TypeDef *port, uint8_t pin)
{
    port->ODR ^= (1U << pin);
}
