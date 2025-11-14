/*
 * motor.c
 *
 *  Created on: Nov 12, 2025
 *      Author: aaron
 */
#include "motor.h"

// PB5 = D4 = IN1
// PB4 = D5 = IN2
#define IN1_PORT GPIOB
#define IN1_PIN  5U
#define IN2_PORT GPIOB
#define IN2_PIN  4U


/**
 * @brief  Configure a given GPIO pin as a push-pull output.
 *
 * This function turns on the clock for the GPIO port (A, B, or C)
 * and sets the selected pin to "general-purpose output, push-pull" mode
 * at 2 MHz speed.
 *
 * @param[in] p    Pointer to GPIO port base (e.g., GPIOA, GPIOB, GPIOC)
 * @param[in] pin  Pin number (0–15) on that port
 */
static void gpio_out(GPIO_TypeDef* p, uint8_t pin)
{
    if (p == GPIOA)
    {
    	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    }

    if (p == GPIOB)
    {
    	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    }

    if (p == GPIOC)
    {
    	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    }

    volatile uint32_t* cr = (pin < 8) ? &p->CRL : &p->CRH;
    uint32_t shift = (pin & 7U) * 4U;
    uint32_t v = *cr;
    v &= ~(0xFU << shift);
    v |=  (0x2U << shift); // MODE=10 (2MHz), CNF=00 (PP)
    *cr = v;
}

/**
 * @brief  Initialize the GPIO pins used for motor direction control.
 *
 * This function prepares the STM32 pins connected to the L298N motor
 * driver’s direction inputs (`IN1` and `IN2`). It sets both pins as
 * push-pull outputs and drives them LOW to place the Peltier
 * in a no current flow state at startup
 *
 * @details
 * - `IN1` and `IN2` are logic inputs that control the direction of
 *   current through the Peltier module.
 * - The function calls `gpio_out()` to configure these pins as outputs.
 * - Then it clears both pins (writes 0) to make sure the bridge starts
 *   disabled (safe, neutral state).
 */
void motor_io_init(void)
{
    gpio_out(IN1_PORT, IN1_PIN);
    gpio_out(IN2_PORT, IN2_PIN);

    // default coast
    IN1_PORT->BRR = (1U << IN1_PIN);
    IN2_PORT->BRR = (1U << IN2_PIN);
}

/**
 * @brief  Set the Peltier module direction.
 *
 * This function drives the two direction pins (`IN1`, `IN2`) connected to the
 * L298N motor driver to control the current direction through the load Peltier module.
 *
 * Depending on the sign of the `dir` argument, it selects one of three states:
 * - HEAT (`dir > 0`):  IN1 = HIGH, IN2 = LOW  → current flows one way
 * - COOL (`dir < 0`):  IN1 = LOW,  IN2 = HIGH → current flows the opposite way
 * - COAST (`dir == 0`): both LOW  → output off, no current flow
 */
void motor_set_dir(int dir)
{
    if (dir < 0)
    {
        // heat
        IN1_PORT->BSRR = (1U << IN1_PIN);
        IN2_PORT->BRR  = (1U << IN2_PIN);
    }
    else if (dir > 0)
    {
    	// cool
        IN1_PORT->BRR  = (1U << IN1_PIN);
        IN2_PORT->BSRR = (1U << IN2_PIN);
    }
    else
    {
    	// standby
        IN1_PORT->BRR  = (1U << IN1_PIN);
        IN2_PORT->BRR  = (1U << IN2_PIN);
    }
}


