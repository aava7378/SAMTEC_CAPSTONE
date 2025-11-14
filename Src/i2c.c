// i2c.c
#include "i2c.h"
#include "stm32f1xx.h"

#define I2C_TIMEOUT 100000U

static int wait_flag_set(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t t = I2C_TIMEOUT;
    while (((*reg) & mask) == 0U) {
        if (--t == 0U) return -1;
    }
    return 0;
}

static int wait_flag_clear(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t t = I2C_TIMEOUT;
    while (((*reg) & mask) != 0U) {
        if (--t == 0U) return -1;
    }
    return 0;
}

void i2c1_init(uint32_t pclk1_hz, uint32_t i2c_speed_hz)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;

    GPIOB->CRH &= ~((0xFU << (0U * 4U)) | (0xFU << (1U * 4U)));
    GPIOB->CRH |=  ((0xFU << (0U * 4U)) | (0xFU << (1U * 4U)));

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    uint32_t freq = pclk1_hz / 1000000U;
    I2C1->CR2 = (uint16_t)freq;

    uint32_t ccr = pclk1_hz / (2U * i2c_speed_hz);
    if (ccr < 4U) ccr = 4U;
    I2C1->CCR = (uint16_t)ccr;
    I2C1->TRISE = (uint16_t)(freq + 1U);
    I2C1->CR1 = I2C_CR1_PE;
}

int i2c1_read_word(uint8_t addr7, uint8_t cmd, uint16_t *value)
{
    if (!value) return -1;

    if (wait_flag_clear(&I2C1->SR2, I2C_SR2_BUSY) < 0) {
        return -1;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (wait_flag_set(&I2C1->SR1, I2C_SR1_SB) < 0) return -1;

    I2C1->DR = (uint32_t)(addr7 << 1);
    if (wait_flag_set(&I2C1->SR1, I2C_SR1_ADDR) < 0) return -1;
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->DR = cmd;
    if (wait_flag_set(&I2C1->SR1, I2C_SR1_TXE) < 0) return -1;
    if (wait_flag_set(&I2C1->SR1, I2C_SR1_BTF) < 0) return -1;

    I2C1->CR1 |= I2C_CR1_START;
    if (wait_flag_set(&I2C1->SR1, I2C_SR1_SB) < 0) return -1;

    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->DR = (uint32_t)((addr7 << 1) | 1U);
    if (wait_flag_set(&I2C1->SR1, I2C_SR1_ADDR) < 0) return -1;

    I2C1->CR1 &= ~I2C_CR1_ACK;
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    if (wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE) < 0) return -1;
    uint8_t low = (uint8_t)I2C1->DR;

    I2C1->CR1 |= I2C_CR1_STOP;

    if (wait_flag_set(&I2C1->SR1, I2C_SR1_RXNE) < 0) return -1;
    uint8_t high = (uint8_t)I2C1->DR;

    *value = (uint16_t)low | ((uint16_t)high << 8);
    return 0;
}
