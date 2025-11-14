/*
 * i2c.h
 *
 *  Created on: Nov 13, 2025
 *      Author: aaron
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

// Initialize I2C1 on PB6/PB7 (standard mode 100 kHz)
void i2c1_init(uint32_t pclk1_hz, uint32_t i2c_speed_hz);

/**
 * Reads a 16-bit word from an I2C device using:
 *  START → SA+W → command → RESTART → SA+R → LSB → MSB → STOP
 *
 * @param addr7 7-bit address (0x5A for MLX90614)
 * @param cmd   register/command byte
 * @param value pointer to store the 16-bit word
 * @return 0 on success, negative on timeout/error
 */
int i2c1_read_word(uint8_t addr7, uint8_t cmd, uint16_t *value);

#endif /* I2C_H_ */
