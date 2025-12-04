/*
 * motor.h
 *
 *  Created on: Nov 12, 2025
 *      Author: aaron
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f1xx.h"
#include <stdint.h>

void motor_io_init(void);
void motor_set_dir(int dir);

#endif /* MOTOR_H_ */
