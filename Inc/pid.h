/*
 * pid.h
 *
 *  Created on: Nov 12, 2025
 *      Author: aaron
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct {
    // Tunables
    float kp;
    float ki;
    float kd;
    float dt_s;

    // Output limits
    float out_min;
    float out_max;

    // State
    float integrator;
    float prev_err;
    uint8_t initialized;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float dt_s, float out_min, float out_max);

float pid_step(pid_t *pid, float setpoint, float measurement);

#endif /* PID_H_ */
