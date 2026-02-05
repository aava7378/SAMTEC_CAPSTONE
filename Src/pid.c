///*
// * pid.c
// *
// *  Created on: Nov 12, 2025
// *      Author: aaron
// */
//#include "pid.h"
//
///**
// * @brief  helper func: Limit a floating-point value between a minimum and maximum.
// *
// * This helper function "clamps" a value so it always stays inside a given range.
// * If the input value `x` is smaller than `lo`, it returns `lo`.
// * If `x` is larger than `hi`, it returns `hi`.
// * Otherwise, it returns `x` unchanged.
// */
//static inline float clampf(float x, float lo, float hi)
//{
//    if (x < lo)
//    {
//    	return lo;
//    }
//
//    if (x > hi)
//    {
//    	return hi;
//    }
//    return x;
//}
//
///**
// * @brief  Initialize a PID controller structure with tuning parameters and limits.
// *
// * This function sets up all internal parameters for the PID controller before use.
// * It loads the proportional (`kp`), integral (`ki`), and derivative (`kd`) gains,
// * the control loop time step (`dt_s`), and the output limits (`out_min`, `out_max`).
// *
// * It also resets all internal state variables such as the integrator and
// * previous error to ensure a clean start.
// *
// * @param[in,out] pid       Pointer to the PID controller structure to initialize.
// * @param[in] kp            Proportional gain — controls reaction strength to the error.
// * @param[in] ki            Integral gain — controls correction of steady-state error.
// * @param[in] kd            Derivative gain — controls damping and response rate.
// * @param[in] dt_s          Sampling time in seconds (0.02 for 20 ms loop).
// * @param[in] out_min       Minimum allowed output value
// * @param[in] out_max       Maximum allowed output value
// */
//void pid_init(pid_t *pid, float kp, float ki, float kd, float dt_s, float out_min, float out_max)
//{
//    pid->kp = kp;
//    pid->ki = ki;
//    pid->kd = kd;
//    pid->dt_s = dt_s;
//
//    if (out_min <= out_max)
//    {
//        pid->out_min = out_min;
//        pid->out_max = out_max;
//    }
//    else
//    {
//        pid->out_min = out_max;
//        pid->out_max = out_min;
//    }
//
//    pid->integrator = 0.0f;
//    pid->prev_err = 0.0f;
//    pid->initialized = 0;
//}
//
///**
// * @brief  Perform one update step of the PID controller.
// *
// * This function calculates the new control output `u` based on the difference
// * between the desired target (`setpoint`) and the measured process value (`measurement`).
// * It applies the PID formula =  u = Kp * error + Ki * integral(error) + Kd * derivative(error)
// *
// * The function also includes anti-windup protection to prevent the integrator
// * from accumulating when the output saturates (e.g, at full duty cycle).
// *
// * @param[in,out] pid         Pointer to the PID controller structure.
// * @param[in] setpoint        The desired target value
// * @param[in] measurement     The current measured value.
// */
//float pid_step(pid_t *pid, float setpoint, float measurement)
//{
//    float err = setpoint - measurement;
//
//    // First-call derivative init to avoid big spike
//    if (!pid->initialized)
//    {
//        pid->prev_err = err;
//        pid->initialized = 1;
//    }
//
//    // Proportional
//    float P = pid->kp * err;
//
//    // Integral (unclamped for now)
//    pid->integrator += err * pid->dt_s;
//    float I = pid->ki * pid->integrator;
//
//    // derivative on error
//    float derr = (err - pid->prev_err) / pid->dt_s;
//    float D = pid->kd * derr;
//
//    // unclamped output
//    float u = P + I + D;
//
//    // Anti-windup: if output saturates, back out the last integral step
//    float u_clamped = clampf(u, pid->out_min, pid->out_max);
//    if (u != u_clamped) {
//        pid->integrator -= err * pid->dt_s;
//        I = pid->ki * pid->integrator;
//        u = P + I + D;
//        u_clamped = clampf(u, pid->out_min, pid->out_max);
//    }
//
//    pid->prev_err = err;
//    return u_clamped;
//}
//
//


/*
 * pid.c
 *
 *  Created on: Nov 12, 2025
 *      Author: aaron
 */
#include "pid.h"

/**
 * @brief Clamp a floating-point value between lo and hi.
 */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief Initialize PID controller state and configuration.
 *
 * NOTE:
 *  - out_min/out_max are now interpreted as your actuator command limits.
 *    For MAX1968 control, these should be in AMPS (e.g. -2.0 to +2.0).
 */
void pid_init(pid_t *pid, float kp, float ki, float kd, float dt_s, float out_min, float out_max)
{
    pid->kp   = kp;
    pid->ki   = ki;
    pid->kd   = kd;
    pid->dt_s = dt_s;

    if (out_min <= out_max) {
        pid->out_min = out_min;
        pid->out_max = out_max;
    } else {
        pid->out_min = out_max;
        pid->out_max = out_min;
    }

    pid->integrator  = 0.0f;
    pid->prev_err    = 0.0f;
    pid->initialized = 0;
}

/**
 * @brief One PID step.
 *
 * Returns control output in the same units as out_min/out_max.
 * For MAX1968: return value should be interpreted as I_cmd (A).
 *
 * Anti-windup method:
 *  - "Integrator clamping": we clamp the integrator so that (P + I + D)
 *    cannot exceed output bounds.
 *  - This behaves better than "undo last step" when saturated for many cycles.
 */
float pid_step(pid_t *pid, float setpoint, float measurement)
{
    float err = setpoint - measurement;

    // First-call init to avoid derivative spike
    if (!pid->initialized) {
        pid->prev_err = err;
        pid->initialized = 1;
    }

    // Proportional
    float P = pid->kp * err;

    // Derivative on error
    float derr = (err - pid->prev_err) / pid->dt_s;
    float D = pid->kd * derr;

    // ----- Integral with anti-windup (integrator clamping) -----
    // Candidate integrator update
    float integrator_new = pid->integrator + (err * pid->dt_s);

    // Convert to I-term
    float I_new = pid->ki * integrator_new;

    // Unclamped output using updated integrator
    float u_new = P + I_new + D;

    // If it would saturate, clamp the integrator so output hits the limit exactly
    if (u_new > pid->out_max)
    {
        // Want: P + (ki*integrator_clamped) + D = out_max
        // => integrator_clamped = (out_max - P - D) / ki
        if (pid->ki != 0.0f) {
            integrator_new = (pid->out_max - P - D) / pid->ki;
        }
        // else ki=0 means no integrator anyway
        u_new = pid->out_max;
    }
    else if (u_new < pid->out_min)
    {
        if (pid->ki != 0.0f) {
            integrator_new = (pid->out_min - P - D) / pid->ki;
        }
        u_new = pid->out_min;
    }

    // Commit state
    pid->integrator = integrator_new;
    pid->prev_err   = err;

    // Final clamp (safety)
    return clampf(u_new, pid->out_min, pid->out_max);
}

