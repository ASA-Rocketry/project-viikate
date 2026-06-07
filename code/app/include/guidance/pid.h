#ifndef ROLL_PID_H_
#define ROLL_PID_H_

#include <stdbool.h>

#include "servo.h" /* for servo_t, DT_SERVO_GET, servo_set_angle() */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- PID configuration ---------- */
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
    float integral_limit; /* anti-windup clamp */
    float output_limit; /* max correction in degrees */

    bool initialized; /* avoids derivative spike on first update */
} pid_t;

/**
 * @brief Reset the roll PID controller's internal state.
 *
 * Call this when re-enabling control after a pause to prevent
 * integral/derivative kick from stale state.
 */
void roll_control_reset(void);

/**
 * @brief Update roll control loop and drive the 4 servos.
 *
 * @param current_roll Current roll angle in degrees.
 * @param target_roll  Desired roll angle in degrees.
 * @param dt           Time elapsed since last call, in seconds.
 */
void roll_control_update(float current_roll, float target_roll, float dt);

#ifdef __cplusplus
}
#endif

#endif /* ROLL_PID_H_ */
