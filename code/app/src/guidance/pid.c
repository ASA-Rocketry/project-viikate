#include "guidance/pid.h"

#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"

static const servo_t servo0 = DT_SERVO_GET(DT_NODELABEL(servo0));
static const servo_t servo1 = DT_SERVO_GET(DT_NODELABEL(servo1));
static const servo_t servo2 = DT_SERVO_GET(DT_NODELABEL(servo2));
static const servo_t servo3 = DT_SERVO_GET(DT_NODELABEL(servo3));

/* ---------- Servo trim / neutral positions ---------- */
#define SERVO0_NEUTRAL 0.0f
#define SERVO1_NEUTRAL 0.0f
#define SERVO2_NEUTRAL 0.0f
#define SERVO3_NEUTRAL 0.0f

#define SERVO_MIN_ANGLE -90.0f
#define SERVO_MAX_ANGLE 90.0f

/* ---------- PID state ---------- */
static pid_t roll_pid = {
    .kp = 0.1745f * 180.0f / 3.14f,
    .ki = 0.0f,
    .kd = 0.09f * 180.0f / 3.14f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 20.0f,
    .output_limit = 45.0f,
    .initialized = false,
};

/* ---------- Helpers ---------- */
static float clampf(float val, float min, float max) {
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

static void pid_reset(pid_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->initialized = false;
}

/*
 * Update PID controller.
 * error: target - current (degrees)
 * dt: time since last update (seconds)
 * returns: correction output (degrees), clamped to output_limit
 */
static float pid_update(pid_t *pid, float error, float dt) {
    if (dt <= 0.0f) {
        dt = 1e-3f; /* avoid div-by-zero */
    }

    /* Integral term with anti-windup */
    pid->integral += error * dt;
    pid->integral =
        clampf(pid->integral, -pid->integral_limit, pid->integral_limit);

    /* Derivative term (skip on first sample to avoid spike) */
    float derivative = 0.0f;
    if (pid->initialized) {
        derivative = (error - pid->prev_error) / dt;
    } else {
        pid->initialized = true;
    }
    pid->prev_error = error;

    float output =
        (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    return clampf(output, -pid->output_limit, pid->output_limit);
}

void roll_control_reset(void) {
    pid_reset(&roll_pid);
}

/*
 * Roll control update.
 *
 * Assumes servo0/servo1 are on one side and servo2/servo3 are on the
 * opposite side, moving in mirrored fashion to correct roll (differential
 * mixing). Adjust the sign/mapping to match your mechanism.
 */
void roll_control_update(float current_roll, float target_roll, float dt) {
    float error = target_roll - current_roll;
    printk("erorr %f\n", (double)error);

    float correction = pid_update(&roll_pid, error, dt);

    float angle0 = SERVO0_NEUTRAL + correction;
    float angle1 = SERVO1_NEUTRAL + correction;
    float angle2 = SERVO2_NEUTRAL - correction;
    float angle3 = SERVO3_NEUTRAL - correction;

    angle0 = clampf(angle0, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    angle1 = clampf(angle1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    angle2 = clampf(angle2, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    angle3 = clampf(angle3, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    uint64_t t_before = k_ticks_to_ns_floor64(k_uptime_ticks());
    servo_set_angle(servo0, angle0);
    servo_set_angle(servo1, angle1);
    servo_set_angle(servo2, angle2);
    servo_set_angle(servo3, angle3);
    uint64_t t_after = k_ticks_to_ns_floor64(k_uptime_ticks());
    uint64_t roll_dt = t_after - t_before;

    static uint64_t roll_dt_max = 0;
    if (roll_dt > roll_dt_max) {
        roll_dt_max = roll_dt;
        printk(
            "NEW MAX roll_control_update: %llu ns at loop \n",
            (unsigned long long)roll_dt
        );
    }
}
