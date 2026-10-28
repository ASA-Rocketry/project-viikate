#include "guidance/pid.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>

#include "servo.h"
#include "zephyr/sys/printk.h"

static const servo_t servo0 = DT_SERVO_GET(DT_NODELABEL(servo0));
static const servo_t servo1 = DT_SERVO_GET(DT_NODELABEL(servo1));
static const servo_t servo2 = DT_SERVO_GET(DT_NODELABEL(servo2));
static const servo_t servo3 = DT_SERVO_GET(DT_NODELABEL(servo3));

#define SERVO_MAX_ANGLE 20.0f

static pid_controller_t roll_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 5.16f,
    .integral_limit = 0.35f,
    .output_limit = SERVO_MAX_ANGLE,
};

static pid_controller_t pitch_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 5.16f,
    .integral_limit = 0.35f,
    .output_limit = SERVO_MAX_ANGLE,
};

static float clampf(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float pid_update(pid_controller_t *pid, float error, float dt) {
    pid->integral = clampf(
        pid->integral + error * dt,
        -pid->integral_limit,
        pid->integral_limit
    );

    float derivative = 0.0f;
    if (pid->initialized) {
        derivative = (error - pid->prev_error) / dt;
    } else {
        pid->initialized = true;
    }
    pid->prev_error = error;

    return clampf(
        pid->kp * error + pid->ki * pid->integral + pid->kd * derivative,
        -pid->output_limit,
        pid->output_limit
    );
}

static void reset_pid(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->initialized = false;
}

static int set_all_servos(
    float angle0,
    float angle1,
    float angle2,
    float angle3
) {
    const servo_t servos[] = {servo0, servo1, servo2, servo3};
    const float angles[] = {angle0, angle1, angle2, angle3};
    int first_error = 0;

    for (size_t i = 0; i < ARRAY_SIZE(servos); ++i) {
        int ret = servo_set_angle(servos[i], angles[i]);
        if (ret != 0 && first_error == 0) {
            first_error = ret;
        }
    }

    return first_error;
}

int attitude_control_init(void) {
    const servo_t servos[] = {servo0, servo1, servo2, servo3};
    for (size_t i = 0; i < ARRAY_SIZE(servos); ++i) {
        if (!pwm_is_ready_dt(&servos[i].spec)) {
            printk(
                "Servo PWM device %s is not ready\n",
                servos[i].spec.dev->name
            );
            return -ENODEV;
        }
    }

    reset_pid(&roll_pid);
    reset_pid(&pitch_pid);
    return set_all_servos(0.0f, 0.0f, 0.0f, 0.0f);
}

int attitude_control_update(float roll, float pitch, float dt) {
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(dt) || dt <= 0.0f) {
        return -EINVAL;
    }
    if (dt > 0.1f) {
        dt = 0.1f;
    }

    float roll_output = pid_update(&roll_pid, -roll, dt);
    float pitch_output = pid_update(&pitch_pid, -pitch, dt);

    /* Assumes the existing roll pairs and an orthogonal pitch pairing. */
    float angle0 = pitch_output + roll_output;
    float angle1 = -pitch_output + roll_output;
    float angle2 = -pitch_output - roll_output;
    float angle3 = pitch_output - roll_output;

    angle0 = clampf(angle0, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    angle1 = clampf(angle1, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    angle2 = clampf(angle2, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    angle3 = clampf(angle3, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);

    return set_all_servos(angle0, angle1, angle2, angle3);
}
