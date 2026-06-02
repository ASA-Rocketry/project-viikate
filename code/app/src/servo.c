#include "servo.h"

#include <math.h>

#include "zephyr/drivers/pwm.h"

int servo_set_angle(const servo_t servo, float angle) {
    float clamped = fminf(fmaxf(angle, -90.0f), 90.0f);
    float percent = (clamped + 90.0f) / 180.f;
    float range = (float)(servo.max_pulse - servo.min_pulse);
    uint32_t pulse = (uint32_t)(range * percent) + servo.min_pulse;

    int ret = pwm_set_pulse_dt(&servo.spec, pulse);
    return ret;
}
