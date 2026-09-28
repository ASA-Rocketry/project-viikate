#pragma once

#include <stdint.h>

#include "zephyr/drivers/pwm.h"

typedef struct {
    struct pwm_dt_spec spec;
    uint32_t min_pulse;
    uint32_t max_pulse;
} servo_t;

#define DT_SERVO_GET(nodelabel) \
    (servo_t) { \
        .spec = PWM_DT_SPEC_GET(nodelabel), \
        .min_pulse = DT_PROP(nodelabel, min_pulse), \
        .max_pulse = DT_PROP(nodelabel, max_pulse) \
    }

int servo_set_angle(const servo_t servo, float angle);
