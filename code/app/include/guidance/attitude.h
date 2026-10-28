#pragma once

#include <stdint.h>

#include "math/linmath.h"
#include "sensors.h"

typedef struct {
    uint64_t last_measurement_timestamp_ns;
    quat orientation;
    vec3 angular_velocity;
    vec3 gyroscope_bias;
    float gravity;
} attitude_filter_t;

attitude_filter_t attitude_filter_init(
    const vec3 initial_acceleration,
    const vec3 gyroscope_bias,
    uint64_t timestamp_ns
);
int attitude_filter_update(
    attitude_filter_t *filter,
    const imu_sample_t *sample
);
