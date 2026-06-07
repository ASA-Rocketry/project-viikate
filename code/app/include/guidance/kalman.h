#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "math/linmath.h"
#include "sensors.h"

typedef struct {
    uint64_t last_measurement_timestamp_ns;
    vec3 position;
    vec3 velocity;
    quat orientation;
    vec3 angular_velocity;
    vec3 accelerometer_bias;
    vec3 gyroscope_bias;
    float gravity;
} kalman_filter_t;

void ekf_predict(kalman_filter_t *s, imu_sample_t sample);

kalman_filter_t kalman_init(const vec3 g);
