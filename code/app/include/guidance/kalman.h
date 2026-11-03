#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "math/linmath.h"
#include "sensors.h"

// State vector indices in state covariance matrix P[16][16]
#define EKF_IDX_POS 0  // 0..2:   position (x, y, z)
#define EKF_IDX_VEL 3  // 3..5:   velocity (vx, vy, vz)
#define EKF_IDX_QUAT 6  // 6..9:   attitude (qx, qy, qz, qw)
#define EKF_IDX_GYRO_BIAS 10  // 10..12: gyroscope_bias (bgx, bgy, bgz)
#define EKF_IDX_ACCEL_BIAS 13  // 13..15: accelerometer_bias (bax, bay, baz)
#define EKF_STATE_DIM 16

typedef struct {
    uint64_t last_measurement_timestamp_ns;
    vec3 position;
    vec3 velocity;
    quat orientation;
    vec3 gyroscope_bias;
    vec3 accelerometer_bias;
    vec3 angular_velocity;
    float gravity;

    // State Covariance Matrix P [16x16]
    float P[EKF_STATE_DIM][EKF_STATE_DIM];

    // Process noise variances (diagonal M in control space, diagonal Q_bias in state space)
    vec3 accel_noise_var;  // sigma_a^2  [(m/s^2)^2]
    vec3 gyro_noise_var;  // sigma_g^2  [(rad/s)^2]
    vec3 gyro_bias_noise_var;  // sigma_bg^2 [(rad/s)^2 / s]
    vec3 accel_bias_noise_var;  // sigma_ba^2 [(m/s^2)^2 / s]
} kalman_filter_t;

void ekf_predict(kalman_filter_t *s, imu_sample_t sample);

kalman_filter_t kalman_init(const vec3 g);
