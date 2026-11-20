#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "math/linmath.h"
#include "sensors.h"
#include "state_machine.h"

// State vector indices in state covariance matrix P[16][16]
#define EKF_IDX_POS 0  // 0..2:   position (x, y, z)
#define EKF_IDX_VEL 3  // 3..5:   velocity (vx, vy, vz)
#define EKF_IDX_QUAT 6  // 6..9:   attitude (qx, qy, qz, qw)
#define EKF_IDX_GYRO_BIAS 10  // 10..12: gyroscope_bias (bgx, bgy, bgz)
#define EKF_IDX_ACCEL_BIAS 13  // 13..15: accelerometer_bias (bax, bay, baz)
#define EKF_STATE_DIM 16

// Ring buffer size for Larsen's delayed GPS innovation method (100 ms at 1 kHz)
#define EKF_HISTORY_LEN 100

// Default measurement noise standard deviations and floors
#define EKF_BARO_STD_DEFAULT 1.5f  // 1.5 m -> var = 2.25 m^2
#define EKF_GPS_HACC_FLOOR_M 1.5f  // Min horizontal pos std dev [m]
#define EKF_GPS_VACC_FLOOR_M 3.0f  // Min vertical pos std dev [m]
#define EKF_GPS_SACC_FLOOR_MS 0.1f  // Min 3D speed std dev [m/s]
#define EKF_MAG_VAR_DEFAULT 2.5e-3f  // (0.05 unit vector)^2

#define EKF_BARO_VAR_PAD 0.0289f  // BME280 on pad: (datasheet RMS = 0.17 m)
#define EKF_BARO_VAR_FLIGHT 1.50f  // in flight: includes A_rel (±1.0 m) & TCO_P
//TODO: Check if this is reasonable
#define EKF_BARO_VAR_BOOST 75.0f  // 50x flight variance during Boost wake

typedef struct {
    uint64_t timestamp_ns;
    float pv
        [6];  // Buffered predicted [px, py, pz_agl, vx, vy, vz] for delayed GPS update
} ekf_pv_history_t;

typedef struct kalman_filter_s {
    uint64_t last_measurement_timestamp_ns;
    vec3 position;  // ENU position with z in AGL [m]
    vec3 velocity;  // ENU velocity [m/s]
    quat orientation;  // Body-to-world quaternion [qx, qy, qz, qw]
    vec3 gyroscope_bias;  // [rad/s]
    vec3 accelerometer_bias;  // [m/s^2]
    vec3 angular_velocity;  // Bias-corrected body angular rate [rad/s]
    float gravity;

    // State Covariance Matrix P [16x16]
    float P[EKF_STATE_DIM][EKF_STATE_DIM];

    // Process noise variances (diagonal M [6x6] in control space, diagonal Q_bias [16x16] in state space)
    vec3 accel_noise_var;  // sigma_a^2  [(m/s^2)^2]
    vec3 gyro_noise_var;  // sigma_g^2  [(rad/s)^2]
    vec3 gyro_bias_noise_var;  // sigma_bg^2 [(rad/s)^2 / s]
    vec3 accel_bias_noise_var;  // sigma_ba^2 [(m/s^2)^2 / s]

    // Barometer measurement noise variance
    float baro_noise_var;
    float mag_noise_var;

    // Ring buffer for delayed GPS innovation (Larsen's method)
    ekf_pv_history_t history[EKF_HISTORY_LEN];
    uint16_t history_head;
    uint16_t history_count;
} kalman_filter_t;

kalman_filter_t kalman_init(const vec3 g);

// IMU lever-arm compensation (happens before EKF)
void ekf_compensate_lever_arm(
    vec3 a_cg_out,
    const vec3 a_imu,
    const vec3 omega,
    const vec3 omega_prev,
    const vec3 r_imu,
    float dt
);

//High-rate EKF prediction step
void ekf_predict(kalman_filter_t *s, imu_sample_t sample, FLIGHTSTATE state);

// Core scalar measurement update for position and velocity (state index m in {0..5}) using Explicit Joseph Form
// If x_delayed_m != NULL, computes innovation against delayed state (Larsen's method) while updating current state/P
void ekf_update_scalar(
    kalman_filter_t *s,
    int m,
    float z_m,
    float r_var,
    const float *x_delayed_m,
    bool freeze_biases
);

// Barometer AGL altitude measurement update (observes p_z, m = 2)
void ekf_update_baro(
    kalman_filter_t *s,
    float altitude_agl_m,
    FLIGHTSTATE state
);

// 6-DOF GPS update (observes ENU/AGL position [m], ENU velocity [m/s], and accuracy std devs. m in {0..5})
void ekf_update_gps_enu(
    kalman_filter_t *s,
    const vec3 pos_enu_agl_m,
    const vec3 vel_enu_ms,
    float h_acc_m,
    float v_acc_m,
    float s_acc_ms,
    uint64_t
        obs_timestamp_ns,  // Pass obs_timestamp_ns = 0 if GPS delay compensation is not used
    FLIGHTSTATE state
);

void ekf_update_mag(
    kalman_filter_t *s,
    const vec3 mag_body_meas,
    const vec3 mag_world_ref,
    FLIGHTSTATE state
);

// LDL^T positive-definiteness check and repair
bool ekf_enforce_psd(kalman_filter_t *s);