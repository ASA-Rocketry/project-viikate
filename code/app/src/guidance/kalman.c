#include "guidance/kalman.h"

#include <math.h>

#include "math/linmath.h"
#include "sensors.h"

// Helper to read state element m in {0..15}
static inline float ekf_get_state(const kalman_filter_t *s, int m) {
    if (m < 3)
        return s->position[m];
    if (m < 6)
        return s->velocity[m - 3];
    if (m < 10)
        return s->orientation[m - 6];
    if (m < 13)
        return s->gyroscope_bias[m - 10];
    return s->accelerometer_bias[m - 13];
}

// Helper to write current state element m in {0..15}
static inline void ekf_set_state(kalman_filter_t *s, int m, float val) {
    if (m < 3)
        s->position[m] = val;
    else if (m < 6)
        s->velocity[m - 3] = val;
    else if (m < 10)
        s->orientation[m - 6] = val;
    else if (m < 13)
        s->gyroscope_bias[m - 10] = val;
    else
        s->accelerometer_bias[m - 13] = val;
}

// Record predicted position and velocity in the (100 ms) ring buffer
static inline void ekf_push_history(kalman_filter_t *s, uint64_t timestamp_ns) {
    ekf_pv_history_t *slot = &s->history[s->history_head];
    slot->timestamp_ns = timestamp_ns;
    slot->pv[0] = s->position[0];
    slot->pv[1] = s->position[1];
    slot->pv[2] = s->position[2];
    slot->pv[3] = s->velocity[0];
    slot->pv[4] = s->velocity[1];
    slot->pv[5] = s->velocity[2];

    s->history_head = (uint16_t)((s->history_head + 1U) % EKF_HISTORY_LEN);
    if (s->history_count < EKF_HISTORY_LEN) {
        s->history_count++;
    }
}

// Find buffered position and velocity closest to observation timestamp
static const float *
ekf_find_history_pv(const kalman_filter_t *s, uint64_t obs_timestamp_ns) {
    if (s->history_count == 0 || obs_timestamp_ns == 0) {
        return NULL;
    }

    uint16_t best_idx = 0;
    uint64_t best_diff = UINT64_MAX;

    for (uint16_t i = 0; i < s->history_count; i++) {
        uint64_t t_i = s->history[i].timestamp_ns;
        uint64_t diff = (t_i >= obs_timestamp_ns) ? (t_i - obs_timestamp_ns)
                                                  : (obs_timestamp_ns - t_i);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx = i;
        }
    }
    return s->history[best_idx].pv;
}

//TODO: Check the logic
static inline bool ekf_should_freeze_biases(FLIGHTSTATE state) {
    return (
        state == Recovery || state == Landing || state == Fault
        || state == Abort
    );
}

// Lever-arm compensation before ekf_predict to calculate a_cg from estimated a_imu,
void ekf_compensate_lever_arm(
    vec3 a_cg_out,
    const vec3 a_imu,
    const vec3 omega,
    const vec3 omega_prev,
    const vec3 r_imu,
    float dt
) {
    if (dt <= 1e-6f) {
        vec3_dup(a_cg_out, a_imu);
        return;
    }
    const float inv_dt = 1.0f / dt;
    const float alpha_x = (omega[0] - omega_prev[0]) * inv_dt;
    const float alpha_y = (omega[1] - omega_prev[1]) * inv_dt;
    const float alpha_z = (omega[2] - omega_prev[2]) * inv_dt;

    const float wx = omega[0], wy = omega[1], wz = omega[2];
    const float rx = r_imu[0], ry = r_imu[1], rz = r_imu[2];

    // ([alpha]_x + [omega]_x^2) * r_imu
    const float rot_acc_x = -(wy * wy + wz * wz) * rx + (wx * wy - alpha_z) * ry
        + (wx * wz + alpha_y) * rz;
    const float rot_acc_y = (wx * wy + alpha_z) * rx - (wx * wx + wz * wz) * ry
        + (wy * wz - alpha_x) * rz;
    const float rot_acc_z = (wx * wz - alpha_y) * rx + (wy * wz + alpha_x) * ry
        - (wx * wx + wy * wy) * rz;

    a_cg_out[0] = a_imu[0] - rot_acc_x;
    a_cg_out[1] = a_imu[1] - rot_acc_y;
    a_cg_out[2] = a_imu[2] - rot_acc_z;
}

void ekf_predict(kalman_filter_t *s, imu_sample_t sample, FLIGHTSTATE state) {
    float dt = s->last_measurement_timestamp_ns != 0
        ? (float)(sample.timestamp_ns - s->last_measurement_timestamp_ns)
            * 1e-9f
        : 0.001f;

    if (dt <= 0.0f || dt > 0.1f) {  //Fall back on 1kHz
        dt = 0.001f;
    }

    vec3 a;
    vec3 w;

    vec3_sub(a, sample.acceleration, s->accelerometer_bias);
    vec3_sub(w, sample.angular_rate, s->gyroscope_bias);

    // Use prior attitude to rotate acceleration to world-frame a_w = R(q)*a-g
    vec3 a_world;
    quat_mul_vec3(a_world, s->orientation, a);
    a_world[2] -= s->gravity;

    // Save prior attitude for Jacobian evaluation
    quat q_prev;
    quat_dup(q_prev, s->orientation);

    // Quaternion propagation: q_bar = q + 0.5 * dt * (q (x) [w_x, w_y, w_z, 0]), then normalize
    quat q_dot;
    quat omega_q = {w[0], w[1], w[2], 0.0f};
    quat_mul(q_dot, s->orientation, omega_q);
    quat_scale(q_dot, q_dot, 0.5f * dt);
    quat_add(s->orientation, s->orientation, q_dot);
    quat_norm(s->orientation, s->orientation);

    // Position and velocity propagation
    vec3 pos_term;
    vec3_scale(pos_term, s->velocity, dt);
    vec3 a_term;
    vec3_scale(a_term, a_world, 0.5f * dt * dt);
    vec3_add(s->position, s->position, pos_term);
    vec3_add(s->position, s->position, a_term);
    vec3 v_term;
    vec3_scale(v_term, a_world, dt);
    vec3_add(s->velocity, s->velocity, v_term);

    vec3_dup(s->angular_velocity, w);
    s->last_measurement_timestamp_ns = sample.timestamp_ns;

    // Save predicted [px, py, pz, vx, vy, vz] for Larsen's delayed GPS update
    ekf_push_history(s, sample.timestamp_ns);

    // Flight-state scaling for IMU white noise M TODO: Check If this is needed for subsonic flights
    float imu_scale = 1.0f;
    if (state == Launch) {
        imu_scale = 15.0f;  // Motor vibration inflation
    } else if (state == Recovery || state == Landing) {
        imu_scale = 10.0f;  // Parachute deployment / landing shock inflation
    }

    // State covariance propagation using the previous state: P = F * P * F^T + V * M * V^T + Q_bias
    const float qx = q_prev[0], qy = q_prev[1], qz = q_prev[2], qw = q_prev[3];
    const float ax = a[0], ay = a[1], az = a[2];
    const float wx = w[0], wy = w[1], wz = w[2];

    const float half_dt = 0.5f * dt;
    const float dt2 = dt * dt;
    const float half_dt2 = 0.5f * dt2;

    // 1. Body-to-World Rotation Matrix R(q)
    const float R[3][3] = {
        {1.0f - 2.0f * (qy * qy + qz * qz),
         2.0f * (qx * qy - qw * qz),
         2.0f * (qx * qz + qw * qy)},
        {2.0f * (qx * qy + qw * qz),
         1.0f - 2.0f * (qx * qx + qz * qz),
         2.0f * (qy * qz - qw * qx)},
        {2.0f * (qx * qz - qw * qy),
         2.0f * (qy * qz + qw * qx),
         1.0f - 2.0f * (qx * qx + qy * qy)}
    };

    // 2. Rotation Jacobian J_q = d(R(q)*a)/dq
    const float u0 = qx * ax + qy * ay + qz * az;
    const float u1 = qw * ax - qz * ay + qy * az;
    const float u2 = qz * ax + qw * ay - qx * az;
    const float u3 = -qy * ax + qx * ay + qw * az;

    const float J_q[3][4] = {
        {2.0f * u0, 2.0f * u3, -2.0f * u2, 2.0f * u1},
        {-2.0f * u3, 2.0f * u0, 2.0f * u1, 2.0f * u2},
        {2.0f * u2, -2.0f * u1, 2.0f * u0, 2.0f * u3}
    };

    // 3. Quaternion kinematics sub-matrices F_qq and Xi(q)
    const float F_qq[4][4] = {
        {1.0f, half_dt * wz, -half_dt * wy, half_dt * wx},
        {-half_dt * wz, 1.0f, half_dt * wx, half_dt * wy},
        {half_dt * wy, -half_dt * wx, 1.0f, half_dt * wz},
        {-half_dt * wx, -half_dt * wy, -half_dt * wz, 1.0f}
    };

    const float Xi[4][3] =
        {{qw, -qz, qy}, {qz, qw, -qx}, {-qy, qx, qw}, {-qx, -qy, -qz}};

    // 4. Scaled sub-blocks of Jacobian F
    float F_pq[3][4], F_vq[3][4], F_pba[3][3], F_vba[3][3], F_qbg[4][3];
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 4; k++) {
            F_pq[i][k] = half_dt2 * J_q[i][k];
            F_vq[i][k] = dt * J_q[i][k];
        }
        for (int k = 0; k < 3; k++) {
            F_pba[i][k] = -half_dt2 * R[i][k];
            F_vba[i][k] = -dt * R[i][k];
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int k = 0; k < 3; k++) {
            F_qbg[i][k] = -half_dt * Xi[i][k];
        }
    }

    // 5. Block-sparse intermediate product: T = F * P  [16x16]
    float T[16][16];
    for (int j = 0; j < 16; j++) {
        // Position rows (0..2) and Velocity rows (3..5)
        for (int i = 0; i < 3; i++) {
            float sum_pq = 0.0f, sum_vq = 0.0f;
            for (int k = 0; k < 4; k++) {
                float p_qk = s->P[EKF_IDX_QUAT + k][j];
                sum_pq += F_pq[i][k] * p_qk;
                sum_vq += F_vq[i][k] * p_qk;
            }
            float sum_pba = 0.0f, sum_vba = 0.0f;
            for (int k = 0; k < 3; k++) {
                float p_bak = s->P[EKF_IDX_ACCEL_BIAS + k][j];
                sum_pba += F_pba[i][k] * p_bak;
                sum_vba += F_vba[i][k] * p_bak;
            }
            T[EKF_IDX_POS + i][j] = s->P[EKF_IDX_POS + i][j]
                + dt * s->P[EKF_IDX_VEL + i][j] + sum_pq + sum_pba;
            T[EKF_IDX_VEL + i][j] = s->P[EKF_IDX_VEL + i][j] + sum_vq + sum_vba;
        }
        // Quaternion rows (6..9)
        for (int i = 0; i < 4; i++) {
            float sum_qq = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum_qq += F_qq[i][k] * s->P[EKF_IDX_QUAT + k][j];
            }
            float sum_qbg = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum_qbg += F_qbg[i][k] * s->P[EKF_IDX_GYRO_BIAS + k][j];
            }
            T[EKF_IDX_QUAT + i][j] = sum_qq + sum_qbg;
        }

        // Gyro & Accel Bias rows (10..15): F is identity on bias rows
        for (int i = 10; i < 16; i++) {
            T[i][j] = s->P[i][j];
        }
    }

    // 6. Block-sparse outer product: P = T * F^T  [16x16]
    for (int r = 0; r < 16; r++) {
        // Position cols (0..2) and Velocity cols (3..5)
        for (int c = 0; c < 3; c++) {
            float sum_pq = 0.0f, sum_vq = 0.0f;
            for (int k = 0; k < 4; k++) {
                float t_qk = T[r][EKF_IDX_QUAT + k];
                sum_pq += t_qk * F_pq[c][k];
                sum_vq += t_qk * F_vq[c][k];
            }
            float sum_pba = 0.0f, sum_vba = 0.0f;
            for (int k = 0; k < 3; k++) {
                float t_bak = T[r][EKF_IDX_ACCEL_BIAS + k];
                sum_pba += t_bak * F_pba[c][k];
                sum_vba += t_bak * F_vba[c][k];
            }
            s->P[r][EKF_IDX_POS + c] = T[r][EKF_IDX_POS + c]
                + dt * T[r][EKF_IDX_VEL + c] + sum_pq + sum_pba;
            s->P[r][EKF_IDX_VEL + c] = T[r][EKF_IDX_VEL + c] + sum_vq + sum_vba;
        }
        // Quaternion cols (6..9)
        for (int c = 0; c < 4; c++) {
            float sum_qq = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum_qq += T[r][EKF_IDX_QUAT + k] * F_qq[c][k];
            }
            float sum_qbg = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum_qbg += T[r][EKF_IDX_GYRO_BIAS + k] * F_qbg[c][k];
            }
            s->P[r][EKF_IDX_QUAT + c] = sum_qq + sum_qbg;
        }

        // Gyro & Accel Bias cols (10..15)
        for (int c = 10; c < 16; c++) {
            s->P[r][c] = T[r][c];
        }
    }

    // 7. Add Process Noise: Q = V * M * V^T + Q_bias
    // 7a. Accel control noise projection: W_a = R * M_a * R^T [3x3]
    const float dt3_2 = 0.5f * dt2 * dt;
    const float dt4_4 = 0.25f * dt2 * dt2;
    const float dt2_4 = 0.25f * dt2;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float W_a_ij = R[i][0] * R[j][0] * s->accel_noise_var[0] * imu_scale
                + R[i][1] * R[j][1] * s->accel_noise_var[1] * imu_scale
                + R[i][2] * R[j][2] * s->accel_noise_var[2] * imu_scale;

            s->P[EKF_IDX_POS + i][EKF_IDX_POS + j] += dt4_4 * W_a_ij;  // Q_pp
            s->P[EKF_IDX_POS + i][EKF_IDX_VEL + j] += dt3_2 * W_a_ij;  // Q_pv
            s->P[EKF_IDX_VEL + i][EKF_IDX_POS + j] += dt3_2 * W_a_ij;  // Q_vp
            s->P[EKF_IDX_VEL + i][EKF_IDX_VEL + j] += dt2 * W_a_ij;  // Q_vv
        }
    }

    // 7b. Gyro control noise projection: W_g = Xi * M_g * Xi^T [4x4]
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            float W_g_rc =
                Xi[r][0] * Xi[c][0] * s->gyro_noise_var[0] * imu_scale
                + Xi[r][1] * Xi[c][1] * s->gyro_noise_var[1] * imu_scale
                + Xi[r][2] * Xi[c][2] * s->gyro_noise_var[2] * imu_scale;

            s->P[EKF_IDX_QUAT + r][EKF_IDX_QUAT + c] += dt2_4 * W_g_rc;  // Q_qq
        }
    }

    // 7c. Bias random walk in state space: Q_bias
    for (int i = 0; i < 3; i++) {
        s->P[EKF_IDX_GYRO_BIAS + i][EKF_IDX_GYRO_BIAS + i] +=
            s->gyro_bias_noise_var[i] * dt;
        s->P[EKF_IDX_ACCEL_BIAS + i][EKF_IDX_ACCEL_BIAS + i] +=
            s->accel_bias_noise_var[i] * dt;
    }
    ekf_enforce_psd(s);  // Check that P is still positive definite
}

void ekf_update_scalar(
    kalman_filter_t *s,
    int m,
    float z_m,
    float r_var,
    const float *x_delayed_m,
    bool freeze_biases
) {
    if (m < 0 || m >= EKF_STATE_DIM || r_var <= 0.0f) {
        return;
    }

    const float x_pred_m =
        (x_delayed_m != NULL) ? (*x_delayed_m) : ekf_get_state(s, m);
    const float y = z_m - x_pred_m;  // Innovation
    const float S = s->P[m][m] + r_var;  // H = 1
    if (S
        <= 1e-12f) {  //Very small S blows up gain -> throws off the estimate if wrong. TODO: Check if the cutoff is reasonable
        return;
    }

    float K[EKF_STATE_DIM];  // Kalman gains
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        K[i] = s->P[i][m] / S;
    }

    if (freeze_biases) {
        for (int i = EKF_IDX_GYRO_BIAS; i < EKF_STATE_DIM; i++) {
            K[i] = 0.0f;
        }
    }

    // State estimate update: x = x_bar + K * y
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        float xi = ekf_get_state(s, i) + K[i] * y;
        ekf_set_state(s, i, xi);
    }
    quat_norm(s->orientation, s->orientation);  //Normalize attitude

    // Explicit Joseph Form covariance update for H = e_m^T:
    // P_new[i][j] = P[i][j] - K[i]*P[m][j] - P[i][m]*K[j] + S*K[i]*K[j]
    float P_next[EKF_STATE_DIM][EKF_STATE_DIM];
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        const float ki = K[i];
        const float pim = s->P[i][m];
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            const float pmj = s->P[m][j];
            const float kj = K[j];
            P_next[i][j] = s->P[i][j] - ki * pmj - pim * kj + S * ki * kj;
        }
    }
    memcpy(s->P, P_next, sizeof(P_next));
}

void ekf_update_baro(
    kalman_filter_t *s,
    float altitude_agl_m,
    FLIGHTSTATE state
) {
    //TODO: Check if a higher variance is needed for launch
    float r_var = s->baro_noise_var;  // 1.50f m^2 in Coast/Recovery
    if (state == Boot || state == Safe || state == Pad || state == Abort) {
        r_var = EKF_BARO_VAR_PAD;  // 0.04f m^2 on stationary launchpad
    }

    const bool freeze_biases =  //TODO: Check bias freezing logic
        (state == Recovery || state == Landing || state == Fault
         || state == Abort);

    ekf_update_scalar( // Scalar update with m = 2 (z_agl)
        s,
        EKF_IDX_POS + 2,
        altitude_agl_m,
        r_var,
        NULL,
        freeze_biases
    );
    ekf_enforce_psd(s);  // Check and enforce positive definiteness
}

void ekf_update_gps_enu(
    kalman_filter_t *s,
    const vec3 pos_enu_agl_m,
    const vec3 vel_enu_ms,
    float h_acc_m,
    float v_acc_m,
    float s_acc_ms,
    uint64_t obs_timestamp_ns,
    FLIGHTSTATE state
) {
    const float h_std = fmaxf(h_acc_m, EKF_GPS_HACC_FLOOR_M);
    const float v_std = fmaxf(v_acc_m, EKF_GPS_VACC_FLOOR_M);
    const float s_std = fmaxf(s_acc_ms, EKF_GPS_SACC_FLOOR_MS);

    const float var_xy = h_std * h_std;
    const float var_z = v_std * v_std;
    const float var_v = s_std * s_std;

    const float z_vec[6] = {
        pos_enu_agl_m[0],
        pos_enu_agl_m[1],
        pos_enu_agl_m[2],
        vel_enu_ms[0],
        vel_enu_ms[1],
        vel_enu_ms[2]
    };
    const float r_vec[6] = {var_xy, var_xy, var_z, var_v, var_v, var_v};

    const float *hist_pv = ekf_find_history_pv(s, obs_timestamp_ns);
    const bool freeze_biases = ekf_should_freeze_biases(state);

    for (int m = 0; m < 6; m++) {  // Scalar updates for position and velocity
        const float *x_del_m = (hist_pv != NULL) ? &hist_pv[m] : NULL;
        ekf_update_scalar(s, m, z_vec[m], r_vec[m], x_del_m, freeze_biases);
    }

    ekf_enforce_psd(s);
}

void ekf_update_mag(
    kalman_filter_t *s,
    const vec3 mag_body_meas,
    const vec3 mag_world_ref,
    FLIGHTSTATE state
) {
    float mag_norm = sqrtf(
        mag_body_meas[0] * mag_body_meas[0]
        + mag_body_meas[1] * mag_body_meas[1]
        + mag_body_meas[2] * mag_body_meas[2]
    );
    if (mag_norm < 1e-6f) {
        return;
    }
    const float z_mag[3] = {
        mag_body_meas[0] / mag_norm,
        mag_body_meas[1] / mag_norm,
        mag_body_meas[2] / mag_norm
    };

    float ref_norm = sqrtf(
        mag_world_ref[0] * mag_world_ref[0]
        + mag_world_ref[1] * mag_world_ref[1]
        + mag_world_ref[2] * mag_world_ref[2]
    );
    if (ref_norm < 1e-6f) {
        return;
    }
    const float mwx = mag_world_ref[0] / ref_norm;
    const float mwy = mag_world_ref[1] / ref_norm;
    const float mwz = mag_world_ref[2] / ref_norm;

    const float qx = s->orientation[0];
    const float qy = s->orientation[1];
    const float qz = s->orientation[2];
    const float qw = s->orientation[3];

    const float v0 = qx * mwx + qy * mwy + qz * mwz;
    const float v1 = qw * mwx + qz * mwy - qy * mwz;
    const float v2 = -qz * mwx + qw * mwy + qx * mwz;
    const float v3 = qy * mwx - qx * mwy + qw * mwz;

    const float J_mag_q[3][4] = {
        //TODO: Check if needs to be re-evalued inside loop
        {2.0f * v0, -2.0f * v3, 2.0f * v2, 2.0f * v1},
        {2.0f * v3, 2.0f * v0, -2.0f * v1, 2.0f * v2},
        {-2.0f * v2, 2.0f * v1, 2.0f * v0, 2.0f * v3}
    };

    const float r_var =
        (s->mag_noise_var > 0.0f) ? s->mag_noise_var : EKF_MAG_VAR_DEFAULT;
    const bool freeze_biases = ekf_should_freeze_biases(state);

    for (int r = 0; r < 3; r++) {
        const float R[3][3] = {
            {1.0f - 2.0f * (qy * qy + qz * qz),
             2.0f * (qx * qy - qw * qz),
             2.0f * (qx * qz + qw * qy)},
            {2.0f * (qx * qy + qw * qz),
             1.0f - 2.0f * (qx * qx + qz * qz),
             2.0f * (qy * qz - qw * qx)},
            {2.0f * (qx * qz - qw * qy),
             2.0f * (qy * qz + qw * qx),
             1.0f - 2.0f * (qx * qx + qy * qy)}
        };

        const float h_r = R[0][r] * mwx + R[1][r] * mwy + R[2][r] * mwz;
        const float y_r = z_mag[r] - h_r;

        // hP[i] = (P * H_r^T)[i] = sum_{a=0..3} P[i][6+a] * J_mag_q[r][a]
        float hP[EKF_STATE_DIM] = {0.0f};
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            float acc = 0.0f;
            for (int a = 0; a < 4; a++) {
                acc += s->P[i][EKF_IDX_QUAT + a] * J_mag_q[r][a];
            }
            hP[i] = acc;
        }

        // S_r = H_r * P * H_r^T + r_var = sum_{a=0..3} J_mag_q[r][a] * hP[6+a] + r_var
        float S_r = r_var;
        for (int a = 0; a < 4; a++) {
            S_r += J_mag_q[r][a] * hP[EKF_IDX_QUAT + a];
        }
        if (S_r <= 1e-12f) {
            continue;
        }

        float K[EKF_STATE_DIM];
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            K[i] = hP[i] / S_r;
        }
        if (freeze_biases) {
            for (int i = EKF_IDX_GYRO_BIAS; i < EKF_STATE_DIM; i++) {
                K[i] = 0.0f;
            }
        }

        for (int i = 0; i < EKF_STATE_DIM; i++) {
            ekf_set_state(s, i, ekf_get_state(s, i) + K[i] * y_r);
        }
        quat_norm(s->orientation, s->orientation);

        float P_next[EKF_STATE_DIM][EKF_STATE_DIM];
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            const float ki = K[i];
            const float hpi = hP[i];
            for (int j = 0; j < EKF_STATE_DIM; j++) {
                P_next[i][j] =
                    s->P[i][j] - ki * hP[j] - hpi * K[j] + S_r * ki * K[j];
            }
        }
        memcpy(s->P, P_next, sizeof(P_next));
    }

    ekf_enforce_psd(s);
}

bool ekf_enforce_psd(kalman_filter_t *s) {
    // 1. Enforce exact symmetry
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = i + 1; j < EKF_STATE_DIM; j++) {
            const float sym = 0.5f * (s->P[i][j] + s->P[j][i]);
            s->P[i][j] = sym;
            s->P[j][i] = sym;
        }
    }

    // 2. Square-root-free LDL^T check: P = L * D * L^T
    float L[EKF_STATE_DIM][EKF_STATE_DIM] = {0};
    float d[EKF_STATE_DIM] = {0};
    bool is_psd = true;

    for (int i = 0; i < EKF_STATE_DIM; i++) {
        L[i][i] = 1.0f;
        float sum_d = 0.0f;
        for (int k = 0; k < i; k++) {
            sum_d += L[i][k] * L[i][k] * d[k];
        }
        d[i] = s->P[i][i] - sum_d;

        if (!isfinite(d[i]) || d[i] <= 1e-9f) {
            is_psd = false;
            d[i] = 1e-6f;
            s->P[i][i] = fmaxf(s->P[i][i], 1e-6f);
            for (int j = 0; j < EKF_STATE_DIM; j++) {
                if (j != i) {
                    s->P[i][j] = 0.0f;
                    s->P[j][i] = 0.0f;
                }
            }
        }

        for (int j = i + 1; j < EKF_STATE_DIM; j++) {
            float sum_l = 0.0f;
            for (int k = 0; k < i; k++) {
                sum_l += L[j][k] * L[i][k] * d[k];
            }
            L[j][i] = (s->P[j][i] - sum_l) / d[i];
        }
    }

    return is_psd;
}

static inline void quat_from_gravity(quat q, const vec3 accel) {
    vec3 g_norm;
    float norm =
        sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (norm < 1e-5f) {
        quat_identity(q);
        return;
    }
    g_norm[0] = accel[0] / norm;
    g_norm[1] = accel[1] / norm;
    g_norm[2] = accel[2] / norm;

    // Pitch/Roll from accelerometer
    float pitch = atan2f(
        -g_norm[0],
        sqrtf(g_norm[1] * g_norm[1] + g_norm[2] * g_norm[2])
    );
    float roll = atan2f(g_norm[1], g_norm[2]);
    float yaw = 0.0f;  // no magnetic reference

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    // Quaternion in [x, y, z, w] order
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
    q[3] = cr * cp * cy + sr * sp * sy;

    quat_norm(q, q);
}

kalman_filter_t kalman_init(const vec3 g) {
    kalman_filter_t k = {0};
    quat_from_gravity(k.orientation, g);
    k.gravity = vec3_len(g);
    // TODO: Check if the bias variances are reasonable from the IMU datasheet
    // Initialize State Covariance P_0 as a diagonal matrix:
    //(Uncertainty at the launchpad)
    for (int i = 0; i < 3; i++) {
        k.P[EKF_IDX_POS + i][EKF_IDX_POS + i] =
            0.0289f;  // Pad pos variance (0.17 m)^2
        k.P[EKF_IDX_VEL + i][EKF_IDX_VEL + i] =
            0.01f;  // Pad vel variance (0.1 m/s)^2
        k.P[EKF_IDX_GYRO_BIAS + i][EKF_IDX_GYRO_BIAS + i] =
            1e-4f;  // Residual after pad tare (uncalibrated G_TyOff is 3e-4f)
        k.P[EKF_IDX_ACCEL_BIAS + i][EKF_IDX_ACCEL_BIAS + i] =
            1e-2f;  // Matches ISM330DHCX LA_TyOff = ±10 mg (0.098 m/s^2)^2
    }
    for (int i = 0; i < 4; i++) {
        k.P[EKF_IDX_QUAT + i][EKF_IDX_QUAT + i] = 1e-2f;
    }

    // Accelerometer noises: hal of M
    // (Z-axis has lower 2.17 kHz resonance & axial thrust)
    k.accel_noise_var[0] =
        0.01f;  // X-axis: (0.1 m/s^2)^2 (datasheet thermal: 3.5e-4f)
    k.accel_noise_var[1] = 0.01f;  // Y-axis: (0.1 m/s^2)^2
    k.accel_noise_var[2] = 0.04f;  // Z-axis: (0.2 m/s^2)^2

    // Initialize nominal IMU process noise variances (half of M and Q_bias)
    for (int i = 0; i < 3; i++) {
        k.gyro_noise_var[i] =
            1e-4f;  // (0.01 rad/s)^2 (datasheet thermal: 7.6e-6f)
        k.gyro_bias_noise_var[i] =
            1e-6f;  // Tracks G_TCOff thermal drift in flight
        k.accel_bias_noise_var[i] =
            1e-4f;  // Tracks LA_TCOff thermal drift in flight
    }

    k.baro_noise_var = EKF_BARO_VAR_FLIGHT;
    k.mag_noise_var = EKF_MAG_VAR_DEFAULT;

    return k;
}
