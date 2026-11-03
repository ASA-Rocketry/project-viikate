#include "guidance/kalman.h"

#include "math/linmath.h"
#include "sensors.h"

void ekf_predict(kalman_filter_t *s, imu_sample_t sample) {
    float dt = s->last_measurement_timestamp_ns != 0
        ? (float)(sample.timestamp_ns - s->last_measurement_timestamp_ns)
            * 1e-9f
        : 0.001f;

    vec3 a;
    vec3 w;

    vec3_sub(a, sample.acceleration, s->accelerometer_bias);
    vec3_sub(w, sample.angular_rate, s->gyroscope_bias);

    // Save prior attitude for Jacobian evaluation
    quat q_prev;
    quat_dup(q_prev, s->orientation);

    quat q_dot;
    quat omega_q = {w[0], w[1], w[2], 0.0f};
    quat_mul(q_dot, s->orientation, omega_q);
    quat_scale(q_dot, q_dot, 0.5f * dt);
    quat_add(s->orientation, s->orientation, q_dot);
    quat_norm(s->orientation, s->orientation);

    vec3 a_world;
    quat_mul_vec3(a_world, s->orientation, a);
    a_world[2] -= s->gravity;

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
            float W_a_ij = R[i][0] * R[j][0] * s->accel_noise_var[0]
                + R[i][1] * R[j][1] * s->accel_noise_var[1]
                + R[i][2] * R[j][2] * s->accel_noise_var[2];

            s->P[EKF_IDX_POS + i][EKF_IDX_POS + j] += dt4_4 * W_a_ij;  // Q_pp
            s->P[EKF_IDX_POS + i][EKF_IDX_VEL + j] += dt3_2 * W_a_ij;  // Q_pv
            s->P[EKF_IDX_VEL + i][EKF_IDX_POS + j] += dt3_2 * W_a_ij;  // Q_vp
            s->P[EKF_IDX_VEL + i][EKF_IDX_VEL + j] += dt2 * W_a_ij;  // Q_vv
        }
    }

    // 7b. Gyro control noise projection: W_g = Xi * M_g * Xi^T [4x4]
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            float W_g_rc = Xi[r][0] * Xi[c][0] * s->gyro_noise_var[0]
                + Xi[r][1] * Xi[c][1] * s->gyro_noise_var[1]
                + Xi[r][2] * Xi[c][2] * s->gyro_noise_var[2];

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
    float pitch = atanf(
        -g_norm[0] / sqrtf(g_norm[1] * g_norm[1] + g_norm[2] * g_norm[2])
    );
    float roll = atanf(g_norm[1] / g_norm[2]);
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
            1.0f;  // Position variance (1 m^2)
        k.P[EKF_IDX_VEL + i][EKF_IDX_VEL + i] =
            0.1f;  // Velocity variance (0.1 (m/s)^2)
        k.P[EKF_IDX_GYRO_BIAS + i][EKF_IDX_GYRO_BIAS + i] =
            1e-4f;  // Gyro bias variance
        k.P[EKF_IDX_ACCEL_BIAS + i][EKF_IDX_ACCEL_BIAS + i] =
            1e-2f;  // Accel bias variance
    }
    for (int i = 0; i < 4; i++) {
        k.P[EKF_IDX_QUAT + i][EKF_IDX_QUAT + i] =
            1e-2f;  // Attitude quaternion variance
    }

    // Initialize nominal IMU process noise variances (M and Q_bias)
    for (int i = 0; i < 3; i++) {
        k.accel_noise_var[i] = 0.25f;  // (0.5 m/s^2)^2
        k.gyro_noise_var[i] = 1e-3f;  // (~0.03 rad/s)^2
        k.gyro_bias_noise_var[i] = 1e-6f;  // Gyro bias random walk
        k.accel_bias_noise_var[i] = 1e-4f;  // Accel bias random walk
    }
    return k;
}
