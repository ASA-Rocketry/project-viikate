#include "guidance/filters.h"

#include <arm_math.h>

#include "guidance/kalman.h"

kalman_filter_state_t *kalman_init_baro(void) {
    kalman_filter_state_t *kf = kalman_init(2, 1);
    if (!kf)
        return NULL;

    kf->state_data[0] = 101.325f;
    kf->state_data[1] = 0.0f;

    kf->covariance_data[0] = 10000.0f;
    kf->covariance_data[1] = 0.0f;
    kf->covariance_data[2] = 0.0f;
    kf->covariance_data[3] = 1000.0f;

    kf->measurement_matrix_data[0] = 1.0f;
    kf->measurement_matrix_data[1] = 0.0f;

    kf->measurement_noise_data[0] = 0.01f;
    kf->process_noise_intensity = 0.0001f;

    return kf;
}

// Nonlinear propagate for 6-DOF attitude + biases (10-state) using CMSIS-DSP
// State x = [qw qx qy qz bgx bgy bgz bax bay baz]^T
// Input u = [ax ay az wx wy wz]^T (body frame)
// Orientation is propagated using gyro; biases are constant in f (random walk
// is handled via Q in covariance propagation).
static inline void
quat_from_omega_cmsis(const float32_t *w, float32_t dt, float32_t *dq) {
    // angle = |w| * dt; dq = [cos(angle/2), axis * sin(angle/2)]
    float32_t dot = 0.0f;
    arm_dot_prod_f32(w, w, 3u, &dot);

    float32_t wnorm = 0.0f;
    (void)arm_sqrt_f32(dot, &wnorm);

    const float32_t th = 0.5f * wnorm * dt;

    if (wnorm > 1e-8f) {
        float32_t s = arm_sin_f32(th) / wnorm;
        dq[0] = arm_cos_f32(th);
        dq[1] = w[0] * s;
        dq[2] = w[1] * s;
        dq[3] = w[2] * s;
    } else {
        // Small-angle: sin(th)/|w| ~ 0.5*dt
        const float32_t s = 0.5f * dt;
        dq[0] = 1.0f;
        dq[1] = w[0] * s;
        dq[2] = w[1] * s;
        dq[3] = w[2] * s;
        // Normalize with CMSIS just in case
        float32_t tmp_in[4] = {dq[0], dq[1], dq[2], dq[3]};
        arm_quaternion_normalize_f32(tmp_in, dq, 1u);
    }
}

void kalman_imu6dof_propagate(
    float32_t *restrict x_next,
    const float32_t *restrict x,
    const float32_t *restrict u,
    float dt
) {
    // Unpack current state
    const float32_t q[4] = {x[0], x[1], x[2], x[3]};
    const float32_t bgx = x[4], bgy = x[5], bgz = x[6];
    const float32_t bax = x[7], bay = x[8], baz = x[9];

    // Bias-compensated gyro
    const float32_t wm[3] = {u[3], u[4], u[5]};
    const float32_t w[3] = {wm[0] - bgx, wm[1] - bgy, wm[2] - bgz};

    // Delta quaternion from angular rate
    float32_t dq[4];
    quat_from_omega_cmsis(w, dt, dq);

    // q_next = q ⊗ dq
    float32_t qn[4];
    arm_quaternion_product_single_f32(q, dq, qn);

    // Normalize quaternion with CMSIS
    float32_t qn_norm[4];
    arm_quaternion_normalize_f32(qn, qn_norm, 1u);

    // Write back quaternion
    x_next[0] = qn_norm[0];
    x_next[1] = qn_norm[1];
    x_next[2] = qn_norm[2];
    x_next[3] = qn_norm[3];

    // Biases remain (random walk handled by Q)
    x_next[4] = bgx;
    x_next[5] = bgy;
    x_next[6] = bgz;
    x_next[7] = bax;
    x_next[8] = bay;
    x_next[9] = baz;
}

kalman_filter_state_t *kalman_init_imu6dof(void) {
    // n = 10 (qw qx qy qz bgx bgy bgz bax bay baz)
    // m = 3  (accelerometer measurement placeholder)
    kalman_filter_state_t *kf = kalman_init(10u, 3u);
    if (!kf)
        return NULL;

    // Initial state: identity quaternion, zero biases
    const float32_t x0[10] = {
        1.0f,
        0.0f,
        0.0f,
        0.0f,  // q
        0.0f,
        0.0f,
        0.0f,  // gyro bias
        0.0f,
        0.0f,
        0.0f  // accel bias
    };
    arm_copy_f32(x0, kf->state_data, 10u);

    // Covariance P: diagonal with conservative defaults
    arm_fill_f32(0.0f, kf->covariance_data, 10u * 10u);
    const float32_t p_q = 1e-3f;  // attitude variance
    const float32_t p_bg = 0.02f * 0.02f;  // gyro bias variance (rad/s)^2
    const float32_t p_ba = 0.10f * 0.10f;  // accel bias variance (m/s^2)^2

    kf->covariance_data[0 * 10 + 0] = p_q;
    kf->covariance_data[1 * 10 + 1] = p_q;
    kf->covariance_data[2 * 10 + 2] = p_q;
    kf->covariance_data[3 * 10 + 3] = p_q;

    kf->covariance_data[4 * 10 + 4] = p_bg;
    kf->covariance_data[5 * 10 + 5] = p_bg;
    kf->covariance_data[6 * 10 + 6] = p_bg;

    kf->covariance_data[7 * 10 + 7] = p_ba;
    kf->covariance_data[8 * 10 + 8] = p_ba;
    kf->covariance_data[9 * 10 + 9] = p_ba;

    // Process noise Q: diagonal random walk for biases; small on quaternion.
    arm_fill_f32(0.0f, kf->process_noise_data, 10u * 10u);
    const float32_t q_q = 1e-6f;  // attitude process noise
    const float32_t q_bg = 1e-6f;  // gyro bias rw (rad/s)^2 per step
    const float32_t q_ba = 1e-6f;  // accel bias rw (m/s^2)^2 per step

    kf->process_noise_data[0 * 10 + 0] = q_q;
    kf->process_noise_data[1 * 10 + 1] = q_q;
    kf->process_noise_data[2 * 10 + 2] = q_q;
    kf->process_noise_data[3 * 10 + 3] = q_q;

    kf->process_noise_data[4 * 10 + 4] = q_bg;
    kf->process_noise_data[5 * 10 + 5] = q_bg;
    kf->process_noise_data[6 * 10 + 6] = q_bg;

    kf->process_noise_data[7 * 10 + 7] = q_ba;
    kf->process_noise_data[8 * 10 + 8] = q_ba;
    kf->process_noise_data[9 * 10 + 9] = q_ba;

    // Measurement model (m = 3 for accel). H is zeroed since the accel update
    // is nonlinear. Use an EKF measurement update with h(x) and H(x).
    arm_fill_f32(0.0f, kf->measurement_matrix_data, 3u * 10u);

    // Measurement noise R for accelerometer (diag). Example sigma_a ~ 0.2 m/s^2
    arm_fill_f32(0.0f, kf->measurement_noise_data, 3u * 3u);
    const float32_t r_a = 0.2f * 0.2f;
    kf->measurement_noise_data[0 * 3 + 0] = r_a;
    kf->measurement_noise_data[1 * 3 + 1] = r_a;
    kf->measurement_noise_data[2 * 3 + 2] = r_a;

    // Keep symmetry with baro init
    kf->process_noise_intensity = 1e-4f;

    // Wire the nonlinear propagate callback
    kf->propagate = kalman_imu6dof_propagate;

    return kf;
}
