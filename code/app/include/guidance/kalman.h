#pragma once
#include <arm_math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef void (*kalman_propagate_fn_t)(
    float32_t *restrict x_next,
    const float32_t *restrict x,
    const float32_t *restrict u,
    float dt
);

typedef struct {
    uint8_t dim_x;
    uint8_t dim_z;

    float32_t process_noise_intensity;

    // When allocated with kalman_init(), this equals the returned pointer
    // and is freed in kalman_deinit(). For inplace init, this is NULL.
    void *owned_block;

    // Core matrices/vectors
    float32_t *state_data;  // x (n)
    float32_t *covariance_data;  // P (n×n)
    float32_t *state_transition_data;  // F (n×n)
    float32_t *process_noise_data;  // Q (n×n)
    float32_t *measurement_matrix_data;  // H (m×n)
    float32_t *measurement_noise_data;  // R (m×m)

    // Working buffers (off stack)
    float32_t *work_x_pred;  // (n)
    float32_t *work_fp;  // (n×n)
    float32_t *work_ft;  // (n×n)
    float32_t *work_p_pred;  // (n×n)
    float32_t *work_p_new;  // (n×n)
    float32_t *work_hx;  // (m)
    float32_t *work_innovation;  // (m)
    float32_t *work_ht;  // (n×m)
    float32_t *work_pht;  // (n×m)
    float32_t *work_hph_t;  // (m×m)
    float32_t *work_s;  // (m×m)
    float32_t *work_invs;  // (m×m)
    float32_t *work_k;  // (n×m)
    float32_t *work_ky;  // (n)
    float32_t *work_kh;  // (n×n)
    float32_t *work_i_minus_kh;  // (n×n)

    kalman_propagate_fn_t propagate;
} kalman_filter_state_t;

// --- Allocation & management ---
size_t kalman_bytes(uint8_t dim_x, uint8_t dim_z);

kalman_filter_state_t *kalman_init(uint8_t dim_x, uint8_t dim_z);

kalman_filter_state_t *
kalman_init_inplace(void *mem, size_t bytes, uint8_t dim_x, uint8_t dim_z);

void kalman_deinit(kalman_filter_state_t *kf);
void kalman_cleanup(void *v);

// --- Core operations ---
void kalman_prepare_cv_model(kalman_filter_state_t *kf, float dt, float q);
void kalman_predict(kalman_filter_state_t *kf);
void kalman_predict_ekf(
    kalman_filter_state_t *kf,
    const float32_t *u,
    float dt,
    float eps
);
void kalman_update(kalman_filter_state_t *kf, const float32_t *measured);
