#include "guidance/kalman.h"

#include <stddef.h>
#include <stdlib.h>

// Alignment helper
#define KALMAN_ALIGN 8u

static inline uintptr_t kalman_align_up(uintptr_t p, uintptr_t a) {
    return (p + (a - 1u)) & ~(a - 1u);
}

// === internal helpers ===
static size_t kalman_float_count(uint8_t n, uint8_t m) {
    // Totals:
    //   x, x_pred, Ky                     -> 3n
    //   P, F, Q, FP, Ft, P_pred, P_new,
    //   KH, I_minus_KH                   -> 9n²
    //   Hx, innovation                   -> 2m
    //   R, HPHt, S, invS                 -> 4m²
    //   H, Ht, PHt, K                    -> 4nm
    size_t nn = (size_t)n * (size_t)n;
    size_t mm = (size_t)m * (size_t)m;
    size_t nm = (size_t)n * (size_t)m;
    size_t total = 0;
    total += 3u * (size_t)n;
    total += 9u * nn;
    total += 2u * (size_t)m;
    total += 4u * mm;
    total += 4u * nm;
    return total;
}

size_t kalman_bytes(uint8_t dim_x, uint8_t dim_z) {
    uint8_t n = dim_x;
    uint8_t m = dim_z;
    size_t hdr = (size_t)kalman_align_up(
        (uintptr_t)sizeof(kalman_filter_state_t),
        (uintptr_t)KALMAN_ALIGN
    );
    size_t floats = kalman_float_count(n, m);
    size_t data = floats * sizeof(float32_t);
    return hdr + data;
}

static void kalman_assign_ptrs(kalman_filter_state_t *kf) {
    uint8_t n = kf->dim_x;
    uint8_t m = kf->dim_z;
    uint8_t *base = (uint8_t *)kf;
    uintptr_t p = (uintptr_t)base + (uintptr_t)sizeof(*kf);
    p = kalman_align_up(p, KALMAN_ALIGN);
    float32_t *f = (float32_t *)p;
    float32_t *const f_start = f;

    // === main buffers ===
    kf->state_data = f;
    f += n;
    kf->covariance_data = f;
    f += (size_t)n * (size_t)n;
    kf->state_transition_data = f;
    f += (size_t)n * (size_t)n;
    kf->process_noise_data = f;
    f += (size_t)n * (size_t)n;
    kf->measurement_matrix_data = f;
    f += (size_t)m * (size_t)n;
    kf->measurement_noise_data = f;
    f += (size_t)m * (size_t)m;

    // === work buffers ===
    kf->work_x_pred = f;
    f += n;
    kf->work_fp = f;
    f += (size_t)n * (size_t)n;
    kf->work_ft = f;
    f += (size_t)n * (size_t)n;
    kf->work_p_pred = f;
    f += (size_t)n * (size_t)n;
    kf->work_p_new = f;
    f += (size_t)n * (size_t)n;

    kf->work_hx = f;
    f += m;
    kf->work_innovation = f;
    f += m;
    kf->work_ht = f;
    f += (size_t)n * (size_t)m;
    kf->work_pht = f;
    f += (size_t)n * (size_t)m;
    kf->work_hph_t = f;
    f += (size_t)m * (size_t)m;
    kf->work_s = f;
    f += (size_t)m * (size_t)m;
    kf->work_invs = f;
    f += (size_t)m * (size_t)m;
    kf->work_k = f;
    f += (size_t)n * (size_t)m;
    kf->work_ky = f;
    f += n;
    kf->work_kh = f;
    f += (size_t)n * (size_t)n;
    kf->work_i_minus_kh = f;
    f += (size_t)n * (size_t)n;

    memset(f_start, 0, (size_t)((uint8_t *)f - (uint8_t *)f_start));
}

// === public allocators ===
kalman_filter_state_t *kalman_init(uint8_t dim_x, uint8_t dim_z) {
    if (dim_x == 0u || dim_z == 0u)
        return NULL;
    size_t need = kalman_bytes(dim_x, dim_z);
    void *blk = malloc(need);
    if (!blk)
        return NULL;
    memset(blk, 0, need);
    kalman_filter_state_t *kf = (kalman_filter_state_t *)blk;
    kf->dim_x = dim_x;
    kf->dim_z = dim_z;
    kf->owned_block = blk;
    kalman_assign_ptrs(kf);
    return kf;
}

kalman_filter_state_t *
kalman_init_inplace(void *mem, size_t bytes, uint8_t dim_x, uint8_t dim_z) {
    if (!mem || dim_x == 0u || dim_z == 0u)
        return NULL;
    size_t need = kalman_bytes(dim_x, dim_z);
    if (bytes < need)
        return NULL;
    memset(mem, 0, need);
    kalman_filter_state_t *kf = (kalman_filter_state_t *)mem;
    kf->dim_x = dim_x;
    kf->dim_z = dim_z;
    kf->owned_block = NULL;
    kalman_assign_ptrs(kf);
    return kf;
}

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

void kalman_deinit(kalman_filter_state_t *kf) {
    if (!kf)
        return;
    if (kf->owned_block) {
        free(kf->owned_block);
    }
}

void kalman_cleanup(void *v) {
    kalman_deinit(*(kalman_filter_state_t **)v);
}

// === model setup ===
void kalman_prepare_cv_model(kalman_filter_state_t *kf, float dt, float q) {
    uint8_t n = kf->dim_x;
    if (n < 2)
        return;

    if (dt <= 0.0f)
        dt = 1e-3f;
    if (q <= 0.0f)
        q = 1e-4f;

    float32_t dt2 = dt * dt;
    float32_t dt3 = dt2 * dt;
    float32_t dt4 = dt3 * dt;

    // F = I
    for (uint8_t i = 0; i < (uint8_t)(n * n); i++)
        kf->state_transition_data[i] = 0.0f;
    for (uint8_t i = 0; i < n; i++)
        kf->state_transition_data[i * n + i] = 1.0f;

    // link velocity terms
    for (uint8_t i = 0; i + 1 < n; i += 2) {
        kf->state_transition_data[i * n + (i + 1)] = dt;
    }

    // Q
    for (uint8_t i = 0; i < (uint8_t)(n * n); i++)
        kf->process_noise_data[i] = 0.0f;

    for (uint8_t blk = 0; blk + 1 < n; blk += 2) {
        uint8_t i = blk;
        uint8_t j = blk + 1;
        kf->process_noise_data[i * n + i] = q * dt4 / 4.0f;
        kf->process_noise_data[i * n + j] = q * dt3 / 2.0f;
        kf->process_noise_data[j * n + i] = q * dt3 / 2.0f;
        kf->process_noise_data[j * n + j] = q * dt2;
    }
}

// === predict ===
void kalman_predict(kalman_filter_state_t *kf) {
    uint8_t n = kf->dim_x;
    if (n == 0)
        return;

    arm_matrix_instance_f32 state, cov, F, Q, x_pred, FP, Ft, P_pred, P_new;

    arm_mat_init_f32(&state, n, 1, kf->state_data);
    arm_mat_init_f32(&cov, n, n, kf->covariance_data);
    arm_mat_init_f32(&F, n, n, kf->state_transition_data);
    arm_mat_init_f32(&Q, n, n, kf->process_noise_data);

    // x = F*x
    arm_mat_init_f32(&x_pred, n, 1, kf->work_x_pred);
    (void)arm_mat_mult_f32(&F, &state, &x_pred);
    for (uint8_t i = 0; i < n; i++)
        kf->state_data[i] = kf->work_x_pred[i];

    // P = FPF^T + Q
    arm_mat_init_f32(&FP, n, n, kf->work_fp);
    (void)arm_mat_mult_f32(&F, &cov, &FP);

    arm_mat_init_f32(&Ft, n, n, kf->work_ft);
    (void)arm_mat_trans_f32(&F, &Ft);

    arm_mat_init_f32(&P_pred, n, n, kf->work_p_pred);
    (void)arm_mat_mult_f32(&FP, &Ft, &P_pred);

    arm_mat_init_f32(&P_new, n, n, kf->work_p_new);
    (void)arm_mat_add_f32(&P_pred, &Q, &P_new);

    memcpy(kf->covariance_data, kf->work_p_new, sizeof(float32_t) * n * n);
}

// === update ===
void kalman_update(kalman_filter_state_t *kf, const float32_t *measured) {
    uint8_t n = kf->dim_x;
    uint8_t m = kf->dim_z;
    if (n == 0 || m == 0)
        return;

    arm_matrix_instance_f32 state, cov, H, R;
    arm_matrix_instance_f32 Ht, PHt, HPHt, S, invS_mat, K;
    arm_matrix_instance_f32 innovation, Hx, Ky;
    arm_matrix_instance_f32 KH, I_minus_KH, P_new;

    arm_mat_init_f32(&state, n, 1, kf->state_data);
    arm_mat_init_f32(&cov, n, n, kf->covariance_data);
    arm_mat_init_f32(&H, m, n, kf->measurement_matrix_data);
    arm_mat_init_f32(&R, m, m, kf->measurement_noise_data);

    // y = z - Hx
    arm_mat_init_f32(&Hx, m, 1, kf->work_hx);
    (void)arm_mat_mult_f32(&H, &state, &Hx);
    for (uint8_t i = 0; i < m; i++)
        kf->work_innovation[i] = measured[i] - kf->work_hx[i];
    arm_mat_init_f32(&innovation, m, 1, kf->work_innovation);

    // S = HPH^T + R
    arm_mat_init_f32(&Ht, n, m, kf->work_ht);
    (void)arm_mat_trans_f32(&H, &Ht);

    arm_mat_init_f32(&PHt, n, m, kf->work_pht);
    (void)arm_mat_mult_f32(&cov, &Ht, &PHt);

    arm_mat_init_f32(&HPHt, m, m, kf->work_hph_t);
    (void)arm_mat_mult_f32(&H, &PHt, &HPHt);

    arm_mat_init_f32(&S, m, m, kf->work_s);
    (void)arm_mat_add_f32(&HPHt, &R, &S);

    // inv(S)
    arm_mat_init_f32(&invS_mat, m, m, kf->work_invs);
    arm_status inv_stat = arm_mat_inverse_f32(&S, &invS_mat);
    if (inv_stat != ARM_MATH_SUCCESS)
        return;

    // K = PH^T inv(S)
    arm_mat_init_f32(&K, n, m, kf->work_k);
    (void)arm_mat_mult_f32(&PHt, &invS_mat, &K);

    // x = x + Ky
    arm_mat_init_f32(&Ky, n, 1, kf->work_ky);
    (void)arm_mat_mult_f32(&K, &innovation, &Ky);
    for (uint8_t i = 0; i < n; i++)
        kf->state_data[i] += kf->work_ky[i];

    // P = (I - KH) P
    arm_mat_init_f32(&KH, n, n, kf->work_kh);
    (void)arm_mat_mult_f32(&K, &H, &KH);

    float32_t *ImKH = kf->work_i_minus_kh;
    for (uint8_t i = 0; i < (uint8_t)(n * n); i++)
        ImKH[i] = 0.0f;
    for (uint8_t i = 0; i < n; i++)
        ImKH[i * n + i] = 1.0f;
    for (uint8_t i = 0; i < (uint8_t)(n * n); i++)
        ImKH[i] -= kf->work_kh[i];

    arm_mat_init_f32(&I_minus_KH, n, n, ImKH);
    arm_mat_init_f32(&P_new, n, n, kf->work_p_new);
    (void)arm_mat_mult_f32(&I_minus_KH, &cov, &P_new);
    memcpy(kf->covariance_data, kf->work_p_new, sizeof(float32_t) * n * n);
}
