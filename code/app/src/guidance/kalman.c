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
    return k;
}
