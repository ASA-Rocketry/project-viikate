#include "guidance/attitude.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "math/linmath.h"

#define GRAVITY_CORRECTION_GAIN 2.0f
#define MAX_FILTER_DT 0.05f
#define ACCELERATION_GATE_FRACTION 0.25f

static void normalize_vec3(vec3 result, const vec3 value) {
    float length = vec3_len(value);
    if (length > 1e-6f) {
        vec3_scale(result, value, 1.0f / length);
    } else {
        result[0] = 0.0f;
        result[1] = 0.0f;
        result[2] = 0.0f;
    }
}

static void quaternion_from_gravity(quat result, const vec3 acceleration) {
    vec3 measured_up;
    normalize_vec3(measured_up, acceleration);
    if (vec3_len(measured_up) < 1e-6f) {
        quat_identity(result);
        return;
    }

    const vec3 world_up = {0.0f, 0.0f, 1.0f};
    float dot = vec3_mul_inner(measured_up, world_up);
    if (dot < -0.999999f) {
        result[0] = 1.0f;
        result[1] = 0.0f;
        result[2] = 0.0f;
        result[3] = 0.0f;
        return;
    }

    vec3_mul_cross(result, measured_up, world_up);
    result[3] = 1.0f + dot;
    quat_norm(result, result);
}

attitude_filter_t attitude_filter_init(
    const vec3 initial_acceleration,
    const vec3 gyroscope_bias,
    uint64_t timestamp_ns
) {
    attitude_filter_t filter = {0};
    quaternion_from_gravity(filter.orientation, initial_acceleration);
    vec3_dup(filter.gyroscope_bias, gyroscope_bias);
    filter.gravity = vec3_len(initial_acceleration);
    filter.last_measurement_timestamp_ns = timestamp_ns;
    return filter;
}

int attitude_filter_update(
    attitude_filter_t *filter,
    const imu_sample_t *sample
) {
    if (filter == NULL || sample == NULL) {
        return -EINVAL;
    }

    if (filter->last_measurement_timestamp_ns != 0
        && sample->timestamp_ns <= filter->last_measurement_timestamp_ns) {
        return 0;
    }

    for (size_t i = 0; i < 3; ++i) {
        if (!isfinite(sample->acceleration[i])
            || !isfinite(sample->angular_rate[i])) {
            return -ERANGE;
        }
    }

    float dt = filter->last_measurement_timestamp_ns == 0
        ? 0.001f
        : (float)(sample->timestamp_ns - filter->last_measurement_timestamp_ns)
            * 1e-9f;
    if (!isfinite(dt) || dt <= 0.0f) {
        return -ERANGE;
    }
    if (dt > MAX_FILTER_DT) {
        dt = MAX_FILTER_DT;
    }
    filter->last_measurement_timestamp_ns = sample->timestamp_ns;

    vec3 corrected_rate;
    vec3_sub(corrected_rate, sample->angular_rate, filter->gyroscope_bias);

    float acceleration_length = vec3_len(sample->acceleration);
    if (filter->gravity > 1e-6f
        && fabsf(acceleration_length - filter->gravity)
            <= filter->gravity * ACCELERATION_GATE_FRACTION) {
        vec3 measured_up;
        normalize_vec3(measured_up, sample->acceleration);

        quat inverse_orientation;
        quat_conj(inverse_orientation, filter->orientation);
        const vec3 world_up = {0.0f, 0.0f, 1.0f};
        vec3 predicted_up;
        quat_mul_vec3(predicted_up, inverse_orientation, world_up);

        vec3 correction;
        vec3_mul_cross(correction, measured_up, predicted_up);
        vec3_scale(correction, correction, GRAVITY_CORRECTION_GAIN);
        vec3_add(corrected_rate, corrected_rate, correction);
    }

    quat angular_rate = {
        corrected_rate[0],
        corrected_rate[1],
        corrected_rate[2],
        0.0f
    };
    quat derivative;
    quat_mul(derivative, filter->orientation, angular_rate);
    quat_scale(derivative, derivative, 0.5f * dt);
    quat_add(filter->orientation, filter->orientation, derivative);
    quat_norm(filter->orientation, filter->orientation);
    vec3_dup(filter->angular_velocity, corrected_rate);
    return 1;
}
