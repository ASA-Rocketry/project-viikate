#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/time_units.h>

#include "guidance/attitude.h"
#include "guidance/pid.h"
#include "math/linmath.h"
#include "sensors.h"

#define IMU_NODE DT_NODELABEL(imu)
#define CALIBRATION_SAMPLES 256
#define CONTROL_PERIOD_NS 20000000ULL
#define IMU_TIMEOUT_NS 100000000ULL

static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);

static void transform_coords(imu_sample_t *sample) {
    vec3 acceleration;
    vec3 angular_rate;
    vec3_dup(acceleration, sample->acceleration);
    vec3_dup(angular_rate, sample->angular_rate);

    sample->acceleration[0] = -acceleration[2];
    sample->acceleration[1] = acceleration[1];
    sample->acceleration[2] = acceleration[0];

    sample->angular_rate[0] = -angular_rate[2];
    sample->angular_rate[1] = angular_rate[1];
    sample->angular_rate[2] = angular_rate[0];
}

static void quaternion_to_tilt(
    const quat orientation,
    float *roll,
    float *pitch
) {
    float x = orientation[0];
    float y = orientation[1];
    float z = orientation[2];
    float w = orientation[3];
    float sin_pitch = 2.0f * (w * y - z * x);

    *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    *pitch = asinf(sin_pitch);
}

int main(void) {
    int ret = attitude_control_init();
    if (ret != 0) {
        printk("Servo initialization failed: %d\n", ret);
        return ret;
    }

    ret = imu_init(imu);
    if (ret != 0) {
        printk("IMU initialization failed: %d\n", ret);
        return ret;
    }

    imu_sample_t sample;
    vec3 initial_acceleration = {0};
    vec3 gyro_bias = {0};

    printk("Keep the canard module upright and still for IMU calibration.\n");
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        ret = k_msgq_get(&imu_msgq, &sample, K_FOREVER);
        if (ret != 0) {
            printk("IMU calibration read failed: %d\n", ret);
            return ret;
        }

        transform_coords(&sample);
        vec3_add(
            initial_acceleration,
            initial_acceleration,
            sample.acceleration
        );
        vec3_add(gyro_bias, gyro_bias, sample.angular_rate);
    }

    vec3_scale(
        initial_acceleration,
        initial_acceleration,
        1.0f / CALIBRATION_SAMPLES
    );
    vec3_scale(gyro_bias, gyro_bias, 1.0f / CALIBRATION_SAMPLES);
    if (!isfinite(vec3_len(initial_acceleration))
        || vec3_len(initial_acceleration) < 1.0f) {
        printk("IMU calibration produced an invalid gravity measurement\n");
        return -ERANGE;
    }

    attitude_filter_t filter = attitude_filter_init(
        initial_acceleration,
        gyro_bias,
        sample.timestamp_ns
    );

    uint64_t last_control_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
    uint64_t last_status_ns = last_control_ns;
    uint64_t last_imu_ns = sample.timestamp_ns;
    bool imu_stale = false;

    while (1) {
        bool new_sample = false;
        while (k_msgq_get(&imu_msgq, &sample, K_NO_WAIT) == 0) {
            transform_coords(&sample);
            ret = attitude_filter_update(&filter, &sample);
            if (ret < 0) {
                printk(
                    "Invalid IMU sample rejected by attitude filter: %d\n",
                    ret
                );
                continue;
            }
            if (ret == 0) {
                continue;
            }
            last_imu_ns = sample.timestamp_ns;
            new_sample = true;
        }

        uint64_t now = k_ticks_to_ns_floor64(k_uptime_ticks());
        if (new_sample && imu_stale) {
            ret = attitude_control_init();
            if (ret != 0) {
                printk("Servo reinitialization failed: %d\n", ret);
            } else {
                imu_stale = false;
                last_control_ns = now;
            }
        }

        if (!imu_stale && now - last_imu_ns > IMU_TIMEOUT_NS) {
            ret = attitude_control_init();
            if (ret != 0) {
                printk(
                    "Failed to neutralize servos after IMU timeout: %d\n",
                    ret
                );
            } else {
                printk("IMU timeout; servos neutralized\n");
            }
            imu_stale = true;
        }

        if (new_sample && !imu_stale
            && now - last_control_ns >= CONTROL_PERIOD_NS) {
            float roll;
            float pitch;
            quaternion_to_tilt(filter.orientation, &roll, &pitch);
            float dt = (float)(now - last_control_ns) * 1e-9f;
            last_control_ns = now;

            ret = attitude_control_update(roll, pitch, dt);
            if (ret != 0) {
                printk("Servo update failed: %d\n", ret);
            }

            if (now - last_status_ns >= 250000000ULL) {
                printk(
                    "ATTITUDE roll=%.1f pitch=%.1f\n",
                    (double)(roll * 57.2957795f),
                    (double)(pitch * 57.2957795f)
                );
                last_status_ns = now;
            }
        }

        k_sleep(K_MSEC(1));
    }

    return 0;
}
