#include <arm_math.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "dsp/filtering_functions.h"
#include "guidance/filters.h"
#include "guidance/kalman.h"
#include "sensors.h"
#include "servo.h"
#include "zephyr/sys/time_units.h"

#define IMU_NODE DT_NODELABEL(imu)
#define BAROMETER_NODE DT_NODELABEL(barometer)

static const struct device *imu = DEVICE_DT_GET(IMU_NODE);
static const struct device *barometer = DEVICE_DT_GET(BAROMETER_NODE);

static const servo_t servo0 = DT_SERVO_GET(DT_NODELABEL(servo0));
static const servo_t servo1 = DT_SERVO_GET(DT_NODELABEL(servo1));
static const servo_t servo2 = DT_SERVO_GET(DT_NODELABEL(servo2));
static const servo_t servo3 = DT_SERVO_GET(DT_NODELABEL(servo3));

#define STEP PWM_USEC(10)

enum direction {
    DOWN,
    UP,
};

#define TASK_BAROMETER_READ_STACK_SIZE 4096
#define TASK_BAROMETER_READ_PRIORITY 1

K_THREAD_DEFINE(
    task_barometer_read_thread_id,
    TASK_BAROMETER_READ_STACK_SIZE,
    task_barometer_read,
    &barometer,
    NULL,
    NULL,
    TASK_BAROMETER_READ_PRIORITY,
    0,
    0
);

static int sweep_servo(const servo_t servo) {
    float angle = -90.0f;  // start at minimum position
    enum direction dir = UP;
    int ret;

    printk("Servomotor control (angle based)\n");

    if (!pwm_is_ready_dt(&servo.spec)) {
        printk("Error: PWM device %s is not ready\n", servo.spec.dev->name);
        return -ENODEV;
    }

    while (1) {
        ret = servo_set_angle(servo, angle);
        if (ret < 0) {
            printk("Error %d: failed to set angle %.1f\n", ret, (double)angle);
            return ret;
        }

        // Step size = degrees per iteration
        const float step_size = 1.0f;

        if (dir == UP) {
            angle += step_size;
            if (angle >= 90.0f) {
                angle = 90.0f;
                dir = DOWN;
            }
        } else {
            angle -= step_size;
            if (angle <= -90.0f) {
                angle = -90.0f;
                dir = UP;
            }
        }

        k_sleep(K_MSEC(20));  // Smooth sweep speed
    }

    return 0;
}

int main(void) {
    barometer_init(barometer);
    imu_init(imu);

    imu_sample_t imu_sample;
    barometer_sample_t baro_sample;
    int ret;

    __attribute__((cleanup(kalman_cleanup)))
    kalman_filter_state_t *barometer_filter = kalman_init_baro();
    uint64_t timestamp_ns_last = 0;

    kalman_filter_state_t *imu_filter = kalman_init_imu6dof();
    assert(imu_filter != NULL);

    float drift_last = 0.0f;
    while (1) {
        /* Read IMU if data is available */
        uint64_t start = k_ticks_to_ns_floor64(k_uptime_ticks());
        int loops = 0;
        while ((ret = k_msgq_get(&imu_msgq, &imu_sample, K_NO_WAIT)) == 0) {
            float u[6] = {
                imu_sample.ax,
                imu_sample.ay,
                imu_sample.az,
                imu_sample.gx,
                imu_sample.gy,
                imu_sample.gz
            };

            static uint64_t last_time = 0;
            float dt = 0.0f;
            if (last_time == 0) {
                dt = 0.01f;
            } else {
                dt = (float)(imu_sample.timestamp_ns - last_time) * 1e-9f;
            }
            last_time = imu_sample.timestamp_ns;

            kalman_predict_ekf(imu_filter, u, dt, 1e-5f);
            //kalman_update(imu_filter, u);

            ++loops;
            //printk(
            //    "[IMU]  ts=%llu | Acc=(%.3f, %.3f, %.3f)g | Gyro=(%.3f, %.3f, %.3f)dps\n",
            //    imu_sample.timestamp_ns,
            //    (double)imu_sample.ax,
            //    (double)imu_sample.ay,
            //    (double)imu_sample.az,
            //    (double)imu_sample.gx,
            //    (double)imu_sample.gy,
            //    (double)imu_sample.gz
            //);
        }

        uint64_t time = k_ticks_to_ns_floor64(k_uptime_ticks()) - start;

        /* Read Barometer if data is available */
        while ((ret = k_msgq_get(&barometer_msgq, &baro_sample, K_NO_WAIT))
               == 0) {
            if (timestamp_ns_last != 0) {
                float dt = (float)(baro_sample.timestamp_ns - timestamp_ns_last)
                    * 1e-9f;  // for some reason dt shows up as 1.0 if its 10e-9f, not sure what that is about .timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks()),
                //printk("update %f\n", (double)dt);
                kalman_prepare_cv_model(
                    barometer_filter,
                    dt,
                    barometer_filter->process_noise_intensity
                );
            } else {
                kalman_prepare_cv_model(
                    barometer_filter,
                    1e-3f,
                    barometer_filter->process_noise_intensity
                );
            }
            kalman_predict(barometer_filter);
            timestamp_ns_last = baro_sample.timestamp_ns;
            //printk("pressure %f\n", (double)baro_sample.pressure);
            kalman_update(barometer_filter, &baro_sample.pressure);
        }

        printk(
            "loops: %d total_time: %f time_per_kalman: %f\n",
            loops,
            (double)((float)time * 1e-9f),
            (double)((float)time * 1e-9f / (float)loops)
        );
        printk(
            "[BARO] ts=%llu | Temp=%.2f°C | Press=%.6f kPa change %.6f\n",
            baro_sample.timestamp_ns,
            (double)baro_sample.temperature,
            (double)barometer_filter->state_data[0],
            (double)barometer_filter->state_data[1]
        );
        const float *x = imu_filter->state_data;
        {
            const float32_t qw = x[0], qx = x[1], qy = x[2], qz = x[3];

            // Axis magnitude = sin(theta/2)
            float32_t mag_v = 0.0f;
            const float32_t v[3] = {qx, qy, qz};
            arm_dot_prod_f32(v, v, 3u, &mag_v);
            arm_sqrt_f32(mag_v, &mag_v);

            float32_t angle_rad = 2.0f * atan2f(mag_v, qw);
            float32_t angle_deg = angle_rad * 57.2957795f;  // convert rad → deg

            float32_t axis[3] = {0.0f, 0.0f, 0.0f};

            if (mag_v > 1e-9f) {
                float32_t inv_mag = 1.0f / mag_v;
                axis[0] = qx * inv_mag;
                axis[1] = qy * inv_mag;
                axis[2] = qz * inv_mag;
            } else {
                // default axis if quaternion ≈ identity
                axis[0] = 0.0f;
                axis[1] = 0.0f;
                axis[2] = 1.0f;
                angle_deg = 0.0f;
            }

            printk(
                "[IMU ORI] ts=%llu | "
                "Axis=[%.4f %.4f %.4f] | Angle=%.2f drift %.2f° | "
                "Quat=[%.4f %.4f %.4f %.4f]\n",
                imu_sample.timestamp_ns,
                (double)axis[0],
                (double)axis[1],
                (double)axis[2],
                (double)angle_deg,
                (double)(angle_deg - drift_last) * 2.0,
                (double)qw,
                (double)qx,
                (double)qy,
                (double)qz
            );
            drift_last = angle_deg;
        }
        k_sleep(K_MSEC(500));  // Read roughly every 500ms
    }
}
