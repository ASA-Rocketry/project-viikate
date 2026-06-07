#include <arm_math.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "dsp/controller_functions.h"
#include "dsp/filtering_functions.h"
#include "guidance/filters.h"
#include "guidance/kalman.h"
#include "guidance/pid.h"
#include "math/linmath.h"
#include "sensors.h"
#include "servo.h"
#include "zephyr/sys/time_units.h"

#define IMU_NODE DT_NODELABEL(imu)
#define BAROMETER_NODE DT_NODELABEL(barometer)

static const struct device *imu = DEVICE_DT_GET(IMU_NODE);
static const struct device *barometer = DEVICE_DT_GET(BAROMETER_NODE);

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

double pressure_to_altitude(double pressure_pa) {
    const double P0 = 101325.0;  // Sea level pressure (Pa)
    const double T0 = 288.15;  // Sea level temperature (K)
    const double L = 0.0065;  // Temperature lapse rate (K/m)
    const double g = 9.80665;  // Gravity (m/s²)
    const double R = 287.05;  // Specific gas constant (J/(kg·K))

    // Barometric formula: h = (T0 / L) * (1 - (P / P0)^(R*L/g))
    double exponent = (R * L) / g;
    double altitude = (T0 / L) * (1.0 - pow(pressure_pa / P0, exponent));

    return altitude;
}

void print_acceleration(const imu_sample_t *sample) {
    printf(
        "Acceleration [m/s²]: X=%f, Y=%f, Z=%f\n",
        (double)sample->acceleration[0],
        (double)sample->acceleration[1],
        (double)sample->acceleration[2]
    );
}

void transform_coords(imu_sample_t *imu_sample) {
    vec3 acc_old;
    vec3 gyro_old;
    vec3_dup(acc_old, imu_sample->acceleration);
    vec3_dup(gyro_old, imu_sample->angular_rate);

    imu_sample->acceleration[0] = -acc_old[2];
    imu_sample->acceleration[1] = acc_old[1];
    imu_sample->acceleration[2] = acc_old[0];

    imu_sample->angular_rate[0] = -gyro_old[2];
    imu_sample->angular_rate[1] = gyro_old[1];
    imu_sample->angular_rate[2] = gyro_old[0];
}

LINMATH_H_FUNC float quat_get_roll(quat const q) {
    float x = q[0];
    float y = q[1];
    float z = q[2];
    float w = q[3];

    float siny_cosp = 2.f * (w * z + x * y);
    float cosy_cosp = 1.f - 2.f * (y * y + z * z);

    return atan2f(siny_cosp, cosy_cosp);
}

int main(void) {
    barometer_init(barometer);
    imu_init(imu);

    imu_sample_t imu_sample;
    barometer_sample_t baro_sample;
    int ret;

    assert(imu_filter != NULL);

    float drift_last = 0.0f;

    vec3 avg_accel = {0}, avg_gyro = {0};

    vec3 g = {0};
#define AVG_COUNT 256
    for (int k_idx = 0; k_idx < AVG_COUNT; ++k_idx) {
        ret = k_msgq_get(&imu_msgq, &imu_sample, K_FOREVER);
        transform_coords(&imu_sample);
        vec3_add(g, g, imu_sample.acceleration);
    }
    vec3_scale(g, g, 1.0f / AVG_COUNT);

    kalman_filter_t k = kalman_init(g);

#define AVG_COUNT 256
    for (int k_idx = 0; k_idx < AVG_COUNT; ++k_idx) {
        ret = k_msgq_get(&imu_msgq, &imu_sample, K_FOREVER);
        transform_coords(&imu_sample);
        vec3 a;
        vec3_sub(a, imu_sample.acceleration, g);

        vec3_add(avg_accel, avg_accel, a);
        vec3_add(avg_gyro, avg_gyro, imu_sample.angular_rate);
    }

    vec3_scale(avg_accel, avg_accel, 1.0f / AVG_COUNT);
    vec3_scale(avg_gyro, avg_gyro, 1.0f / AVG_COUNT);

    vec3_dup(k.gyroscope_bias, avg_gyro);
    vec3_dup(k.accelerometer_bias, avg_accel);

    float base_altitude = 0.0f;
    for (int k_idx = 0; k_idx < 8; ++k_idx) {
        ret = k_msgq_get(&barometer_msgq, &baro_sample, K_FOREVER);

        base_altitude += pressure_to_altitude((double)baro_sample.pressure);
    }

    base_altitude *= 1.0f / 8;

    uint32_t loop_counter = 0;
    uint64_t time = 0;
    int loops = 0;

    roll_control_reset();

    while (1) {
        uint64_t start = k_ticks_to_ns_floor64(k_uptime_ticks());

        /* Read IMU if data is available */
        while ((ret = k_msgq_get(&imu_msgq, &imu_sample, K_NO_WAIT)) == 0) {
            transform_coords(&imu_sample);
            ekf_predict(&k, imu_sample);
            loops++;
        }

        time += k_ticks_to_ns_floor64(k_uptime_ticks()) - start;

        while ((ret = k_msgq_get(&barometer_msgq, &baro_sample, K_NO_WAIT))
               == 0) {
            float altitude = (float)pressure_to_altitude(baro_sample.pressure)
                - base_altitude;
            k.position[1] = altitude;
        }

        /* Print state every 500 iterations (~500 ms at 1 kHz) */
        if (loop_counter % 500 == 0) {
            printk(
                "loops: %d total_time: %f time_per_kalman: %f\n",
                loops,
                (double)((float)time * 1e-9f),
                (double)((float)time * 1e-9f / (float)(loops > 0 ? loops : 1))
            );
            printk(
                "STATE:\n"
                "    pos = [%.3f, %.3f, %.3f]\n"
                "    vel = [%.3f, %.3f, %.3f]\n"
                "    ori = [%.4f, %.4f, %.4f, %.4f]\n"
                "    ang_vel = [%.4f, %.4f, %.4f]\n"
                "    acc_bias = [%.4f, %.4f, %.4f]\n"
                "    gyro_bias = [%.4f, %.4f, %.4f]\n"
                "    g = %.4f\n"
                "    t_last = %llu ns\n",
                (double)k.position[0],
                (double)k.position[1],
                (double)k.position[2],
                (double)k.velocity[0],
                (double)k.velocity[1],
                (double)k.velocity[2],
                (double)k.orientation[0],
                (double)k.orientation[1],
                (double)k.orientation[2],
                (double)k.orientation[3],
                (double)k.angular_velocity[0],
                (double)k.angular_velocity[1],
                (double)k.angular_velocity[2],
                (double)k.accelerometer_bias[0],
                (double)k.accelerometer_bias[1],
                (double)k.accelerometer_bias[2],
                (double)k.gyroscope_bias[0],
                (double)k.gyroscope_bias[1],
                (double)k.gyroscope_bias[2],
                (double)k.gravity,
                (unsigned long long)k.last_measurement_timestamp_ns
            );
            loops = 0;
        }

        static uint64_t last_roll_update_ns;

        if (loop_counter % 20 == 0) {
            uint64_t now = k_ticks_to_ns_floor64(k_uptime_ticks());
            float dt = last_roll_update_ns == 0
                ? 0.02f
                : (float)(now - last_roll_update_ns) * 1e-9f;
            last_roll_update_ns = now;

            roll_control_update(quat_get_roll(k.orientation), 0.0f, dt);
        }

        loop_counter++;
        k_sleep(K_MSEC(1));  // 1 kHz update rate
    }
}
