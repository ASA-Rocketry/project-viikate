#include <math.h>
#include <stdint.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "sensors.h"
#include "servo.h"

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

#define TASK_BAROMETER_READ_STACK_SIZE 1024
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

    while (1) {
        /* Read IMU if data is available */
        ret = k_msgq_get(&imu_msgq, &imu_sample, K_NO_WAIT);
        if (ret == 0) {
            printk(
                "[IMU]  ts=%llu | Acc=(%.3f, %.3f, %.3f)g | Gyro=(%.3f, %.3f, %.3f)dps\n",
                imu_sample.timestamp_ns,
                (double)imu_sample.ax,
                (double)imu_sample.ay,
                (double)imu_sample.az,
                (double)imu_sample.gx,
                (double)imu_sample.gy,
                (double)imu_sample.gz
            );
        }

        /* Read Barometer if data is available */
        ret = k_msgq_get(&barometer_msgq, &baro_sample, K_NO_WAIT);
        if (ret == 0) {
            printk(
                "[BARO] ts=%llu | Temp=%.2f°C | Press=%.2f hPa\n",
                baro_sample.timestamp_ns,
                (double)baro_sample.temperature,
                (double)baro_sample.pressure
            );
        }

        k_sleep(K_MSEC(500));  // Read roughly every 500ms
    }
}
