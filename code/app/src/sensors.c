#include "sensors.h"

#include <stdalign.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>

#include "math/linmath.h"
#include "zephyr/device.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"

#define IMU_QUEUE_SIZE 64
#define BAROMETER_QUEUE_SIZE 16
#define BAROMETER_TASK_FREQUENCY_HZ 10
#define BAROMETER_TASK_PERIOD (1000 / BAROMETER_TASK_FREQUENCY_HZ)

#define TASK_RBF_STACK_SIZE 1024
#define TASK_RBF_PRIORITY 3
#define RBF_TASK_PERIOD_MS 25
#define RBF_SWITCH_SAMPLES \
    50  // Similar consequent samples required. Avoid switch bounce.

K_MSGQ_DEFINE(
    imu_msgq,
    sizeof(imu_sample_t),
    IMU_QUEUE_SIZE,
    alignof(imu_sample_t)
);

K_MSGQ_DEFINE(
    barometer_msgq,
    sizeof(barometer_sample_t),
    BAROMETER_QUEUE_SIZE,
    alignof(barometer_sample_t)
);

static void imu_trigger_handler(
    const struct device *dev,
    const struct sensor_trigger *trig
) {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    int rc;

    rc = sensor_sample_fetch(dev);
    if (rc) {
        return;  // or count errors
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc)
        return;

    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (rc)
        return;

    imu_sample_t s;
    s.timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());

    vec3 acceleration = {
        sensor_value_to_float(&accel[0]),
        sensor_value_to_float(&accel[1]),
        sensor_value_to_float(&accel[2])
    };

    vec3 angular_acceleration = {
        sensor_value_to_float(&gyro[0]),
        sensor_value_to_float(&gyro[1]),
        sensor_value_to_float(&gyro[2])
    };

    vec3_dup(s.acceleration, acceleration);
    vec3_dup(s.angular_rate, angular_acceleration);

    // DDR(Jozef): this drops the old sample if we run out of space which should be fine for the IMU
    while (k_msgq_put(&imu_msgq, &s, K_NO_WAIT) != 0) {
        k_msgq_purge(&imu_msgq);
    }
}

int imu_init(const struct device *imu) {
    int ret;

    ret = device_is_ready(imu);
    if (ret == 0) {
        printk("IMU is not ready: %d!\n", ret);
        return ret;
    }

    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };

    ret = sensor_trigger_set(imu, &trig, imu_trigger_handler);
    if (ret != 0) {
        printk("Trigger set for IMU failed: %d\n", ret);
        return ret;
    }

    printk("IMU initialized successfully\n");

    return 0;
}

void task_barometer_read(void *arg1, void *arg2, void *arg3) {
    struct device *barometer = *(struct device **)arg1;
    (void)arg2;
    (void)arg3;

    while (1) {
        uint64_t start_time = k_uptime_get();

        int rc;

        rc = sensor_sample_fetch(barometer);
        if (rc) {
            printk("Barometer fetch failed: %d\n", rc);
            k_msleep(1);
            continue;
        }

        struct sensor_value pressure;
        struct sensor_value temperature;
        rc = sensor_channel_get(barometer, SENSOR_CHAN_PRESS, &pressure);
        rc = sensor_channel_get(
            barometer,
            SENSOR_CHAN_AMBIENT_TEMP,
            &temperature
        );
        if (rc) {
            printk("Barometer decode failed: %d\n", rc);
            k_msleep(1);
            continue;
        }

        uint64_t elapsed = k_uptime_get() - start_time;

        barometer_sample_t s = {
            .timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks()),
            .pressure = sensor_value_to_float(&pressure),
            .temperature = sensor_value_to_float(&temperature)
        };

        k_msgq_put(&barometer_msgq, &s, K_NO_WAIT);

        if (elapsed < BAROMETER_TASK_PERIOD) {
            k_msleep(BAROMETER_TASK_PERIOD - elapsed);
        }
    }
}

int barometer_init(const struct device *barometer) {
    int ret;

    ret = device_is_ready(barometer);
    if (ret == 0) {
        printk("Barometer is not ready: %d!\n", ret);
        k_msleep(10);
        device_init(barometer);
        ret = device_is_ready(barometer);
        return ret;
    }

    printk("Barometer initialized successfuly\n");

    return 0;
}

static const struct gpio_dt_spec rbf = GPIO_DT_SPEC_GET(DT_ALIAS(rbf), gpios);

static bool rbf_removed = false;
static bool rbf_last_raw = false;
static int rbf_same_count = 0;

static bool rbf_read_raw(void) {
    int v = gpio_pin_get_dt(&rbf);
    if (v < 0) {
        return false;
    }
    return v == 0;
}

int rbf_init(void) {
    int ret;

    if (!gpio_is_ready_dt(&rbf)) {
        printk("RBF gpio not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&rbf, GPIO_INPUT);
    if (ret != 0) {
        printk("RBF configure failed\n");
        return 0;
    }

    k_busy_wait(2000);  // Time for pull up to settle
    rbf_removed = rbf_read_raw();
    rbf_last_raw = rbf_removed;

    printk("RBF initialized successfully\n");

    return ret;
}

static void rbf_update(void) {
    bool raw = rbf_read_raw();

    if (raw == rbf_last_raw) {
        if (rbf_same_count < RBF_SWITCH_SAMPLES) {
            rbf_same_count++;
        }
    } else {
        rbf_last_raw = raw;
        rbf_same_count = 1;
    }

    if (raw != rbf_removed && rbf_same_count >= RBF_SWITCH_SAMPLES) {
        rbf_removed = raw;
    }
}

bool rbf_is_removed(void) {
    return rbf_removed;
}

static void task_rbf(void *arg1, void *arg2, void *arg3) {
    (void)arg1;
    (void)arg2;
    (void)arg3;

    while (1) {
        rbf_update();
        k_msleep(RBF_TASK_PERIOD_MS);
    }
}

K_THREAD_DEFINE(
    task_rbf_thread_id,
    TASK_RBF_STACK_SIZE,
    task_rbf,
    NULL,
    NULL,
    NULL,
    TASK_RBF_PRIORITY,
    0,
    0
);