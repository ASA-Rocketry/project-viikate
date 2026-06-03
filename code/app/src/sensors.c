#include "sensors.h"

#include <stdalign.h>
#include <zephyr/drivers/sensor.h>

#include "zephyr/device.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"

#define IMU_QUEUE_SIZE 64
#define BAROMETER_QUEUE_SIZE 16
#define BAROMETER_TASK_FREQUENCY_HZ 10
#define BAROMETER_TASK_PERIOD (1000 / BAROMETER_TASK_FREQUENCY_HZ)

K_MSGQ_DEFINE(
    imu_msgq,
    sizeof(imu_sample_t),
    IMU_QUEUE_SIZE,
    alignof(imu_sample_t)
);

K_MSGQ_DEFINE(
    barometer_msgq,
    sizeof(barometer_sample_t),
    IMU_QUEUE_SIZE,
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

    s.ax = sensor_value_to_float(&accel[0]);
    s.ay = sensor_value_to_float(&accel[1]);
    s.az = sensor_value_to_float(&accel[2]);

    s.gx = sensor_value_to_float(&gyro[0]);
    s.gy = sensor_value_to_float(&gyro[1]);
    s.gz = sensor_value_to_float(&gyro[2]);

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

    printk("IMU initialized successfuly\n");

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
        if (ret == 0)
            return 0;
        return ret;
    }

    printk("Barometer initialized successfuly\n");

    return 0;
}
