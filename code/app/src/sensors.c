#include "sensors.h"

#include <errno.h>
#include <stdalign.h>
#include <zephyr/drivers/sensor.h>

#include "math/linmath.h"
#include "zephyr/device.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"

#define IMU_QUEUE_SIZE 64

K_MSGQ_DEFINE(
    imu_msgq,
    sizeof(imu_sample_t),
    IMU_QUEUE_SIZE,
    alignof(imu_sample_t)
);

static void imu_trigger_handler(
    const struct device *dev,
    const struct sensor_trigger *trig
) {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    int rc;
    (void)trig;

    rc = sensor_sample_fetch(dev);
    if (rc) {
        printk("IMU sample fetch failed: %d\n", rc);
        return;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc) {
        printk("IMU acceleration read failed: %d\n", rc);
        return;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (rc) {
        printk("IMU gyroscope read failed: %d\n", rc);
        return;
    }

    imu_sample_t s;
    s.timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());

    vec3 acceleration = {
        sensor_value_to_float(&accel[0]),
        sensor_value_to_float(&accel[1]),
        sensor_value_to_float(&accel[2])
    };

    vec3 angular_rate = {
        sensor_value_to_float(&gyro[0]),
        sensor_value_to_float(&gyro[1]),
        sensor_value_to_float(&gyro[2])
    };

    vec3_dup(s.acceleration, acceleration);
    vec3_dup(s.angular_rate, angular_rate);

    while (k_msgq_put(&imu_msgq, &s, K_NO_WAIT) != 0) {
        k_msgq_purge(&imu_msgq);
    }
}

int imu_init(const struct device *imu) {
    int ret;

    if (!device_is_ready(imu)) {
        printk("IMU is not ready\n");
        return -ENODEV;
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
