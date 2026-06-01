#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define IMU_NODE DT_NODELABEL(imu)
#define BAROMETER_NODE DT_NODELABEL(barometer)

static const struct device *imu = DEVICE_DT_GET(IMU_NODE);
static const struct device *barometer = DEVICE_DT_GET(BAROMETER_NODE);

static void imu_trigger_handler(
    const struct device *dev,
    const struct sensor_trigger *trig
) {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    int rc;

    rc = sensor_sample_fetch(dev);
    if (rc) {
        printk("Fetch failed: %d\n", rc);
        return;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc) {
        printk("Accel read failed: %d\n", rc);
        return;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (rc) {
        printk("Gyro read failed: %d\n", rc);
        return;
    }

    printk(
        "Accel [m/s²]: X=%d.%06d Y=%d.%06d Z=%d.%06d | "
        "Gyro [deg/s]: X=%d.%06d Y=%d.%06d Z=%d.%06d\n",
        accel[0].val1,
        accel[0].val2,
        accel[1].val1,
        accel[1].val2,
        accel[2].val1,
        accel[2].val2,
        gyro[0].val1,
        gyro[0].val2,
        gyro[1].val1,
        gyro[1].val2,
        gyro[2].val1,
        gyro[2].val2
    );
}

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 1

#define TASK_FREQUENCY_HZ 10
#define TASK_PERIOD_MS (1000 / TASK_FREQUENCY_HZ)

void periodic_task(void) {
    while (1) {
        uint64_t start_time = k_uptime_get();

        int rc;

        rc = sensor_sample_fetch(barometer);
        if (rc) {
            printk("Fetch failed: %d\n", rc);
            return;
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
            printk("Accel read failed: %d\n", rc);
            return;
        }

        printk(
            "Barometer (%lld): %d.%06d kPa, %d.%06d C\n",
            start_time,
            pressure.val1,
            pressure.val2,
            temperature.val1,
            temperature.val2
        );

        uint64_t elapsed = k_uptime_get() - start_time;
        if (elapsed < TASK_PERIOD_MS) {
            k_msleep(TASK_PERIOD_MS - elapsed);
        }
    }
}

K_THREAD_DEFINE(
    periodic_thread_id,
    MY_STACK_SIZE,
    periodic_task,
    NULL,
    NULL,
    NULL,
    MY_PRIORITY,
    0,
    0
);

int main(void) {
    int rc;

    if (!device_is_ready(imu)) {
        printk("ISM330DHCX not ready!\n");
        return 0;
    }

    if (!device_is_ready(barometer)) {
        printk("BME280 not ready!\n");
        return 0;
    }

    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };

    rc = sensor_trigger_set(imu, &trig, imu_trigger_handler);
    if (rc) {
        printk("Trigger set failed: %d\n", rc);
        return 0;
    }

    printk("ISM330DHCX trigger set — waiting for DATA_READY events...\n");

    /* Nothing else to do here; callback runs on interrupt. */
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
