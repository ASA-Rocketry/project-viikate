/*
 * Simulated IMU and barometer. They expose the same Zephyr sensor API the
 * real ISM330DHCX and BME280 drivers do (fetch, channel_get, data-ready
 * trigger), with values pushed in by the plant bridge. Units follow the
 * Zephyr sensor API: m/s^2, rad/s, kPa, degC.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "sim_internal.h"

#define IMU_NODE DT_INST(0, viikate_sim_imu)
#define BARO_NODE DT_INST(0, viikate_sim_baro)

static double imu_accel[3];
static double imu_gyro[3];
static sensor_trigger_handler_t imu_handler;
static const struct sensor_trigger *imu_trigger;

static double baro_kpa;
static double baro_temp_c;

static int
sim_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    ARG_UNUSED(dev);
    ARG_UNUSED(chan);
    return 0;
}

static int imu_channel_get(
    const struct device *dev,
    enum sensor_channel chan,
    struct sensor_value *val
) {
    const double *src;

    ARG_UNUSED(dev);

    switch (chan) {
        case SENSOR_CHAN_ACCEL_XYZ:
            src = imu_accel;
            break;
        case SENSOR_CHAN_GYRO_XYZ:
            src = imu_gyro;
            break;
        default:
            return -ENOTSUP;
    }

    for (int i = 0; i < 3; i++) {
        sensor_value_from_double(&val[i], src[i]);
    }
    return 0;
}

static int imu_trigger_set(
    const struct device *dev,
    const struct sensor_trigger *trig,
    sensor_trigger_handler_t handler
) {
    ARG_UNUSED(dev);

    if (trig->type != SENSOR_TRIG_DATA_READY) {
        return -ENOTSUP;
    }
    imu_trigger = trig;
    imu_handler = handler;
    return 0;
}

static DEVICE_API(sensor, imu_api) = {
    .sample_fetch = sim_sample_fetch,
    .channel_get = imu_channel_get,
    .trigger_set = imu_trigger_set,
};

DEVICE_DT_DEFINE(
    IMU_NODE,
    NULL,
    NULL,
    NULL,
    NULL,
    POST_KERNEL,
    CONFIG_SENSOR_INIT_PRIORITY,
    &imu_api
);

void viikate_sim_imu_publish(
    const double accel_mps2[3],
    const double gyro_radps[3]
) {
    for (int i = 0; i < 3; i++) {
        imu_accel[i] = accel_mps2[i];
        imu_gyro[i] = gyro_radps[i];
    }
    if (imu_handler != NULL) {
        imu_handler(DEVICE_DT_GET(IMU_NODE), imu_trigger);
    }
}

static int baro_channel_get(
    const struct device *dev,
    enum sensor_channel chan,
    struct sensor_value *val
) {
    ARG_UNUSED(dev);

    switch (chan) {
        case SENSOR_CHAN_PRESS:
            return sensor_value_from_double(val, baro_kpa);
        case SENSOR_CHAN_AMBIENT_TEMP:
            return sensor_value_from_double(val, baro_temp_c);
        default:
            return -ENOTSUP;
    }
}

static DEVICE_API(sensor, baro_api) = {
    .sample_fetch = sim_sample_fetch,
    .channel_get = baro_channel_get,
};

DEVICE_DT_DEFINE(
    BARO_NODE,
    NULL,
    NULL,
    NULL,
    NULL,
    POST_KERNEL,
    CONFIG_SENSOR_INIT_PRIORITY,
    &baro_api
);

void viikate_sim_baro_set(double pressure_kpa, double temperature_c) {
    baro_kpa = pressure_kpa;
    baro_temp_c = temperature_c;
}
