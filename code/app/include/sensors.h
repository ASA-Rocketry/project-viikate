#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

typedef struct {
    uint64_t timestamp_ns;
    float ax, ay, az;
    float gx, gy, gz;
} imu_sample_t;

typedef struct {
    uint64_t timestamp_ns;
    float temperature;
    float pressure;
} barometer_sample_t;

extern struct k_msgq imu_msgq;
extern struct k_msgq barometer_msgq;

void task_barometer_read(void *arg1, void *arg2, void *arg3);

int imu_init(const struct device *imu);
int barometer_init(const struct device *barometer);
