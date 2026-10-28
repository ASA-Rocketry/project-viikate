#pragma once

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "math/linmath.h"

typedef struct {
    uint64_t timestamp_ns;
    vec3 acceleration;
    vec3 angular_rate;
} imu_sample_t;

extern struct k_msgq imu_msgq;

int imu_init(const struct device *imu);
