#pragma once

#include "guidance/kalman.h"

kalman_filter_state_t *kalman_init_baro(void);

kalman_filter_state_t *kalman_init_imu6dof(void);
void kalman_imu6dof_propagate(
    float32_t *x_next,
    const float32_t *x,
    const float32_t *u,
    float dt
);
