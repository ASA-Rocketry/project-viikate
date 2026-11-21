#pragma once
#include "guidance/kalman.h"

typedef enum {
    Boot,
    Safe,
    Pad,
    Launch,
    Guidance,
    Fault,
    Recovery,
    Landing,
    Abort,
} FLIGHTSTATE;

typedef struct {
    FLIGHTSTATE previous_state;
    FLIGHTSTATE current_state;
    float pad_altitude;  // m
    int64_t checklist_time;  // ms
    int64_t liftoff_time;  // ms
    int64_t fault_activation_time;
    float landing_reference_altitude;  // m
    int64_t landing_stable_since;  // ....

} state_machine;

typedef struct {
    FLIGHTSTATE state;
    float pad_altitude;
    float flight_time_s;
} flight_record;

FLIGHTSTATE state_machine_update(
    const kalman_filter_t *k,
    const imu_sample_t *is,
    bool controls_checked
);
