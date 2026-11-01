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
    uint64_t checklist_time;  // ms
    uint64_t liftoff_time;  // ms
    uint64_t fault_entry_time;  // ms
    float landing_reference_altitude;  // m
    uint64_t landing_stable_since;  // ....
    uint64_t fault_activation_time;
    bool landing_timer_running;

} state_machine;

FLIGHTSTATE
state_machine_update(const kalman_filter_t *k, const imu_sample_t *is);
