#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensors.h"

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
    int64_t fault_entry_time;  // ms
    float landing_reference_altitude;  // m
    int64_t landing_stable_since;  // ....
    int64_t fault_activation_time;
    bool landing_timer_running;

} state_machine;

typedef struct {
    FLIGHTSTATE state;
    float pad_altitude;
    float flight_time_s;
} flight_record;

struct kalman_filter_s;  //Forward declare to avoid circular #include

FLIGHTSTATE
state_machine_update(const struct kalman_filter_s *k, const imu_sample_t *is);