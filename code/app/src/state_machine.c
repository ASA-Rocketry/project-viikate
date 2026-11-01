#include "state_machine.h"

#include <assert.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

#include "guidance/kalman.h"

#define GYRO_BIAS_MAX 1.0f  // rad/s
#define ACCELERATION_BIAS_MAX 0.5f  // m/s^2
#define LAUNCH_ACCEL_THRESHOLD \
    16.0f  // m/s^2 (Absolute value including gravity)
#define LAUNCH_ALT_THRESHOLD 25.0f  // meters (Above launch pad)
#define GUIDANCE_ALT_START 30.0f  // meters (Above launch pad)
#define RECOVERY_DEPLOYMENT_TIME \
    20.0f  // s (When recovery is forced after launch. Should be apogee + some margin)
#define MAX_FLIGHT_TIME 240.0f  // s (When landing is forced after launch)
#define CHECKLIST_TIMEOUT 60.0f  // s (Checklist fails after this time)
#define FAULT_CONFIRMATION_TIME 1.0f  // s (Fault must be active for this long)
#define LANDING_ALTITUDE_TOLERANCE \
    0.25f  // m (How much altitude can fluctuate in landing state)
#define LANDING_STABLE_TIME \
    5.0f  // s (How long altitude should stay within tolerance for landing to be detected)
#define LANDING_SPEED_TOLERANCE \
    0.5f  // m/s (How much speed can fluctuate in landing state)
#define BATTERY_VOLTAGE_MIN 4.5f  // v TODO: NOT IN USE
#define GRAVITY_NOMINAL 9.81f  // m/s^2

static_assert(GYRO_BIAS_MAX > 0.0f, "Gyro bias max must be positive!");
static_assert(
    ACCELERATION_BIAS_MAX > 0.0f,
    "Acceleration bias max must be positive!"
);
static_assert(
    LAUNCH_ACCEL_THRESHOLD > GRAVITY_NOMINAL,
    "Launch acceleration threshold must exceed nominal gravity to be detectable!"
);
static_assert(
    LAUNCH_ALT_THRESHOLD < GUIDANCE_ALT_START,
    "Launch altitude threshold must be less than guidance start altitude!"
);
static_assert(
    RECOVERY_DEPLOYMENT_TIME < MAX_FLIGHT_TIME,
    "Recovery activation time must be less than total flight time!"
);
static_assert(CHECKLIST_TIMEOUT > 0.0f, "Checklist timeout must be positive!");
static_assert(
    FAULT_CONFIRMATION_TIME > 0.0f,
    "Fault confirmation time must be positive!"
);
static_assert(
    LANDING_ALTITUDE_TOLERANCE > 0.0f,
    "Landing altitude tolerance must be positive!"
);
static_assert(
    LANDING_STABLE_TIME > 0.0f,
    "Landing stable time must be positive!"
);
static_assert(
    LANDING_SPEED_TOLERANCE > 0.0f,
    "Landing speed tolerance must be positive!"
);

// Helpers
static float norm3(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float norm4(const float v[4]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
}

static state_machine machine = {
    .previous_state = Boot,
    .current_state = Boot,
    .pad_altitude = 0.0f,
    .checklist_time = 0.0f,
    .liftoff_time = 0.0f,
    .fault_entry_time = 0.0f,
    .landing_reference_altitude = 0.0f,
    .landing_stable_since = 0.0f,
    .landing_timer_running = false,

};

static bool safe_check(
    const kalman_filter_t *k,
    const imu_sample_t *is
) {  // TODO: Do we inted to boot with the RBF in?
    return fabsf(norm3(is->acceleration) - GRAVITY_NOMINAL)
        < LAUNCH_ACCEL_THRESHOLD
        && !rbf_is_removed()
        && k_ticks_to_ns_floor64(k_uptime_ticks()) * 1e-9f
        > 5.0f;  // Stabilizing after boot for 5s before system can proceed
}

static bool pad_check(const kalman_filter_t *k) {
    if (!rbf_is_removed()) {
        return false;
    }
    if (machine.checklist_time == 0.0f) {
        machine.checklist_time = k_ticks_to_ns_floor64(k_uptime_ticks());
    }
    if (norm3(k->accelerometer_bias) > ACCELERATION_BIAS_MAX
        || norm3(k->gyroscope_bias) > GYRO_BIAS_MAX) {
        printk("IMU BIAS TOO HIGH");
        return false;
    }
    if (0) {  // TODO: Orientation check
        printk("ORIENTATION FAIL");
        return false;
    }
    if (0) {  // TODO: battery measurement at the launchpad
        printk("BATTERY FAIL");
        return false;
    }

    if (0) {  // TODO: sd card inserted
        printk("SD CARD FAIL");
        return false;
    }

    return true;
}

static bool launch_check(const kalman_filter_t *k, const imu_sample_t *is) {
    return (
        (k->position[1] > machine.pad_altitude + LAUNCH_ALT_THRESHOLD)
        || norm3(is->acceleration) > LAUNCH_ACCEL_THRESHOLD
    );
}

static bool guidance_check(const kalman_filter_t *k) {
    return (k->position[1] > machine.pad_altitude + GUIDANCE_ALT_START);
}

static bool fault_check(const kalman_filter_t *k, const imu_sample_t *is) {
    // Finite values:
    for (int i = 0; i < 3; i++) {
        if (!isfinite(k->position[i]) || !isfinite(k->velocity[i])
            || !isfinite(k->angular_velocity[i])
            || !isfinite(k->accelerometer_bias[i])
            || !isfinite(k->gyroscope_bias[i])) {
            return true;
        }
    }
    if (!isfinite(k->orientation[0]) || !isfinite(k->orientation[1])
        || !isfinite(k->orientation[2]) || !isfinite(k->orientation[3])
        || !isfinite(k->gravity)) {
        return true;
    }

    // Quaternion scale:
    float qnormalized = norm4(k->orientation);
    if (qnormalized < 0.95f || qnormalized > 1.05f) {
        return true;
    }

    // Time from last measurement:
    if (machine.current_state > Safe
        && (k->last_measurement_timestamp_ns != 0
            && (k_ticks_to_ns_floor64(k_uptime_ticks())
                - k->last_measurement_timestamp_ns)
                > 100000000ULL)) {  // Must measure atleast every 100 ms
        return true;
    }

    // Speed reasonable:
    if (machine.current_state >= Launch && norm3(k->velocity) > 300.0f) {
        return true;
    }

    // Booting in flight
    if (machine.current_state == Boot
        && norm3(is->acceleration) > LAUNCH_ACCEL_THRESHOLD
        && rbf_is_removed()) {
        return true;
    }
    return false;
}

static bool recovery_check(const kalman_filter_t *k) {
    const int64_t now_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
    static int64_t start_time = 0.0f;
    static bool timing_active = false;

    if ((k->position[1] > (machine.pad_altitude + LAUNCH_ALT_THRESHOLD)
         && k->velocity[1] < 0.0f && rbf_is_removed())) {
        if (!timing_active) {
            timing_active = true;
            start_time = now_ns;
        }
    } else {
        timing_active = false;
        return false;
    }
    return (
        (now_ns - start_time) * 1e-9f >= 1.0f
    );  // Recovery activates when conditions apply for 1 second
}

static bool landing_check(const kalman_filter_t *k, const imu_sample_t *is) {
    const int64_t now_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
    const float altitude = k->position[1];
    const float altitude_difference =
        fabsf(altitude - machine.landing_reference_altitude);
    const float stable_time =
        (float)(now_ns - machine.landing_stable_since) * 1e-9f;

    if (!machine.landing_timer_running) {
        machine.landing_reference_altitude = altitude;
        machine.landing_stable_since = now_ns;
        machine.landing_timer_running = true;
        return false;
    }

    if (altitude_difference > LANDING_ALTITUDE_TOLERANCE
        || norm3(k->velocity) > LANDING_SPEED_TOLERANCE) {
        machine.landing_timer_running = false;
        return false;
    }

    // Confirm landing only after altitude has remained stable and total speed is sufficiently low
    return (
        stable_time >= LANDING_STABLE_TIME
        && norm3(k->velocity) < LANDING_SPEED_TOLERANCE
    );  // TODO: Acceleration here
}

static void entry_action(
    const FLIGHTSTATE entered_state,
    const kalman_filter_t *k,
    const int64_t now_ns
) {
    switch (entered_state) {
        case Safe:
            break;
        case Pad:
            machine.pad_altitude = (float)k->position[1];
            break;
        case Launch:
            if (machine.liftoff_time == 0.0) {
                machine.liftoff_time = now_ns;
            }
            break;
        case Guidance:
            break;
        case Recovery:
            break;
        case Landing:
            break;
        case Fault:
            machine.fault_entry_time = now_ns;
            break;
        case Abort:
            break;
        default:
            break;
    }
}

FLIGHTSTATE
state_machine_update(const kalman_filter_t *k, const imu_sample_t *is) {
    const int64_t now_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
    const float time_flight = (float)(now_ns - machine.liftoff_time) * 1e-9f;

    FLIGHTSTATE temp_state = machine.current_state;
    switch (machine.current_state) {
        case Boot:
            if (safe_check(k, is)) {
                machine.current_state = Safe;
            }
            break;
        case Safe:
            if (pad_check(k)) {
                machine.current_state = Pad;
            } else if (machine.checklist_time != 0.0
                       && (now_ns - machine.checklist_time) * 1e-9f
                           >= CHECKLIST_TIMEOUT) {
                machine.current_state = Abort;
            }
            break;
        case Pad:
            if (launch_check(k, is)) {
                machine.current_state = Launch;
            }
            break;
        case Launch:
            if (guidance_check(k)) {
                machine.current_state = Guidance;
            }
            break;
        case Guidance:
            if (recovery_check(k)) {
                machine.current_state = Recovery;
            }
            break;
        case Recovery:
            if (landing_check(k, is)) {
                machine.current_state = Landing;
            }
            break;
        case Landing:
            break;
        case Fault:  // Vehicle is made safe
            if (machine.liftoff_time == 0.0f
                && norm3(is->acceleration) < LAUNCH_ACCEL_THRESHOLD) {
                machine.current_state = Abort;
            } else {
                if (time_flight > MAX_FLIGHT_TIME) {
                    machine.current_state = Landing;
                } else if (recovery_check(k)) {
                    machine.current_state = Recovery;
                }  // Only activate recovery when speed allows
            }
            break;
        case Abort:
            break;
        default:
            machine.current_state = Fault;
            break;
    }

    // Time based transitions:
    if (machine.current_state == Launch || machine.current_state == Guidance
        || machine.current_state == Recovery) {
        if (time_flight >= RECOVERY_DEPLOYMENT_TIME
            && time_flight < MAX_FLIGHT_TIME) {
            machine.current_state = Recovery;
        } else if (time_flight >= MAX_FLIGHT_TIME) {
            machine.current_state = Landing;
        }
    }

    if (machine.current_state > temp_state) {
        machine.previous_state = temp_state;
        entry_action(machine.current_state, k, now_ns);
    }

    if (machine.current_state != Fault && machine.current_state != Abort
        && machine.current_state != Safe && machine.current_state != Boot
        && machine.current_state != Recovery
        && machine.current_state
            != Landing) {  // Safe and terminal states never go to fault.
        if (fault_check(k, is)) {
            if (machine.fault_activation_time == 0.0f) {
                machine.fault_activation_time = now_ns;
            } else if ((now_ns - machine.fault_activation_time)
                       >= FAULT_CONFIRMATION_TIME * 1e9f) {
                machine.fault_activation_time = 0.0f;
                machine.previous_state = machine.current_state;
                machine.current_state = Fault;
                entry_action(machine.current_state, k, now_ns);
            }
        } else {
            machine.fault_activation_time =
                0.0f;  // Fault disappeared. Reset time.
        }
    } else {
        machine.fault_activation_time =
            0.0f;  // No fault checking necessary in current state. Reset time.
    }
    return machine.current_state;
}