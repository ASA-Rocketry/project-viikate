/*
 * Rocket plant: implements viikate_plant.h on top of the C generated from
 * simulation/matlab/plant/viikate_rocket.slx (simulation/plant_codegen).
 *
 * Timing: viikate_plant_init() runs the generated initialize and one
 * generated step with zero fin commands, so its outputs are the model
 * outputs at t = 0. Call k of viikate_plant_step() sets the fin commands,
 * runs one generated step and returns the outputs at t = k * 1 ms; those
 * commands act on the dynamics from that time on.
 *
 * Frames: the model's inertial frame has x up; see rocket_frames.h.
 * Position is reported relative to the pad (the position at t = 0).
 */

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "rocket_frames.h"
#include "viikate_plant.h"
#include "viikate_rocket.h"

#define STEP_S 0.001

typedef enum {
    RANGE_ANY,
    RANGE_NON_NEGATIVE,
    RANGE_POSITIVE,
    RANGE_SEED, /* whole number >= 0 */
} range_t;

static const struct {
    const char *name;
    double *dest;
    double sign; /* -1 where the model axis points the other way */
    range_t range;
} param_table[] = {
    {"ignition_time_s", &rp_ignition_time_s, 1.0, RANGE_NON_NEGATIVE},
    {"thrust_scale", &rp_thrust_scale, 1.0, RANGE_NON_NEGATIVE},
    {"mass_full_kg", &rp_mass_full_kg, 1.0, RANGE_POSITIVE},
    {"mass_fuel_kg", &rp_mass_fuel_kg, 1.0, RANGE_NON_NEGATIVE},
    /* rp_wind_model_mps is [up, east, north] */
    {"wind_north_mps", &rp_wind_model_mps[2], 1.0, RANGE_ANY},
    {"wind_east_mps", &rp_wind_model_mps[1], 1.0, RANGE_ANY},
    {"wind_down_mps", &rp_wind_model_mps[0], -1.0, RANGE_ANY},
    {"gust_variance", &rp_gust_variance, 1.0, RANGE_NON_NEGATIVE},
    {"gust_seed_1", &rp_gust_seed[0], 1.0, RANGE_SEED},
    {"gust_seed_2", &rp_gust_seed[1], 1.0, RANGE_SEED},
    {"servo_tau_s", &rp_servo_tau_s, 1.0, RANGE_POSITIVE},
    {"servo_rate_limit_radps", &rp_servo_rate_limit_radps, 1.0, RANGE_POSITIVE},
};

static struct {
    unsigned long steps;
    double pad_model_m[3];
    const char *error;
} plant;

const char *viikate_plant_name(void) {
    return "rocket_plant";
}

double viikate_plant_step_size_s(void) {
    return STEP_S;
}

static int in_range(double value, range_t range) {
    if (!isfinite(value)) {
        return 0;
    }
    switch (range) {
        case RANGE_NON_NEGATIVE:
            return value >= 0.0;
        case RANGE_POSITIVE:
            return value > 0.0;
        case RANGE_SEED:
            return value >= 0.0 && value == floor(value);
        default:
            return 1;
    }
}

int viikate_plant_set_param(const char *name, double value) {
    for (size_t i = 0; i < sizeof(param_table) / sizeof(param_table[0]); i++) {
        if (strcmp(name, param_table[i].name) == 0) {
            if (!in_range(value, param_table[i].range)) {
                return -2;
            }
            *param_table[i].dest = param_table[i].sign * value;
            return 0;
        }
    }
    return -1;
}

static void fill_outputs(viikate_plant_outputs_t *out) {
    const ExtY_viikate_rocket_T *y = &viikate_rocket_Y;
    double rel_model_m[3];

    memset(out, 0, sizeof(*out));
    for (int i = 0; i < 3; i++) {
        out->specific_force_body_mps2[i] = y->sf_body_mps2[i];
        out->angular_rate_body_radps[i] = y->omega_body_radps[i];
        out->truth.body_rate_radps[i] = y->omega_body_radps[i];
        out->truth.euler_rad[i] = y->euler_rad[i];
        rel_model_m[i] = y->pos_model_m[i] - plant.pad_model_m[i];
    }
    out->static_pressure_pa = rocket_isa_pressure_pa(rel_model_m[0]);
    out->air_temperature_c = rocket_isa_temperature_c(rel_model_m[0]);

    out->truth.t_s = (double)plant.steps * STEP_S;
    rocket_model_to_ned(rel_model_m, out->truth.position_ned_m);
    rocket_model_to_ned(y->vel_model_mps, out->truth.velocity_ned_mps);
    rocket_dcm_be_to_quat_ned(y->dcm_be, out->truth.quat_wxyz);
    for (int i = 0; i < VIIKATE_PLANT_FIN_COUNT; i++) {
        out->truth.fin_angle_rad[i] = y->fin_angle_rad[i];
    }
}

static void check_model_error(void) {
    const char *status = rtmGetErrorStatus(viikate_rocket_M);

    if (plant.error == NULL && status != NULL) {
        plant.error = status;
    }
}

void viikate_plant_init(viikate_plant_outputs_t *out) {
    memset(&plant, 0, sizeof(plant));
    if (rp_mass_fuel_kg >= rp_mass_full_kg) {
        plant.error = "mass_fuel_kg must be less than mass_full_kg";
    }

    viikate_rocket_initialize();
    memset(
        viikate_rocket_U.fin_cmd_rad,
        0,
        sizeof(viikate_rocket_U.fin_cmd_rad)
    );
    viikate_rocket_step();
    check_model_error();

    memcpy(
        plant.pad_model_m,
        viikate_rocket_Y.pos_model_m,
        sizeof(plant.pad_model_m)
    );
    fill_outputs(out);
}

void viikate_plant_step(
    const viikate_plant_inputs_t *in,
    viikate_plant_outputs_t *out
) {
    for (int i = 0; i < VIIKATE_PLANT_FIN_COUNT; i++) {
        viikate_rocket_U.fin_cmd_rad[i] = in->fin_cmd_rad[i];
    }
    viikate_rocket_step();
    check_model_error();
    plant.steps++;
    fill_outputs(out);
}

const char *viikate_plant_error(void) {
    return plant.error;
}
