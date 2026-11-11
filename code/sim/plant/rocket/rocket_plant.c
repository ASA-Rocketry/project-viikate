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
 *
 * Design: the generated code's initial values are simulation/rockets/
 * viikate_v1.rocket (generate_plant.m). viikate_plant_load_design() replaces
 * them with another design file; --plant-params then override single values.
 * Generated matrices are column-major: element (i, j) of an m x n rp_* is
 * [j * m + i].
 */

#include "rocket_plant.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "rocket_design.h"
#include "rocket_frames.h"
#include "rocket_io.h"
#include "viikate_plant.h"
#include "viikate_rocket.h"

#define STEP_S 0.001
#define PLANT_PI 3.14159265358979323846
/* pi/180 times the value, the same double as MATLAB's deg2rad. */
#define DEG2RAD (PLANT_PI / 180.0)
#define FIN_ROWS 4

typedef enum {
    RANGE_ANY,
    RANGE_NON_NEGATIVE,
    RANGE_POSITIVE,
    RANGE_SEED, /* whole number >= 0 */
} range_t;

static void set_canard_fwd(double d);
static void set_canard_radius(double r);

static const struct {
    const char *name;
    double *dest; /* NULL when set() is used */
    double scale; /* dest = scale * value; -1 where the model axis flips */
    void (*set)(double value);
    range_t range;
} param_table[] = {
    /* Run conditions */
    {"ignition_time_s", &rp_ignition_time_s, 1.0, NULL, RANGE_NON_NEGATIVE},
    {"thrust_scale", &rp_thrust_scale, 1.0, NULL, RANGE_NON_NEGATIVE},
    /* rp_wind_model_mps is [up, east, north] */
    {"wind_north_mps", &rp_wind_model_mps[2], 1.0, NULL, RANGE_ANY},
    {"wind_east_mps", &rp_wind_model_mps[1], 1.0, NULL, RANGE_ANY},
    {"wind_down_mps", &rp_wind_model_mps[0], -1.0, NULL, RANGE_ANY},
    {"gust_variance", &rp_gust_variance, 1.0, NULL, RANGE_NON_NEGATIVE},
    {"gust_seed_1", &rp_gust_seed[0], 1.0, NULL, RANGE_SEED},
    {"gust_seed_2", &rp_gust_seed[1], 1.0, NULL, RANGE_SEED},
    /* Single-number design keys, with the design file's rules */
    {"burn_time_s", &rp_burn_time_s, 1.0, NULL, RANGE_POSITIVE},
    {"mass_full_kg", &rp_mass_full_kg, 1.0, NULL, RANGE_POSITIVE},
    {"mass_fuel_kg", &rp_mass_fuel_kg, 1.0, NULL, RANGE_NON_NEGATIVE},
    {"length_m", &rp_length_m, 1.0, NULL, RANGE_POSITIVE},
    {"frontal_area_m2", &rp_frontal_area_m2, 1.0, NULL, RANGE_POSITIVE},
    {"cd0", &rp_cd0, 1.0, NULL, RANGE_NON_NEGATIVE},
    {"cn_alpha", &rp_cn_alpha, 1.0, NULL, RANGE_NON_NEGATIVE},
    {"cp_aft_of_cg_m", &rp_r_cp_m[0], -1.0, NULL, RANGE_ANY},
    {"cmq", &rp_cmq, 1.0, NULL, RANGE_ANY},
    {"cnr", &rp_cnr, 1.0, NULL, RANGE_ANY},
    {"clp", &rp_clp, 1.0, NULL, RANGE_ANY},
    {"roll_ref_length_m", &rp_roll_ref_length_m, 1.0, NULL, RANGE_POSITIVE},
    {"canard_area_m2", &rp_canard_area_m2, 1.0, NULL, RANGE_POSITIVE},
    {"canard_cl_alpha", &rp_canard_cl_alpha, 1.0, NULL, RANGE_NON_NEGATIVE},
    {"canard_fwd_of_cg_m", NULL, 1.0, set_canard_fwd, RANGE_ANY},
    {"canard_lift_radius_m", NULL, 1.0, set_canard_radius, RANGE_POSITIVE},
    {"servo_tau_s", &rp_servo_tau_s, 1.0, NULL, RANGE_POSITIVE},
    {"servo_rate_limit_dps",
     &rp_servo_rate_limit_radps,
     DEG2RAD,
     NULL,
     RANGE_POSITIVE},
};

static struct {
    unsigned long steps;
    double pad_model_m[3];
    const char *error;
} plant;

/* Kept across viikate_plant_init(), which resets plant. */
static struct {
    struct rocket_design design;
    int loaded;
    char error[512];
    char text[ROCKET_FILE_MAX + 1];
} load;

const char *viikate_plant_name(void) {
    return "rocket_plant";
}

/* rp_fin_positions_m = [d 0 r; d -r 0; d 0 -r; d r 0], column-major. */
static void set_canard_fwd(double d) {
    for (int row = 0; row < FIN_ROWS; row++) {
        rp_fin_positions_m[0 * FIN_ROWS + row] = d;
    }
}

static void set_canard_radius(double r) {
    rp_fin_positions_m[2 * FIN_ROWS + 0] = r;
    rp_fin_positions_m[1 * FIN_ROWS + 1] = -r;
    rp_fin_positions_m[2 * FIN_ROWS + 2] = -r;
    rp_fin_positions_m[1 * FIN_ROWS + 3] = r;
}

/* J = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz] (symmetric, so either order). */
static void
set_inertia(double J[9], const double diag[3], const double prod[3]) {
    J[0] = diag[0];
    J[4] = diag[1];
    J[8] = diag[2];
    J[1] = J[3] = prod[0];
    J[2] = J[6] = prod[1];
    J[5] = J[7] = prod[2];
}

static void apply_design(const struct rocket_design *d) {
    rp_burn_time_s = d->burn_time_s;
    rp_mass_full_kg = d->mass_full_kg;
    rp_mass_fuel_kg = d->mass_fuel_kg;
    set_inertia(
        rp_inertia_full_kgm2,
        d->inertia_full_kgm2,
        d->inertia_full_products_kgm2
    );
    set_inertia(
        rp_inertia_empty_kgm2,
        d->inertia_empty_kgm2,
        d->inertia_empty_products_kgm2
    );
    rp_length_m = d->length_m;
    rp_frontal_area_m2 = d->frontal_area_m2;
    rp_cd0 = d->cd0;
    rp_cn_alpha = d->cn_alpha;
    rp_r_cp_m[0] = -d->cp_aft_of_cg_m;
    rp_r_cp_m[1] = 0.0;
    rp_r_cp_m[2] = 0.0;
    rp_cmq = d->cmq;
    rp_cnr = d->cnr;
    rp_clp = d->clp;
    rp_roll_ref_length_m = d->roll_ref_length_m;
    rp_canard_area_m2 = d->canard_area_m2;
    rp_canard_cl_alpha = d->canard_cl_alpha;
    memset(rp_fin_positions_m, 0, sizeof(rp_fin_positions_m));
    set_canard_fwd(d->canard_fwd_of_cg_m);
    set_canard_radius(d->canard_lift_radius_m);
    rp_servo_tau_s = d->servo_tau_s;
    rp_servo_rate_limit_radps = DEG2RAD * d->servo_rate_limit_dps;
    memcpy(rp_thrust_t_s, d->thrust_t_s, sizeof(rp_thrust_t_s));
    memcpy(rp_thrust_n, d->thrust_n, sizeof(rp_thrust_n));
}

static int read_into_text(const char *path) {
    long n = rocket_read_file(path, load.text, ROCKET_FILE_MAX + 1);

    if (n < 0) {
        snprintf(
            load.error,
            sizeof(load.error),
            "%.400s: cannot read file",
            path
        );
        return -1;
    }
    return (int)n;
}

int viikate_plant_load_design(const char *path) {
    struct rocket_design d;
    char motor[512];
    char err[400];
    int n;

    load.error[0] = '\0';
    n = read_into_text(path);
    if (n < 0) {
        return -1;
    }
    if (rocket_design_parse(
            load.text,
            (size_t)n,
            path,
            &d,
            load.error,
            sizeof(load.error)
        )
        != 0) {
        return -1;
    }
    if (rocket_design_motor_path(path, &d, motor, sizeof(motor)) != 0) {
        snprintf(
            load.error,
            sizeof(load.error),
            "%.400s: motor path too long",
            path
        );
        return -1;
    }
    n = read_into_text(motor);
    if (n < 0) {
        return -1;
    }
    if (rocket_motor_parse_eng(
            load.text,
            (size_t)n,
            motor,
            &d,
            load.error,
            sizeof(load.error)
        )
        != 0) {
        return -1;
    }
    if (rocket_design_check(&d, err, sizeof(err)) != 0) {
        snprintf(load.error, sizeof(load.error), "%.300s: %.200s", path, err);
        return -1;
    }
    load.design = d;
    load.loaded = 1;
    apply_design(&load.design);
    return 0;
}

const char *viikate_plant_design_name(void) {
    return load.loaded ? load.design.name : "viikate_v1";
}

const struct rocket_design *rocket_plant_design(void) {
    return load.loaded ? &load.design : NULL;
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
            if (param_table[i].set != NULL) {
                param_table[i].set(value);
            } else {
                *param_table[i].dest = param_table[i].scale * value;
            }
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
    } else if (!(rp_burn_time_s > 0.0)) {
        plant.error = "burn_time_s must be > 0";
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
    if (load.error[0] != '\0') {
        return load.error;
    }
    return plant.error;
}
