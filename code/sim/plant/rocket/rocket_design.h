/*
 * Rocket design files: simulation/rockets/<name>.rocket plus a RASP .eng
 * motor file. Format and key table: simulation/rockets/README.md.
 *
 * Pure functions on text buffers: no file I/O and no dependency on the
 * generated plant, so the parser builds for the firmware simulation, for
 * plant_replay and as a host library for the tests.
 *
 * Every function returns 0 on success, or -1 with a message in err:
 * "<path>:<line>: <message>" for an error on a line, "<path>: <message>"
 * for a file-level error. rocket_design_check() messages carry no path.
 */

#ifndef ROCKET_DESIGN_H_
#define ROCKET_DESIGN_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROCKET_THRUST_POINTS 128
#define ROCKET_FILE_MAX 65536

struct rocket_design {
    char name[64];
    char motor[256]; /* as written in the file */
    int burn_time_given;
    double burn_time_s, mass_full_kg, mass_fuel_kg;
    double inertia_full_kgm2[3], inertia_empty_kgm2[3];
    double inertia_full_products_kgm2[3], inertia_empty_products_kgm2[3];
    double length_m, frontal_area_m2, cd0, cn_alpha, cp_aft_of_cg_m;
    double cmq, cnr, clp, roll_ref_length_m;
    double canard_area_m2, canard_cl_alpha, canard_fwd_of_cg_m;
    double canard_lift_radius_m;
    double servo_tau_s, servo_rate_limit_dps;
    int thrust_count; /* points from the file, after inserting (0, 0) */
    /* Thrust table padded to ROCKET_THRUST_POINTS with t_last + k s, 0 N. */
    double thrust_t_s[ROCKET_THRUST_POINTS], thrust_n[ROCKET_THRUST_POINTS];
};

/* Parse a .rocket file. Leaves the thrust table untouched. */
int rocket_design_parse(
    const char *text,
    size_t len,
    const char *path,
    struct rocket_design *out,
    char *err,
    size_t err_len
);

/* Parse a RASP .eng file into out's thrust table. */
int rocket_motor_parse_eng(
    const char *text,
    size_t len,
    const char *path,
    struct rocket_design *out,
    char *err,
    size_t err_len
);

/*
 * Checks across keys, after both files are parsed: fuel below full mass,
 * positive-definite inertia tensors. Sets burn_time_s to the last thrust
 * time when the design does not give it.
 */
int rocket_design_check(struct rocket_design *d, char *err, size_t err_len);

/* Canonical "key = value" text of every key and thrust point (%.17g). */
int rocket_design_format(const struct rocket_design *d, char *buf, size_t len);

/* Directory of rocket_path joined with d->motor; -1 if it does not fit. */
int rocket_design_motor_path(
    const char *rocket_path,
    const struct rocket_design *d,
    char *out,
    size_t len
);

/* sizeof(struct rocket_design), for the ctypes tests. */
size_t rocket_design_sizeof(void);

#ifdef __cplusplus
}
#endif

#endif /* ROCKET_DESIGN_H_ */
