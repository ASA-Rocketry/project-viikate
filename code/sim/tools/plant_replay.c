/*
 * Step a plant (viikate_plant.h) without Zephyr and print its outputs as CSV.
 * Built by code/sim/sil.py with the rocket plant adapter and the generated
 * code; used by code/sim/tests/test_replay.py and test_flight.py.
 *
 *   plant_replay (--input FILE | --zero-input) --duration SECONDS
 *                [--period-ms N] [name=value ...]
 *
 * --input FILE: CSV with header t_s,fin0_rad,fin1_rad,fin2_rad,fin3_rad; each
 * row applies from step round(t_s / step size) until the next row. Steps
 * start at 1: init already ran the step at t = 0 with zero commands.
 *
 * Exit codes: 0 done, 1 usage or input file error, 2 bad parameter,
 * 3 plant error or non-finite output.
 */

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "viikate_plant.h"

#define MAX_INPUT_ROWS 4096

typedef struct {
    long step;
    double fin_rad[VIIKATE_PLANT_FIN_COUNT];
} input_row_t;

static input_row_t input_rows[MAX_INPUT_ROWS];
static size_t input_count;

static void usage(void) {
    fprintf(
        stderr,
        "usage: plant_replay (--input FILE | --zero-input) --duration SECONDS"
        " [--period-ms N] [name=value ...]\n"
    );
    exit(1);
}

static double parse_number(const char *text, const char *what) {
    char *end;
    double value;

    errno = 0;
    value = strtod(text, &end);
    if (end == text || *end != '\0' || errno != 0 || !isfinite(value)) {
        fprintf(stderr, "plant_replay: bad number for %s: %s\n", what, text);
        exit(what[0] == '-' ? 1 : 2);
    }
    return value;
}

static void read_input(const char *path, double step_s) {
    char line[512];
    FILE *f = fopen(path, "r");

    if (f == NULL) {
        fprintf(stderr, "plant_replay: cannot open %s\n", path);
        exit(1);
    }
    if (fgets(line, sizeof(line), f) == NULL) {
        fprintf(stderr, "plant_replay: %s is empty\n", path);
        exit(1);
    }
    while (fgets(line, sizeof(line), f) != NULL) {
        input_row_t *row = &input_rows[input_count];
        double t_s;

        if (line[0] == '\n' || line[0] == '\0') {
            continue;
        }
        if (input_count == MAX_INPUT_ROWS) {
            fprintf(stderr, "plant_replay: too many rows in %s\n", path);
            exit(1);
        }
        if (sscanf(
                line,
                "%lf,%lf,%lf,%lf,%lf",
                &t_s,
                &row->fin_rad[0],
                &row->fin_rad[1],
                &row->fin_rad[2],
                &row->fin_rad[3]
            )
            != 5) {
            fprintf(stderr, "plant_replay: bad row in %s: %s", path, line);
            exit(1);
        }
        row->step = lround(t_s / step_s);
        if (input_count > 0 && row->step < input_rows[input_count - 1].step) {
            fprintf(stderr, "plant_replay: rows out of order in %s\n", path);
            exit(1);
        }
        input_count++;
    }
    fclose(f);
}

static void apply_param(char *arg) {
    char *eq = strchr(arg, '=');
    double value;

    if (eq == NULL) {
        fprintf(stderr, "plant_replay: expected name=value, got %s\n", arg);
        exit(2);
    }
    *eq = '\0';
    value = parse_number(eq + 1, arg);
    switch (viikate_plant_set_param(arg, value)) {
        case 0:
            return;
        case -2:
            fprintf(stderr, "plant_replay: value out of range for %s\n", arg);
            exit(2);
        default:
            fprintf(
                stderr,
                "plant_replay: %s has no parameter %s\n",
                viikate_plant_name(),
                arg
            );
            exit(2);
    }
}

static int outputs_finite(const viikate_plant_outputs_t *out) {
    const unsigned char *bytes = (const unsigned char *)out;

    for (size_t i = 0; i < sizeof(*out) / sizeof(double); i++) {
        double v;

        memcpy(&v, bytes + i * sizeof(double), sizeof(v));
        if (!isfinite(v)) {
            return 0;
        }
    }
    return 1;
}

static void check(const viikate_plant_outputs_t *out) {
    const char *err = viikate_plant_error();

    if (err != NULL) {
        fprintf(stderr, "plant_replay: plant error: %s\n", err);
        exit(3);
    }
    if (!outputs_finite(out)) {
        fprintf(
            stderr,
            "plant_replay: non-finite output at t = %g s\n",
            out->truth.t_s
        );
        exit(3);
    }
}

static void print_header(void) {
    printf(
        "t_s,sf_x_mps2,sf_y_mps2,sf_z_mps2,gyro_x_radps,gyro_y_radps,"
        "gyro_z_radps,pressure_pa,temperature_c,pos_n_m,pos_e_m,pos_d_m,"
        "vel_n_mps,vel_e_mps,vel_d_mps,q_w,q_x,q_y,q_z,roll_rad,pitch_rad,"
        "yaw_rad,fin0_rad,fin1_rad,fin2_rad,fin3_rad\n"
    );
}

static void print_row(const viikate_plant_outputs_t *out) {
    const viikate_plant_truth_t *t = &out->truth;
    const double values[] = {
        t->t_s,
        out->specific_force_body_mps2[0],
        out->specific_force_body_mps2[1],
        out->specific_force_body_mps2[2],
        out->angular_rate_body_radps[0],
        out->angular_rate_body_radps[1],
        out->angular_rate_body_radps[2],
        out->static_pressure_pa,
        out->air_temperature_c,
        t->position_ned_m[0],
        t->position_ned_m[1],
        t->position_ned_m[2],
        t->velocity_ned_mps[0],
        t->velocity_ned_mps[1],
        t->velocity_ned_mps[2],
        t->quat_wxyz[0],
        t->quat_wxyz[1],
        t->quat_wxyz[2],
        t->quat_wxyz[3],
        t->euler_rad[0],
        t->euler_rad[1],
        t->euler_rad[2],
        t->fin_angle_rad[0],
        t->fin_angle_rad[1],
        t->fin_angle_rad[2],
        t->fin_angle_rad[3],
    };
    const size_t n = sizeof(values) / sizeof(values[0]);

    for (size_t i = 0; i < n; i++) {
        printf(i + 1 < n ? "%.12g," : "%.12g\n", values[i]);
    }
}

int main(int argc, char **argv) {
    const double step_s = viikate_plant_step_size_s();
    const char *input_path = NULL;
    int zero_input = 0;
    double duration_s = -1.0;
    long period_ms = 10;
    viikate_plant_inputs_t in = {0};
    viikate_plant_outputs_t out;
    size_t next_row = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
            input_path = argv[++i];
        } else if (strcmp(argv[i], "--zero-input") == 0) {
            zero_input = 1;
        } else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            duration_s = parse_number(argv[++i], "--duration");
        } else if (strcmp(argv[i], "--period-ms") == 0 && i + 1 < argc) {
            period_ms = lround(parse_number(argv[++i], "--period-ms"));
        } else if (strncmp(argv[i], "--", 2) == 0) {
            usage();
        } else {
            apply_param(argv[i]);
        }
    }
    if ((input_path == NULL) == !zero_input || duration_s < 0.0
        || period_ms <= 0) {
        usage();
    }
    if (input_path != NULL) {
        read_input(input_path, step_s);
    }

    const long steps = lround(duration_s / step_s);
    const long period_steps = lround(period_ms * 1e-3 / step_s);

    viikate_plant_init(&out);
    check(&out);
    print_header();
    print_row(&out);

    for (long k = 1; k <= steps; k++) {
        while (next_row < input_count && input_rows[next_row].step <= k) {
            memcpy(
                in.fin_cmd_rad,
                input_rows[next_row].fin_rad,
                sizeof(in.fin_cmd_rad)
            );
            next_row++;
        }
        viikate_plant_step(&in, &out);
        check(&out);
        if (k % period_steps == 0) {
            print_row(&out);
        }
    }
    return 0;
}
