/*
 * Plant bridge: steps the plant in native_sim simulated time, turns the
 * firmware's servo PWM into fin commands and publishes sensor samples back.
 *
 * Every plant step (1 ms for the toy plant):
 *   servo PWM pulse -> fin command (rad) -> plant step
 *   -> IMU sample + data-ready trigger, barometer reading
 *
 * Command line (append to zephyr.exe):
 *   --plant-params=name=value[,name=value...]  scenario parameters
 *   --servo-signs=s0,s1,s2,s3                  +1/-1 per servo channel,
 *                                              mounting direction into the
 *                                              plant (default all +1)
 *
 * Log lines on stdout, every CONFIG_VIIKATE_SIM_LOG_PERIOD_MS:
 *   SIMHDR,<plant name>,<interface version>,<column names...>
 *   SIM,<values...>
 *
 * The run stops early with:
 *   SIMEND,ground_impact   exit 0: the rocket is more than 1 m below the
 *                          pad (the plant has no ground once it leaves it)
 *   SIMERR,<reason>        exit 1: a plant output is not finite
 *                          (non_finite_output) or the plant reported an
 *                          error (its message). Nothing is published to the
 *                          firmware or logged from that step.
 */

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/arch/posix/posix_trace.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "cmdline.h"
#include "posix_board_if.h"
#include "posix_native_task.h"
#include "sim_internal.h"
#include "viikate_plant.h"

#define SIM_PI 3.14159265358979323846
#define SERVO_SIGNS_USAGE "--servo-signs: expected 4 values of +1/-1\n"
#define RAD2DEG (180.0 / SIM_PI)
#define DEG2RAD (SIM_PI / 180.0)
/* Ground impact: more than this below the pad. */
#define GROUND_MARGIN_M 1.0

#define SERVO_INFO(n) \
    { \
        .channel = DT_PWMS_CHANNEL(DT_NODELABEL(n)), \
        .min_ns = DT_PROP(DT_NODELABEL(n), min_pulse), \
        .max_ns = DT_PROP(DT_NODELABEL(n), max_pulse), \
    }

struct servo_info {
    uint32_t channel;
    uint32_t min_ns;
    uint32_t max_ns;
};

/* Same nodes the firmware drives in code/app/src/guidance/pid.c. */
static const struct servo_info servos[VIIKATE_PLANT_FIN_COUNT] = {
    SERVO_INFO(servo0),
    SERVO_INFO(servo1),
    SERVO_INFO(servo2),
    SERVO_INFO(servo3),
};

static char *plant_params_arg;
static char *servo_signs_arg;
static double servo_sign[VIIKATE_PLANT_FIN_COUNT] = {1.0, 1.0, 1.0, 1.0};

static viikate_plant_inputs_t plant_in;
static viikate_plant_outputs_t plant_out;
static struct k_timer plant_timer;
static uint32_t step_count;
static uint32_t steps_per_log;

static void add_cmdline_options(void) {
    static struct args_struct_t options[] = {
        {
            .option = "plant-params",
            .name = "name=value,...",
            .type = 's',
            .dest = (void *)&plant_params_arg,
            .descript = "Plant scenario parameters",
        },
        {
            .option = "servo-signs",
            .name = "s0,s1,s2,s3",
            .type = 's',
            .dest = (void *)&servo_signs_arg,
            .descript = "Sign (+1/-1) applied to each servo channel",
        },
        ARG_TABLE_ENDMARKER,
    };

    native_add_command_line_opts(options);
}

NATIVE_TASK(add_cmdline_options, PRE_BOOT_1, 10);

static void apply_plant_params(char *arg) {
    char *item = arg;

    while (item != NULL && *item != '\0') {
        char *next = strchr(item, ',');
        char *eq;
        char *end;
        double value;

        if (next != NULL) {
            *next++ = '\0';
        }
        eq = strchr(item, '=');
        if (eq == NULL) {
            posix_print_error_and_exit(
                "--plant-params: missing '=' in %s\n",
                item
            );
        }
        *eq = '\0';
        value = strtod(eq + 1, &end);
        if (end == eq + 1 || *end != '\0' || !isfinite(value)) {
            posix_print_error_and_exit(
                "--plant-params: bad value for %s\n",
                item
            );
        }
        switch (viikate_plant_set_param(item, value)) {
            case 0:
                break;
            case -2:
                posix_print_error_and_exit(
                    "--plant-params: value out of range for %s\n",
                    item
                );
                break;
            default:
                posix_print_error_and_exit(
                    "--plant-params: %s has no parameter %s\n",
                    viikate_plant_name(),
                    item
                );
        }
        item = next;
    }
}

static void apply_servo_signs(const char *arg) {
    const char *p = arg;

    for (int i = 0; i < VIIKATE_PLANT_FIN_COUNT; i++) {
        char *end;
        double sign = strtod(p, &end);

        if (end == p || (sign != 1.0 && sign != -1.0)) {
            posix_print_error_and_exit(SERVO_SIGNS_USAGE);
        }
        servo_sign[i] = sign;
        p = end;
        if (i < VIIKATE_PLANT_FIN_COUNT - 1) {
            if (*p != ',') {
                posix_print_error_and_exit(SERVO_SIGNS_USAGE);
            }
            p++;
        }
    }
    if (*p != '\0') {
        posix_print_error_and_exit(SERVO_SIGNS_USAGE);
    }
}

/* Inverse of servo_set_angle() in code/app/src/servo.c. */
static double servo_angle_deg(const struct servo_info *servo) {
    uint32_t pulse = viikate_sim_pwm_pulse_ns(servo->channel);

    if (pulse == 0) {
        return 0.0; /* never commanded: assume neutral */
    }
    return ((double)pulse - servo->min_ns)
        / (double)(servo->max_ns - servo->min_ns) * 180.0
        - 90.0;
}

/*
 * IMU mounting. code/app/src/main.c (transform_coords) maps raw sensor x to
 * its z-up axis, i.e. raw x points along the rocket axis like body x here.
 * Assumed identity until the real mounting is confirmed.
 */
static void body_to_imu(const double body[3], double imu[3]) {
    imu[0] = body[0];
    imu[1] = body[1];
    imu[2] = body[2];
}

static void publish_sensors(void) {
    double accel[3];
    double gyro[3];

    body_to_imu(plant_out.specific_force_body_mps2, accel);
    body_to_imu(plant_out.angular_rate_body_radps, gyro);
    viikate_sim_baro_set(
        plant_out.static_pressure_pa / 1000.0,
        plant_out.air_temperature_c
    );
    viikate_sim_imu_publish(accel, gyro);
}

static void log_header(void) {
    printk(
        "SIMHDR,%s,%d,t_ms,roll_deg,roll_rate_dps,"
        "cmd0_deg,cmd1_deg,cmd2_deg,cmd3_deg,"
        "fin0_deg,fin1_deg,fin2_deg,fin3_deg,"
        "pitch_deg,yaw_deg,pitch_rate_dps,yaw_rate_dps,"
        "pos_n_m,pos_e_m,pos_d_m,vel_n_mps,vel_e_mps,vel_d_mps,"
        "q_w,q_x,q_y,q_z,sf_x_mps2,sf_y_mps2,sf_z_mps2,pressure_pa\n",
        viikate_plant_name(),
        VIIKATE_PLANT_IF_VERSION
    );
}

static void log_row(void) {
    const viikate_plant_truth_t *t = &plant_out.truth;
    const double *sf = plant_out.specific_force_body_mps2;

    printk(
        "SIM,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f\n",
        (unsigned int)lround(t->t_s * 1000.0),
        t->euler_rad[0] * RAD2DEG,
        t->body_rate_radps[0] * RAD2DEG,
        plant_in.fin_cmd_rad[0] * RAD2DEG,
        plant_in.fin_cmd_rad[1] * RAD2DEG,
        plant_in.fin_cmd_rad[2] * RAD2DEG,
        plant_in.fin_cmd_rad[3] * RAD2DEG,
        t->fin_angle_rad[0] * RAD2DEG,
        t->fin_angle_rad[1] * RAD2DEG,
        t->fin_angle_rad[2] * RAD2DEG,
        t->fin_angle_rad[3] * RAD2DEG,
        t->euler_rad[1] * RAD2DEG,
        t->euler_rad[2] * RAD2DEG,
        t->body_rate_radps[1] * RAD2DEG,
        t->body_rate_radps[2] * RAD2DEG,
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
        sf[0],
        sf[1],
        sf[2],
        plant_out.static_pressure_pa
    );
}

/* The outputs struct holds only doubles, so it can be scanned as an array. */
BUILD_ASSERT(sizeof(viikate_plant_outputs_t) % sizeof(double) == 0);

static bool outputs_finite(const viikate_plant_outputs_t *out) {
    const unsigned char *bytes = (const unsigned char *)out;

    for (size_t i = 0; i < sizeof(*out) / sizeof(double); i++) {
        double v;

        memcpy(&v, bytes + i * sizeof(double), sizeof(v));
        if (!isfinite(v)) {
            return false;
        }
    }
    return true;
}

/* Stop the run if the plant is unhealthy. Called after init and each step. */
static void check_plant(void) {
    const char *err = viikate_plant_error();

    if (err != NULL) {
        printk("SIMERR,%s\n", err);
        posix_exit(1);
    }
    if (!outputs_finite(&plant_out)) {
        printk("SIMERR,non_finite_output\n");
        posix_exit(1);
    }
}

/*
 * The pad hold keeps the rocket exactly at pad level until thrust exceeds
 * weight, so being below the pad always means it came back down, including
 * after a hop too low to clear the margin above the pad.
 */
static void check_ground_impact(void) {
    if (plant_out.truth.position_ned_m[2] > GROUND_MARGIN_M) {
        printk("SIMEND,ground_impact\n");
        posix_exit(0);
    }
}

static void plant_tick(struct k_timer *timer) {
    ARG_UNUSED(timer);

    for (int i = 0; i < VIIKATE_PLANT_FIN_COUNT; i++) {
        plant_in.fin_cmd_rad[i] =
            servo_sign[i] * servo_angle_deg(&servos[i]) * DEG2RAD;
    }
    viikate_plant_step(&plant_in, &plant_out);
    check_plant();
    publish_sensors();

    step_count++;
    if (step_count % steps_per_log == 0) {
        log_row();
    }
    check_ground_impact();
}

static int sim_bridge_init(void) {
    const double step_s = viikate_plant_step_size_s();
    const uint32_t step_us = (uint32_t)lround(step_s * 1e6);

    if (plant_params_arg != NULL) {
        apply_plant_params(plant_params_arg);
    }
    if (servo_signs_arg != NULL) {
        apply_servo_signs(servo_signs_arg);
    }

    steps_per_log =
        (uint32_t)lround(CONFIG_VIIKATE_SIM_LOG_PERIOD_MS * 1e-3 / step_s);
    if (steps_per_log == 0) {
        steps_per_log = 1;
    }

    viikate_plant_init(&plant_out);
    check_plant();
    publish_sensors();
    log_header();

    k_timer_init(&plant_timer, plant_tick, NULL);
    k_timer_start(&plant_timer, K_USEC(step_us), K_USEC(step_us));
    return 0;
}

/*
 * After the simulated devices, before the boot delay and main(): the plant
 * runs from power-on, so plant time equals firmware uptime.
 */
SYS_INIT(sim_bridge_init, POST_KERNEL, 99);
