/*
 * Toy plant: the rocket held vertical (nose up) on a roll test stand in a
 * constant airflow. Only roll is free. Placeholder until the
 * Simulink-generated rocket model is wired in behind the same interface.
 *
 * Constants come from simulation/matlab/closedModel/closedparameters.m.
 * Time starts at firmware power-on. The default roll-rate kick comes after
 * the firmware's boot delay (1 s) and sensor calibration (about 0.7 s).
 * Servos: first-order lag per fin. Roll moment: every fin's lift times the
 * fin moment arm, plus aerodynamic roll damping. Integrated with RK4.
 */

#include <math.h>
#include <string.h>

#include "viikate_plant.h"

#define SIM_PI 3.14159265358979323846
#define STEP_S 0.001
#define G0 9.80665

/* closedparameters.m */
#define RHO 1.225
#define FIN_AREA 0.0016875
#define FIN_CL_ALPHA 1.975
#define FIN_ARM (0.075 / 2.0 + 0.03415 * 0.65)
#define IXX 0.0027
#define CLP (-0.3)
#define REF_AREA 0.00442
#define REF_LEN 0.14
#define SERVO_TAU 0.05

#define STATE_PHI 0
#define STATE_P 1
#define STATE_FIN0 2
#define STATE_COUNT (STATE_FIN0 + VIIKATE_PLANT_FIN_COUNT)

typedef struct {
    double airspeed_mps;
    double initial_roll_rad;
    double kick_time_s;
    double kick_roll_rate_radps;
    double static_pressure_pa;
    double air_temperature_c;
} params_t;

static params_t params = {
    .airspeed_mps = 100.0,
    .initial_roll_rad = 0.0,
    .kick_time_s = 2.5,
    .kick_roll_rate_radps = 1.0,
    .static_pressure_pa = 101325.0,
    .air_temperature_c = 15.0,
};

static struct {
    double t_s;
    double x[STATE_COUNT];
    int kicked;
} plant;

static const struct {
    const char *name;
    double *dest;
} param_table[] = {
    {"airspeed_mps", &params.airspeed_mps},
    {"initial_roll_rad", &params.initial_roll_rad},
    {"kick_time_s", &params.kick_time_s},
    {"kick_roll_rate_radps", &params.kick_roll_rate_radps},
    {"static_pressure_pa", &params.static_pressure_pa},
    {"air_temperature_c", &params.air_temperature_c},
};

const char *viikate_plant_name(void) {
    return "toy_roll_plant";
}

double viikate_plant_step_size_s(void) {
    return STEP_S;
}

int viikate_plant_set_param(const char *name, double value) {
    for (unsigned i = 0; i < sizeof(param_table) / sizeof(param_table[0]);
         i++) {
        if (strcmp(name, param_table[i].name) == 0) {
            *param_table[i].dest = value;
            return 0;
        }
    }
    return -1;
}

static void derivatives(const double *x, const double *cmd, double *dx) {
    const double q = 0.5 * RHO * params.airspeed_mps * params.airspeed_mps;
    double fin_sum = 0.0;

    for (int i = 0; i < VIIKATE_PLANT_FIN_COUNT; i++) {
        dx[STATE_FIN0 + i] = (cmd[i] - x[STATE_FIN0 + i]) / SERVO_TAU;
        fin_sum += x[STATE_FIN0 + i];
    }

    const double fin_moment = q * FIN_AREA * FIN_CL_ALPHA * FIN_ARM * fin_sum;
    const double damping_moment = q * REF_AREA * REF_LEN * CLP * x[STATE_P]
        * REF_LEN / (2.0 * params.airspeed_mps);

    dx[STATE_PHI] = x[STATE_P];
    dx[STATE_P] = (fin_moment + damping_moment) / IXX;
}

static void fill_outputs(viikate_plant_outputs_t *out) {
    const double phi = plant.x[STATE_PHI];
    const double c45 = cos(SIM_PI / 4.0);
    const double s45 = sin(SIM_PI / 4.0);

    memset(out, 0, sizeof(*out));

    /* Nose up: the accelerometer reads +g along body x. */
    out->specific_force_body_mps2[0] = G0;
    out->angular_rate_body_radps[0] = plant.x[STATE_P];
    out->static_pressure_pa = params.static_pressure_pa;
    out->air_temperature_c = params.air_temperature_c;

    out->truth.t_s = plant.t_s;
    /* Launch frame: only roll moves on the test stand. */
    out->truth.euler_rad[0] = phi;
    out->truth.body_rate_radps[0] = plant.x[STATE_P];
    /* q = q_pitch(90 deg) * q_roll(phi) */
    out->truth.quat_wxyz[0] = c45 * cos(phi / 2.0);
    out->truth.quat_wxyz[1] = c45 * sin(phi / 2.0);
    out->truth.quat_wxyz[2] = s45 * cos(phi / 2.0);
    out->truth.quat_wxyz[3] = -s45 * sin(phi / 2.0);
    for (int i = 0; i < VIIKATE_PLANT_FIN_COUNT; i++) {
        out->truth.fin_angle_rad[i] = plant.x[STATE_FIN0 + i];
    }
}

void viikate_plant_init(viikate_plant_outputs_t *out) {
    memset(&plant, 0, sizeof(plant));
    plant.x[STATE_PHI] = params.initial_roll_rad;
    fill_outputs(out);
}

void viikate_plant_step(
    const viikate_plant_inputs_t *in,
    viikate_plant_outputs_t *out
) {
    double k1[STATE_COUNT], k2[STATE_COUNT], k3[STATE_COUNT], k4[STATE_COUNT];
    double tmp[STATE_COUNT];
    const double h = STEP_S;

    if (!plant.kicked && plant.t_s >= params.kick_time_s) {
        plant.x[STATE_P] += params.kick_roll_rate_radps;
        plant.kicked = 1;
    }

    derivatives(plant.x, in->fin_cmd_rad, k1);
    for (int i = 0; i < STATE_COUNT; i++) {
        tmp[i] = plant.x[i] + 0.5 * h * k1[i];
    }
    derivatives(tmp, in->fin_cmd_rad, k2);
    for (int i = 0; i < STATE_COUNT; i++) {
        tmp[i] = plant.x[i] + 0.5 * h * k2[i];
    }
    derivatives(tmp, in->fin_cmd_rad, k3);
    for (int i = 0; i < STATE_COUNT; i++) {
        tmp[i] = plant.x[i] + h * k3[i];
    }
    derivatives(tmp, in->fin_cmd_rad, k4);
    for (int i = 0; i < STATE_COUNT; i++) {
        plant.x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    plant.t_s += h;
    fill_outputs(out);
}

const char *viikate_plant_error(void) {
    return NULL;
}
