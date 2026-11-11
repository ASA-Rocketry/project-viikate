/*
 * Interface between the SIL bridge and a plant model.
 *
 * A plant is anything that implements the functions below: the toy roll
 * plant in plant/ today, the Simulink-generated rocket model later. The
 * bridge only talks to this interface, so swapping plants does not touch the
 * simulated drivers or the firmware.
 *
 * Conventions:
 * - SI units, angles in radians.
 * - Body frame: x along the rocket axis towards the nose, y and z complete
 *   a right-handed frame (same as the Simulink model).
 * - Inertial frame: NED.
 * - One call to viikate_plant_step() advances the plant by
 *   viikate_plant_step_size_s().
 *
 * Bump VIIKATE_PLANT_IF_VERSION on any incompatible change.
 */

#ifndef VIIKATE_PLANT_H_
#define VIIKATE_PLANT_H_

#ifdef __cplusplus
extern "C" {
#endif

#define VIIKATE_PLANT_IF_VERSION 3
#define VIIKATE_PLANT_FIN_COUNT 4

typedef struct {
    /* Commanded deflection per servo channel, before servo dynamics. */
    double fin_cmd_rad[VIIKATE_PLANT_FIN_COUNT];
} viikate_plant_inputs_t;

typedef struct {
    double t_s;
    double position_ned_m[3];
    double velocity_ned_mps[3];
    /* Attitude, body to NED, scalar first: w, x, y, z. Authoritative. */
    double quat_wxyz[4];
    /*
     * Roll, pitch, yaw relative to the launch frame, not to NED (which is
     * singular for a vertical rocket). Roll is rotation about the rocket
     * axis. All three are zero on the pad.
     */
    double euler_rad[3];
    double body_rate_radps[3];
    /* Actual deflection after servo dynamics. */
    double fin_angle_rad[VIIKATE_PLANT_FIN_COUNT];
} viikate_plant_truth_t;

typedef struct {
    /* What ideal sensors at the CG would read, in the body frame. */
    double specific_force_body_mps2[3];
    double angular_rate_body_radps[3];
    double static_pressure_pa;
    double air_temperature_c;
    /* Ground truth, for logging and assertions only. Never fed to firmware. */
    viikate_plant_truth_t truth;
} viikate_plant_outputs_t;

/* Short identifier of the plant implementation, for logs. */
const char *viikate_plant_name(void);

/* Fixed integration step of the plant. */
double viikate_plant_step_size_s(void);

/*
 * Load a design file (simulation/rockets/<name>.rocket) before any
 * viikate_plant_set_param() and before viikate_plant_init(). Returns 0, or
 * -1 with the reason in viikate_plant_error().
 */
int viikate_plant_load_design(const char *path);

/*
 * Name of the design in use: the built-in one before any load, the file's
 * after a load. NULL for a plant without designs.
 */
const char *viikate_plant_design_name(void);

/*
 * Set a scenario parameter by name before viikate_plant_init().
 * Returns 0 on success, -1 if the plant has no such parameter, -2 if the
 * value is out of range for it.
 */
int viikate_plant_set_param(const char *name, double value);

/* Reset the plant to its initial state; fills the outputs at t = 0. */
void viikate_plant_init(viikate_plant_outputs_t *out);

/* Advance one step with the given inputs held constant. */
void viikate_plant_step(
    const viikate_plant_inputs_t *in,
    viikate_plant_outputs_t *out
);

/*
 * NULL while the plant is healthy, otherwise a message saying what went
 * wrong. Check after viikate_plant_init() and after every step; the outputs
 * are not valid once this is non-NULL.
 */
const char *viikate_plant_error(void);

#ifdef __cplusplus
}
#endif

#endif /* VIIKATE_PLANT_H_ */
