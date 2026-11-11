/*
 * Frame mapping and standard atmosphere for the rocket plant adapter.
 *
 * The Simulink model (simulation/matlab/plant/viikate_rocket.slx) uses an
 * inertial frame with x up. viikate_plant.h uses NED. The body frame is the
 * same in both: x along the rocket axis towards the nose.
 *
 * Pure functions with no state, so they can be tested on their own
 * (code/sim/tests/test_frames.py).
 */

#ifndef ROCKET_FRAMES_H_
#define ROCKET_FRAMES_H_

/* [north, east, down] = [z, y, -x] */
void rocket_model_to_ned(const double v_model[3], double v_ned[3]);
void rocket_ned_to_model(const double v_ned[3], double v_model[3]);

/*
 * dcm_be: inertial (model frame) to body, column-major as generated.
 * quat: body to NED, scalar first, normalised, w >= 0.
 */
void rocket_dcm_be_to_quat_ned(const double dcm_be[9], double quat_wxyz[4]);

/*
 * International Standard Atmosphere troposphere, with the constants the
 * firmware's pressure_to_altitude() inverts (code/app/src/main.c).
 */
double rocket_isa_pressure_pa(double altitude_m);
double rocket_isa_temperature_c(double altitude_m);

#endif /* ROCKET_FRAMES_H_ */
