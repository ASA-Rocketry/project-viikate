/* Glue between the simulated devices and the plant bridge. */

#ifndef VIIKATE_SIM_INTERNAL_H_
#define VIIKATE_SIM_INTERNAL_H_

#include <stdint.h>

/* Last pulse width written to a channel of the simulated PWM, 0 if never. */
uint32_t viikate_sim_pwm_pulse_ns(uint32_t channel);

/* Update the simulated IMU and fire its data-ready trigger. */
void viikate_sim_imu_publish(
    const double accel_mps2[3],
    const double gyro_radps[3]
);

/* Update the simulated barometer reading. */
void viikate_sim_baro_set(double pressure_kpa, double temperature_c);

#endif /* VIIKATE_SIM_INTERNAL_H_ */
