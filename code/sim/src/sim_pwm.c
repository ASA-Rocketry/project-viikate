/*
 * Simulated PWM controller. Stores the pulse width per channel so the plant
 * bridge can turn servo commands into fin deflections. One cycle is 1 ns, so
 * pulse widths in cycles are nanoseconds.
 */

#define DT_DRV_COMPAT viikate_sim_pwm

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#include "sim_internal.h"

#define SIM_PWM_CHANNELS 8

static uint32_t pulse_ns[SIM_PWM_CHANNELS];

static int sim_pwm_set_cycles(
    const struct device *dev,
    uint32_t channel,
    uint32_t period_cycles,
    uint32_t pulse_cycles,
    pwm_flags_t flags
) {
    ARG_UNUSED(dev);
    ARG_UNUSED(period_cycles);
    ARG_UNUSED(flags);

    if (channel >= SIM_PWM_CHANNELS) {
        return -EINVAL;
    }
    pulse_ns[channel] = pulse_cycles;
    return 0;
}

static int sim_pwm_get_cycles_per_sec(
    const struct device *dev,
    uint32_t channel,
    uint64_t *cycles
) {
    ARG_UNUSED(dev);
    ARG_UNUSED(channel);

    *cycles = NSEC_PER_SEC;
    return 0;
}

uint32_t viikate_sim_pwm_pulse_ns(uint32_t channel) {
    return channel < SIM_PWM_CHANNELS ? pulse_ns[channel] : 0;
}

static DEVICE_API(pwm, sim_pwm_api) = {
    .set_cycles = sim_pwm_set_cycles,
    .get_cycles_per_sec = sim_pwm_get_cycles_per_sec,
};

DEVICE_DT_INST_DEFINE(
    0,
    NULL,
    NULL,
    NULL,
    NULL,
    POST_KERNEL,
    CONFIG_PWM_INIT_PRIORITY,
    &sim_pwm_api
);
