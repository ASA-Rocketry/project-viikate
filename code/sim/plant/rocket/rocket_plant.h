/* Rocket plant internals shared with plant_replay. */

#ifndef ROCKET_PLANT_H_
#define ROCKET_PLANT_H_

#include "rocket_design.h"

/* The design loaded by viikate_plant_load_design(), NULL before a load. */
const struct rocket_design *rocket_plant_design(void);

#endif /* ROCKET_PLANT_H_ */
