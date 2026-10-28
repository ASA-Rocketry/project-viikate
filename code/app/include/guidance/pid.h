#ifndef ATTITUDE_PID_H_
#define ATTITUDE_PID_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;

    bool initialized;
} pid_controller_t;

int attitude_control_init(void);
int attitude_control_update(float roll, float pitch, float dt);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_PID_H_ */
