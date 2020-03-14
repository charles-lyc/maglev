#ifndef __PID_H
#define __PID_H
#include "datatypes.h"

void pid_init(pid_t *pid, pid_param_t *pid_param);
float pid_regulator(float error, pid_t *pid);

#endif

