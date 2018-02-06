#ifndef PID_H
#define PID_H

#include "stm32f3xx_hal.h"
#include "constants.h"

typedef struct {
    double kp;
    double ki;
    double kd;
    double sumError;
    double lastInput;
} PID;

void pid_init(PID* pid);
void pid_config(PID* pid, double kp, double ki, double kd);
int pid_compute(PID* pid, int error, int lastInput);

extern PID leftVelPID;
extern PID leftPosPID;
extern PID rightVelPID;
extern PID rightPosPID;

extern PID distancePID;
extern PID anglePID;

#endif
