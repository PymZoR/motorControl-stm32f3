#include "motionControl/PID.h"

void pid_init(PID* pid) {
    pid->sumError  = 0;
    pid->lastInput = 0;
}

void pid_config(PID* pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

int pid_calcul(PID* pid, int error, int input) {
    // Integral term
    pid->sumError += error;

    // Derivative term with anti-windup
    unsigned long dInput = pid->lastInput - input;
    pid->lastInput = input;

    int out = pid->kp * (error + pid->ki * pid->sumError + pid->kd * dInput);
    return out;
}

PID leftVelPID;
PID leftPosPID;
PID rightVelPID;
PID rightPosPID;

PID distancePID;
PID anglePID;
