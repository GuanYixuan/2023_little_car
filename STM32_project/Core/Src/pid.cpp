#include <pid.hpp>
#include "main.h"

void PID_Init(PID *pid, float Kp, float Ki, float Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->LastError = 0;
    pid->IntegError = 0;
}
void PID_Reset(PID* pid) {
    pid->LastError = 0;
    pid->IntegError = 0;
}

void PID_SetTarget(PID *pid, int target){
    pid->target = target;
}

int PID_Update(PID *pid, int result){ // result is the current value of the sensor
    float error = pid->target - result;
    float P = pid->Kp * error;
    float I = pid->Ki * (error + pid->IntegError);
    float D = pid->Kd * (error - pid->LastError);
    pid->LastError = error;
    pid->IntegError += error;
    return (P + I + D); // return the output of the PID controller
}