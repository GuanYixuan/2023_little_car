#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct{
	int target;
	int result;
	float Kp, Ki, Kd;
	float IntegError;
    float LastError;
} PID;

void PID_Init(PID *pid, float Kp, float Ki, float Kd);
void PID_Reset(PID* pid);
void PID_SetTarget(PID *pid, int target);
int PID_Update(PID *pid, int result);
#endif /* INC_PID_H_ */
