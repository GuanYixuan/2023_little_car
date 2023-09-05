#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

typedef struct{
    int motor_serial;
    int rotate_speed;
    int rotate_time;
    int rotate_direction;
} MOTOR;

void MotorInit(MOTOR *motor, int seial);
void SetRotateSpeed(MOTOR *motor, int rotate_speed);
void SetRotateDirection(MOTOR *motor, int speed);
void Rotate(MOTOR *motor);

#endif