#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

struct MOTOR{
    int motor_serial;
    int rotate_speed;
    int rotate_time;
    int rotate_direction;
};

void MotorInit(MOTOR *motor, int seial);
void SetRotateSpeed(MOTOR *motor, int rotate_speed);
void SetRotateDirection(MOTOR *motor, int speed);
void Rotate(MOTOR *motor);

#endif