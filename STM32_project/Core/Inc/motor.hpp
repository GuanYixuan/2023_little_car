#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include <main.h>

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

constexpr uint16_t rotate_0[4] = {GPIO_PIN_0, GPIO_PIN_2, GPIO_PIN_5, GPIO_PIN_7};
constexpr uint16_t rotate_1[4] = {GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_6};

#endif