#include "motor.hpp"
#include "main.h"
/* USER CODE BEGIN PD */
# define DT 100 // 100ms
# define REDUCTION_RATIO 21.3
# define MOTOR_LF 0
# define MOTOR_LB 1
# define MOTOR_RB 2
# define MOTOR_RF 3
# define FORWARD 0
# define BACKWARD 1
/* USER CODE END PD */


extern TIM_HandleTypeDef* timer[4];
extern TIM_HandleTypeDef htim8;
extern uint16_t rotate_0[4];
extern uint16_t rotate_1[4];
extern uint32_t PWM_channels[4];
extern uint32_t TIM8_CCR[4];


void MotorInit(MOTOR *motor, int serial)
{
    motor->motor_serial = serial;
    motor->rotate_speed = 0;
    motor->rotate_time = 0;
}

void SetRotateSpeed(MOTOR *motor, int speed)
{
    motor->rotate_speed = speed;
}

void SetRotateDirection(MOTOR *motor, int speed)
{
    if (speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOD, rotate_0[motor->motor_serial], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, rotate_1[motor->motor_serial], GPIO_PIN_SET);
    }
    else
    {
        motor->rotate_speed = - motor->rotate_speed;
        HAL_GPIO_WritePin(GPIOD, rotate_0[motor->motor_serial], GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, rotate_1[motor->motor_serial], GPIO_PIN_RESET);
    }

}

void Rotate(MOTOR *motor)
{
    switch (motor->motor_serial)
    {
    case 0:
        TIM8->CCR1 = motor->rotate_speed;
        break;
    case 1:
        TIM8->CCR2 = motor->rotate_speed;
        break;
    case 2:
        TIM8->CCR3 = motor->rotate_speed;
        break;
    case 3:
        TIM8->CCR4 = motor->rotate_speed;
        break;
    default:
        break;
    }
}