#include <motor.hpp>
#include <pid.hpp>
#include "move.hpp"
#include "main.h"

#include <stdio.h>

# define DT 100 // 100ms
# define REDUCTION_RATIO 21.3
# define FORWARD_RATIO (1 / 1)
# define RIGHT_RATIO (1 / 1.3)
# define LENGTH_HALF 85
# define WIDTH_HALF 95
# define STEER_RATIO (1 / 1.25)

extern uint16_t rotate_0[4];
extern uint16_t rotate_1[4];
extern uint32_t PWM_channels[4];
extern uint32_t TIM8_CCR[4];
extern MOTOR motor[4];
extern int i_encoder_temp[4];
extern int i_encoder_output[4];
extern int i_tim8_ccr_target[4];

extern PID pid[4];
extern TIM_HandleTypeDef* MOTOR_TIMER_HANDLE_P[4];

int GetTimEncoder(int encoder_serial) {
  static short i_encoder;
  i_encoder = __HAL_TIM_GetCounter(MOTOR_TIMER_HANDLE_P[encoder_serial]);
  __HAL_TIM_SetCounter(MOTOR_TIMER_HANDLE_P[encoder_serial], 0);
	return i_encoder;
}

void Brake() {
    for (int i = 0; i < 4; i ++) {
        HAL_GPIO_WritePin(GPIOD, rotate_0[i], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, rotate_1[i], GPIO_PIN_RESET);
    }
}

void forward(int16_t dist) {
    int cnt_max = 0;
    int i_cnt = 0;
    int dist_positive = 0, brake = 0;
    if (dist > 0) {
        dist_positive = dist * RIGHT_RATIO;
        SetRotateDirection(&motor[0], 1);
        SetRotateDirection(&motor[1], 1);
        SetRotateDirection(&motor[2], 1);
        SetRotateDirection(&motor[3], 1);
        cnt_max = distToCnt(dist_positive);
    }
    else if (dist < 0) {
        dist_positive = - dist * RIGHT_RATIO;
        SetRotateDirection(&motor[0], -1);
        SetRotateDirection(&motor[1], -1);
        SetRotateDirection(&motor[2], -1);
        SetRotateDirection(&motor[3], -1);
        cnt_max = distToCnt(dist_positive);
    } else brake = 1;
    i_tim8_ccr_target[0] = 30;
    i_tim8_ccr_target[1] = 30;
    i_tim8_ccr_target[2] = 31;
    i_tim8_ccr_target[3] = 31;
    for (int i = 0; i < 4; i++) {
        PID_Reset(&pid[i]);
        __HAL_TIM_SetCounter(MOTOR_TIMER_HANDLE_P[i], 0);
    }

    while(1) {
        if (brake == 1) {
            Brake();
            break;
        }
        for (int i = 0; i < 4; i ++) {
            /*读取编码器*/
            i_encoder_temp[i] = GetTimEncoder(i);
            if (i_encoder_temp[i] < 0) i_encoder_temp[i] = - i_encoder_temp[i];
            // i_encoder_output[i] = i_encoder_temp[i] * 6000 * 1000 / (DT * 4 * 11 * REDUCTION_RATIO * 280); //?
            i_encoder_output[i] = i_encoder_temp[i] * 487.013 / DT / REDUCTION_RATIO;
            /*PID*/
            PID_SetTarget(&pid[i], i_tim8_ccr_target[i]);
            int adjusted_target = i_tim8_ccr_target[i] + PID_Update(&pid[i], i_encoder_output[i]);
            if (adjusted_target > 99) adjusted_target = 99;
            else if (adjusted_target < 0) adjusted_target = -adjusted_target;
            /*设置运动速度*/
            SetRotateSpeed(&motor[i], adjusted_target);
            /*电机运动*/
            Rotate(&motor[i]);
        }
        HAL_Delay(DT);

        if (i_encoder_output[0]) i_cnt += (i_encoder_temp[0] + i_encoder_temp[1] + i_encoder_temp[2] + i_encoder_temp[3]) / 4;
        printf("channels:%i, %i, %i, %i, %i\n", i_encoder_output[0], i_encoder_output[1], i_encoder_output[2], i_encoder_output[3], i_cnt);

        if (i_cnt >= cnt_max)
        {
            i_cnt = 0;
            for (int i = 0; i < 4; i ++)
            {
                SetRotateSpeed(&motor[i], 0);
                Rotate(&motor[i]);
                i_tim8_ccr_target[i] = 0;
            }
            break;
        }
    }
}

void right(int16_t dist)
{
    int cnt_max = 0;
    int i_cnt = 0;
    int dist_positive = 0, brake = 0;
    if (dist > 0) {
        dist_positive = dist * RIGHT_RATIO;
        SetRotateDirection(&motor[0], 90);
        SetRotateDirection(&motor[1], -90);
        SetRotateDirection(&motor[2], 90);
        SetRotateDirection(&motor[3], -90);
        cnt_max = distToCnt(dist_positive);
    } else if (dist < 0) {
        dist_positive = - dist * RIGHT_RATIO;
        SetRotateDirection(&motor[0], -90);
        SetRotateDirection(&motor[1], 90);
        SetRotateDirection(&motor[2], -90);
        SetRotateDirection(&motor[3], 90);
        cnt_max = distToCnt(dist_positive);
    } else brake = 1;

    while (1) {
        if (brake == 1) {
            Brake();
            break;
        }
        for (int i = 0; i < 4; i ++) {
            i_tim8_ccr_target[i] = 95;
            /*读取编码器*/
            i_encoder_temp[i] = GetTimEncoder(i);
            if (i_encoder_temp[i] < 0) i_encoder_temp[i] = - i_encoder_temp[i];
            // i_encoder_output[i] = i_encoder_temp[i] * 6000 * 1000 / DT / 4 / 11 / REDUCTION_RATIO / 280; //?
            i_encoder_output[i] = i_encoder_temp[i] * 487.013 / DT / REDUCTION_RATIO; //?
            /*PID*/
            PID_SetTarget(&pid[i], i_tim8_ccr_target[i]);
            int adjusted_target = i_tim8_ccr_target[i] + PID_Update(&pid[i], i_encoder_output[i]);
            if (adjusted_target > 99) adjusted_target = 99;
            else if (adjusted_target < 0) adjusted_target = -adjusted_target;
            /*设置运动速度*/
            SetRotateSpeed(&motor[i], adjusted_target);
            /*电机运动*/
            Rotate(&motor[i]);
        }
        HAL_Delay(DT);

        if (i_encoder_output[0]) i_cnt += (i_encoder_temp[0] + i_encoder_temp[1] + i_encoder_temp[2] + i_encoder_temp[3]) / 4;
        printf("channels:%i, %i, %i, %i, %i\n", i_encoder_output[0], i_encoder_output[1], i_encoder_output[2], i_encoder_output[3], i_cnt);

        if (i_cnt >= cnt_max) {
            for (int i = 0; i < 4; i ++) {
                SetRotateSpeed(&motor[i], 0);
                Rotate(&motor[i]);
                i_tim8_ccr_target[i] = 0;
            }
            break;
        }
    }
}

#define STEER_DT 33
void steer(int16_t angle) /*逆时针*/
{
    int cnt_max = 0;
    int i_cnt = 0;
    int angle_positive = 0;
    int brake = 0;
    if (angle > 0) {
        angle_positive = angle * STEER_RATIO;
        SetRotateDirection(&motor[0], -90);
        SetRotateDirection(&motor[1], -90);
        SetRotateDirection(&motor[2], 90);
        SetRotateDirection(&motor[3], 90);
        cnt_max = distToCnt(angle_positive * 3.1416 * (LENGTH_HALF + WIDTH_HALF) / 180);
    } else if (angle < 0) {
        angle_positive = - angle * STEER_RATIO;
        SetRotateDirection(&motor[0], 90);
        SetRotateDirection(&motor[1], 90);
        SetRotateDirection(&motor[2], -90);
        SetRotateDirection(&motor[3], -90);
        cnt_max = distToCnt(angle_positive * 3.1416 * (LENGTH_HALF + WIDTH_HALF) / 180);
    } else brake = 1;

    while (1) {
        if (brake == 1) {
            Brake();
            break;
        }
        for (int i = 0; i < 4; i ++) {
            i_tim8_ccr_target[i] = 95;
            /*读取编码器*/
            i_encoder_temp[i] = GetTimEncoder(i);
            if (i_encoder_temp[i] < 0) i_encoder_temp[i] = - i_encoder_temp[i];
            // i_encoder_output[i] = i_encoder_temp[i] * 6000 * 1000 / (STEER_DT * 4 * 11 * REDUCTION_RATIO * 280); //?
            i_encoder_output[i] = i_encoder_temp[i] * 487.013 / (STEER_DT * REDUCTION_RATIO); //?
            /*PID*/
            PID_SetTarget(&pid[i], i_tim8_ccr_target[i]);
            int adjusted_target = i_tim8_ccr_target[i] + PID_Update(&pid[i], i_encoder_output[i]);
            if (adjusted_target > 99) adjusted_target = 99;
            else if (adjusted_target < 0) adjusted_target = -adjusted_target;
            /*设置运动速度*/
            SetRotateSpeed(&motor[i], adjusted_target);
            /*电机运动*/
            Rotate(&motor[i]);
        }
        HAL_Delay(STEER_DT);

        if (i_encoder_output[0]) i_cnt += (i_encoder_temp[0] + i_encoder_temp[1] + i_encoder_temp[2] + i_encoder_temp[3]) / 4;
        printf("channels:%i, %i, %i, %i, %i\n", i_encoder_output[0], i_encoder_output[1], i_encoder_output[2], i_encoder_output[3], i_cnt);

        if (i_cnt >= cnt_max) {
            for (int i = 0; i < 4; i ++) {
                SetRotateSpeed(&motor[i], 0);
                Rotate(&motor[i]);
                i_tim8_ccr_target[i] = 0;
            }
            break;
        }
    }
}

int distToCnt(int16_t dist)
{
    return dist * 4 * 11 * REDUCTION_RATIO / (60 * 3.14);
}
