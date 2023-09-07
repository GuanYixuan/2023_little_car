#include "pid.hpp"
#include "motor.hpp"
#include "move.hpp"
#include "main.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdarg>

# define DT 100 // 100ms
# define REDUCTION_RATIO 21.3
# define RIGHT_RATIO (1 / 1.3)
# define LENGTH_HALF 85
# define WIDTH_HALF 95

extern MOTOR motor[4];
extern UART_HandleTypeDef huart1;

extern PID pid[4];
extern TIM_HandleTypeDef* MOTOR_TIMER_HANDLE_P[4];

void my_printf(const char* format, ...) {
    return;
    static uint8_t func_buffer[256];
	va_list args;
	va_start(args, format);
	vsprintf((char*)func_buffer, format, args);
	va_end(args);

	HAL_UART_Transmit(&huart1, func_buffer, strlen((char*)func_buffer), 1000);
}

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

bool forward(int16_t dist) {
    static int i_encoder_temp[4];
    static int i_encoder_output[4];
    static int CCR_target[4];

    // 零距离平移 = 刹车
    if (dist == 0) {
        Brake();
        return true;
    }

    int dist_positive = abs(dist);
    if (dist_positive <= FORWARD_MIN) dist_positive = 1; // 最小限制
    if (dist > 0) {
        SetRotateDirection(&motor[0], 1);
        SetRotateDirection(&motor[1], 1);
        SetRotateDirection(&motor[2], 1);
        SetRotateDirection(&motor[3], 1);
    } else {
        SetRotateDirection(&motor[0], -1);
        SetRotateDirection(&motor[1], -1);
        SetRotateDirection(&motor[2], -1);
        SetRotateDirection(&motor[3], -1);
    }

    // 寻找合适的参数
    int param_set_index = 0;
    for (int i = FORWARD_PARAM_SET_COUNT - 1; i >= 0 ; i--) if (dist_positive >= FORWARD_PARAMS[i][0]) {
        param_set_index = i;
        break;
    }
    for (int i = 0; i < 4; i++) CCR_target[i] = FORWARD_PARAMS[param_set_index][i+1];
    // 计算真正的编码器目标
    int cnt_max = distToCnt(round(dist_positive + FORWARD_OFFSET[param_set_index]));

    // PID初始化
    for (int i = 0; i < 4; i++) {
        PID_Reset(&pid[i]);
        __HAL_TIM_SetCounter(MOTOR_TIMER_HANDLE_P[i], 0);
    }

    // 进入PID循环
    for (int encoder_cnt = 0; encoder_cnt <= cnt_max; ) {
        for (int i = 0; i < 4; i ++) {
            /*读取编码器*/
            i_encoder_temp[i] = abs(GetTimEncoder(i));
            // i_encoder_output[i] = i_encoder_temp[i] * 6000 * 1000 / (DT * 4 * 11 * REDUCTION_RATIO * 280); //?
            i_encoder_output[i] = i_encoder_temp[i] * 487.013 / DT / REDUCTION_RATIO;
            /*PID*/
            PID_SetTarget(&pid[i], CCR_target[i]);
            int adjusted_target = CCR_target[i] + PID_Update(&pid[i], i_encoder_output[i]);
            if (adjusted_target > 99) adjusted_target = 99;
            else if (adjusted_target < 0) adjusted_target = -adjusted_target;
            /*设置运动速度*/
            SetRotateSpeed(&motor[i], adjusted_target);
            /*电机运动*/
            Rotate(&motor[i]);
        }
        HAL_Delay(DT);

        if (i_encoder_output[0]) encoder_cnt += (i_encoder_temp[0] + i_encoder_temp[1] + i_encoder_temp[2] + i_encoder_temp[3]) / 4;
        my_printf("channels:%i, %i, %i, %i, %i\n", i_encoder_output[0], i_encoder_output[1], i_encoder_output[2], i_encoder_output[3], encoder_cnt);
    }
    // Finish up
    for (int i = 0; i < 4; i ++) {
        SetRotateSpeed(&motor[i], 0);
        Rotate(&motor[i]);
    }
    return true;
}

bool right(int16_t dist) {
    static int i_encoder_temp[4];
    static int i_encoder_output[4];
    static int CCR_target[4];

    // 零距离平移 = 刹车
    if (dist == 0) {
        Brake();
        return true;
    }

    int dist_positive = abs(dist);
    if (dist_positive <= SHIFT_RIGHT_MIN) dist_positive = 1; // 最小限制
    if (dist > 0) {
        SetRotateDirection(&motor[0], 1);
        SetRotateDirection(&motor[1], -1);
        SetRotateDirection(&motor[2], 1);
        SetRotateDirection(&motor[3], -1);
    } else if (dist < 0) {
        SetRotateDirection(&motor[0], -1);
        SetRotateDirection(&motor[1], 1);
        SetRotateDirection(&motor[2], -1);
        SetRotateDirection(&motor[3], 1);
    }

    // 寻找合适的参数
    int param_set_index = 0;
    for (int i = SHIFT_RIGHT_PARAM_SET_COUNT - 1; i >= 0 ; i--) if (dist_positive >= SHIFT_RIGHT_PARAMS[i][0]) {
        param_set_index = i;
        break;
    }
    for (int i = 0; i < 4; i++) CCR_target[i] = SHIFT_RIGHT_PARAMS[param_set_index][i+1];
    int cnt_max = distToCnt(round(dist_positive + SHIFT_RIGHT_OFFSET[param_set_index]));

    // PID初始化
    for (int i = 0; i < 4; i++) {
        PID_Reset(&pid[i]);
        __HAL_TIM_SetCounter(MOTOR_TIMER_HANDLE_P[i], 0);
    }

   for (int encoder_cnt = 0; encoder_cnt <= cnt_max; ) {
        for (int i = 0; i < 4; i ++) {
            /*读取编码器*/
            i_encoder_temp[i] = GetTimEncoder(i);
            if (i_encoder_temp[i] < 0) i_encoder_temp[i] = - i_encoder_temp[i];
            // i_encoder_output[i] = i_encoder_temp[i] * 6000 * 1000 / DT / 4 / 11 / REDUCTION_RATIO / 280; //?
            i_encoder_output[i] = i_encoder_temp[i] * 487.013 / DT / REDUCTION_RATIO; //?
            /*PID*/
            PID_SetTarget(&pid[i], CCR_target[i]);
            int adjusted_target = CCR_target[i] + PID_Update(&pid[i], i_encoder_output[i]);
            if (adjusted_target > 99) adjusted_target = 99;
            else if (adjusted_target < 0) adjusted_target = -adjusted_target;
            /*设置运动速度*/
            SetRotateSpeed(&motor[i], adjusted_target);
            /*电机运动*/
            Rotate(&motor[i]);
        }
        HAL_Delay(DT);

        if (i_encoder_output[0]) encoder_cnt += (i_encoder_temp[0] + i_encoder_temp[1] + i_encoder_temp[2] + i_encoder_temp[3]) / 4;
        my_printf("channels:%i, %i, %i, %i, %i\n", i_encoder_output[0], i_encoder_output[1], i_encoder_output[2], i_encoder_output[3], encoder_cnt);
    }
    // Finish up
    for (int i = 0; i < 4; i ++) {
        SetRotateSpeed(&motor[i], 0);
        Rotate(&motor[i]);
    }
    return true;
}

#define STEER_DT 33
bool steer(int16_t angle) {
    static int i_encoder_temp[4];
    static int i_encoder_output[4];
    static int CCR_target[4];

    if (angle == 0) return true;

    int angle_positive = abs(angle);
    if (angle_positive <= STEER_MIN) angle_positive = 1; // 最小限制
    if (angle > 0) {
        SetRotateDirection(&motor[0], -1);
        SetRotateDirection(&motor[1], -1);
        SetRotateDirection(&motor[2], 1);
        SetRotateDirection(&motor[3], 1);
    } else if (angle < 0) {
        SetRotateDirection(&motor[0], 1);
        SetRotateDirection(&motor[1], 1);
        SetRotateDirection(&motor[2], -1);
        SetRotateDirection(&motor[3], -1);
    }

    // 寻找合适的参数
    int param_set_index = 0;
    for (int i = STEER_PARAM_SET_COUNT - 1; i >= 0 ; i--) if (angle_positive >= STEER_PARAMS[i][0]) {
        param_set_index = i;
        break;
    }
    for (int i = 0; i < 4; i++) CCR_target[i] = STEER_PARAMS[param_set_index][i+1];
    // 计算真正的编码器目标
    int cnt_max = distToCnt(round((angle_positive + STEER_OFFSET[param_set_index]) * 3.1416 * (LENGTH_HALF + WIDTH_HALF) / 180));

    // PID初始化
    for (int i = 0; i < 4; i++) {
        PID_Reset(&pid[i]);
        __HAL_TIM_SetCounter(MOTOR_TIMER_HANDLE_P[i], 0);
    }

    // 进入PID循环
    for (int encoder_cnt = 0; encoder_cnt <= cnt_max; ) {
        for (int i = 0; i < 4; i ++) {
            /*读取编码器*/
            i_encoder_temp[i] = abs(GetTimEncoder(i));
            // i_encoder_output[i] = i_encoder_temp[i] * 6000 * 1000 / (STEER_DT * 4 * 11 * REDUCTION_RATIO * 280); //?
            i_encoder_output[i] = i_encoder_temp[i] * 487.013 / (STEER_DT * REDUCTION_RATIO); //?
            /*PID*/
            PID_SetTarget(&pid[i], CCR_target[i]);
            int adjusted_target = CCR_target[i] + PID_Update(&pid[i], i_encoder_output[i]);
            if (adjusted_target > 99) adjusted_target = 99;
            else if (adjusted_target < 0) adjusted_target = -adjusted_target;
            /*设置运动速度*/
            SetRotateSpeed(&motor[i], adjusted_target);
            /*电机运动*/
            Rotate(&motor[i]);
        }
        HAL_Delay(STEER_DT);

        if (i_encoder_output[0]) encoder_cnt += (i_encoder_temp[0] + i_encoder_temp[1] + i_encoder_temp[2] + i_encoder_temp[3]) / 4;
        my_printf("channels:%i, %i, %i, %i, %i\n", i_encoder_output[0], i_encoder_output[1], i_encoder_output[2], i_encoder_output[3], encoder_cnt);
    }
    // Finish up
    for (int i = 0; i < 4; i ++) {
        SetRotateSpeed(&motor[i], 0);
        Rotate(&motor[i]);
    }
    return true;
}

bool shift(int16_t _forward, int16_t shift_right) {
    bool ret = forward(_forward);
    return ret && right(shift_right);
}

int distToCnt(int16_t dist) {
    return dist * 4 * 11 * REDUCTION_RATIO / (60 * 3.14);
}
