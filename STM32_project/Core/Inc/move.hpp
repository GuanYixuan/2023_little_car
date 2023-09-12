#ifndef MOVE_H_
#define MOVE_H_

#include <stdint.h>

// 前进指令参数组数
constexpr int FORWARD_PARAM_SET_COUNT = 5;
// 前进指令的最短距离, 小于等于此值则会被设置为1
constexpr int FORWARD_MIN = 12;
// 不同距离前进指令所用的参数, 格式是(min_dist, 四个_ccr_target)
constexpr int FORWARD_PARAMS[FORWARD_PARAM_SET_COUNT][5] = {
    {0, 20, 20, 20, 21},
    {40, 20, 20, 20, 21},
    {120, 30, 30, 31, 31},
    {500, 50, 50, 51, 52},
    {1000, 80, 80, 83, 83}
};
// 不同距离前进指令所加的距离调整系数
constexpr float FORWARD_OFFSET[FORWARD_PARAM_SET_COUNT] = {0, -30, -60, -140, -350};

// 右移指令参数组数
constexpr int SHIFT_RIGHT_PARAM_SET_COUNT = 5;
// 右移指令的最短距离, 小于等于此值则会被设置为1
constexpr int SHIFT_RIGHT_MIN = 10;
// 不同距离右移指令所用的参数, 格式是(min_dist, 四个_ccr_target)
constexpr int SHIFT_RIGHT_PARAMS[SHIFT_RIGHT_PARAM_SET_COUNT][5] = {
    {0, 20, 20, 21, 20},
    {40, 20, 20, 20, 21},
    {120, 30, 30, 31, 30},
    {500, 50, 50, 51, 50},
    {1000, 80, 81, 81, 80}
};
// 不同距离右移指令所加的距离调整系数
constexpr float SHIFT_RIGHT_OFFSET[SHIFT_RIGHT_PARAM_SET_COUNT] = {0, -30, -40, -60, -100};

// 转弯指令参数组数
constexpr int STEER_PARAM_SET_COUNT = 3;
// 转弯指令的最小转角, 小于等于此值则会被设置为1
constexpr int STEER_MIN = 2;
// 不同角度转弯指令所用的参数, 格式是(min_angle, 四个_ccr_target)
constexpr int STEER_PARAMS[STEER_PARAM_SET_COUNT][5] = {
    {0, 20, 20, 20, 21},
    {15, 30, 30, 30, 31},
    {90, 40, 40, 41, 41},
};
// 不同角度转弯指令所加的距离调整系数
constexpr float STEER_OFFSET[STEER_PARAM_SET_COUNT] = {-1, -7, -10};

int GetTimEncoder(int encoder_serial);
void my_printf(const char* format, ...);

// 中间层接口, 让小车进入刹车状态
void Brake(void);

// 中间层接口, 让小车前进指定的毫米数, 负数表示后退
bool forward(int16_t forward_time);

// 中间层接口, 让小车向右平移指定的毫米数, 负数表示左移
bool right(int16_t right_time);

// 上层接口, 让小车平移指定距离
bool shift(int16_t forward, int16_t shift_right);

// 上层接口, 让小车逆时针旋转指定的角度, 负数表示顺时针旋转
bool steer(int16_t right_time);

int distToCnt(int16_t dist);
#endif