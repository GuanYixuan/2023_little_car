#ifndef MOVE_H_
#define MOVE_H_

#include <stdint.h>

int GetTimEncoder(int encoder_serial);

void Brake(void);

// 不同距离前进指令所用的参数
// 格式是(min_dist, 四个_ccr_target)
constexpr int forward_params[2][5] = {
    {1, 28, 28, 30, 30},
    {800, 90, 90, 96, 96}
};

void forward(int16_t forward_time);

void right(int16_t right_time);

int distToCnt(int16_t dist);
#endif