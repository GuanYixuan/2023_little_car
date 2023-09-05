#ifndef MOVE_H_
#define MOVE_H_
#include "main.h"
#include "motor.h"

void Brake(void);

void forward(int16_t forward_time);

void right(int16_t right_time);

int distToCnt(int16_t dist);
#endif