#pragma once
#include"../Dependency.h"

void HWT101_init();
float getAngle(float interestAngle);
float getLoopAngle(float angle,float interestAngle);
extern float g_f32HWT101_Direction;
void HWT101_update();
#ifdef HWT101_ResetPin
void HWT101_Z_reset();
#endif