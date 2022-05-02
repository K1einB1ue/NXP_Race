#pragma once
#include"../Dependency.h"

//Tracer_X安装于平行正前方向
//Tracer_Y安装于垂直正前方向
//且两者均不可位于中心交点

//开启Tracer测试
#define Tracer_Test 0
//Tracer_X向前偏时g_i32Arg_X变小则设为0,反之设1
#define Tracer_X_Reverse 0
//Tracer_Y向左偏g_i32Arg_X变小则设为0,反之设1
#define Tracer_Y_Reverse 0

extern float g_f32TracerArg_X;
extern float g_f32TracerArg_Y;
extern uint8 g_u8Tracer_X_line_cnt;
extern uint8 g_u8Tracer_Y_line_cnt;

void Tracer_init();
void Tracer_update();

