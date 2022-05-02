#pragma once
#include"../Dependency.h"

//Tracer_X��װ��ƽ����ǰ����
//Tracer_Y��װ�ڴ�ֱ��ǰ����
//�����߾�����λ�����Ľ���

//����Tracer����
#define Tracer_Test 0
//Tracer_X��ǰƫʱg_i32Arg_X��С����Ϊ0,��֮��1
#define Tracer_X_Reverse 0
//Tracer_Y����ƫg_i32Arg_X��С����Ϊ0,��֮��1
#define Tracer_Y_Reverse 0

extern float g_f32TracerArg_X;
extern float g_f32TracerArg_Y;
extern uint8 g_u8Tracer_X_line_cnt;
extern uint8 g_u8Tracer_Y_line_cnt;

void Tracer_init();
void Tracer_update();

