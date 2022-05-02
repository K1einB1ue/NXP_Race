#pragma once


#include"../../HardWare/HardWare.h"
#include"../SliceManager/SliceManager.h"


#define PathIntegral_MaxSpeed 45
#define Grid_Tracing_With_Line_Count_Speed 45

#define PathIntegral_Index 1

#define PathType_Forward_and_Backward_Index   1
#define PathType_Leftward_and_Rightward_Index (4.5/3.3333)
#define PathType_Angleward_Index (15/11)

extern Vector2 g_vec2_currentPos;

typedef enum{
  PathType_Forward     = 1,
  PathType_Leftward    = 2,
  PathType_F_L         = 3,
  PathType_F_R         = 4,
}PathTypeN_enum;

void Speed_Buffer_Bind(float* Motor_FL,float* Motor_FR,float* Motor_BL,float* Motor_BR);
void Speed_Buffer_Clear();
void Speed_Buffer_Load();

//为了防止程序与程序间的耦合..所以使用指针来进行数据沟通..设计了一个面向接口的结构.几乎是Zero-cost abstraction(零成本抽象),当然会有些存储指针的开销大概32Byte.  当不想要这个头文件时只要删除这个头的调用即可.
void PathIntegral_init(float* rotation,float* rotationTo,float* Encoder_FL,float* Encoder_FR,float* Encoder_BL,float* Encoder_BR);
//驱动以下函数所需要调用的定时更新(包括SliceManager中调用)
void PathIntegral_update();
void PathIntegral_PID_buffer_clear();
//注意!!!!需要调整陀螺仪为至顺时针为负值.
//设定角度误差范围在多少时开始走直线(或者斜走)
//程序上禁止大于90 基本上最好小于2 大于0.01(否则其响应过于慢了)
#define Angle_error_range 0.1
//并没有程序上的限制,其精度并没有单位,需要经验求解.呃.即 差不多得了.jpg
#define Path_error_range 200
//最好不要直接调用,推荐使用slice_move,搭配SliceManager
void PathIntegral_reset();
void PathIntegral_move(Vector2* Position,uint8 PathType);
void PathIntegral_moveLen(float lenth,uint8 PathType);
void PathIntegral_rotateAngle(float angle);
//可以理解为以上函数的返回值.但是是惰性求值的.
//单独设计返回值,可以优化PathIntegral_move不被额外进入一次.(因为函数的改变不会导致物理环节马上发生改变 所以其返回值会晚一个周期求解,而为了其结构不与SliceManager绑定所以如此.)
//最好不要直接调用,推荐使用slice_move,搭配SliceManager
uint8 PathIntegral_getFlag();
uint8 PathIntegral_getMoveFlag();
uint8 PathIntegral_getRotateFlag();





extern ST_PID_POS  m_stMotor_Pos;
extern ST_PID_POS m_stGrid_X_Pos;
extern ST_PID_POS m_stGrid_Y_Pos;


extern float m_f32CurrentTarget_X;
extern float m_f32CurrentTarget_Y;

//网格移动,注意!需要同时初始化PathIntegral_init!否则函数不会正确执行.
void Grid_init(float* Tracer_X,float* Tracer_Y,uint8* Tracer_X_line_cnt,uint8* Tracer_Y_line_cnt);
//驱动以下函数所需要调用的定时更新(包括SliceManager中调用)
void Grid_update();
void Grid_PID_buffer_clear();
void Grid_correctCross();
void Grid_correctX();
void Grid_correctY();
uint8 Grid_getCorrectCrossFlag();
uint8 Grid_getCorrectXFlag();
uint8 Grid_getCorrectYFlag();
void Grid_enable_X_tracing();
void Grid_disable_X_tracing();
void Grid_enable_Y_tracing();
void Grid_disable_Y_tracing();

void Grid_X_tracing(int Line_Y_cnt);
void Grid_Y_tracing(int Line_X_cnt);
uint8 Grid_getX_tracingFlag();
uint8 Grid_getY_tracingFlag();
