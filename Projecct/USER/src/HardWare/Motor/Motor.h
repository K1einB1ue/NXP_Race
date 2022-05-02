#pragma once

#include"../Dependency.h"

//配置电机是否需要旋转PID*******
//如果需要更加精确 需要先调用Rotation_update再调用Motor_update,但是实际上可以忽略不计;
#define Motor_Rotation_Enable 1

//调用接口************************************
//完美的电机只要最简单的烹饪方式(?雾
void Motor_init();
//将更新Motor的编码器速度环(注意:需要为一个定时器调用!否则其PID是不可靠的.)
void Motor_update();


//调整方式************************************
//(需要调用Motor_init与Motor_update)
//0.调整轮子为X形安装
//1.打开Motor_EncoderTestMode并且使轮子方向为向前. 调整宏定义使所有的PID输入读数为正.
#define Motor_EncoderTestMode 0
#define Motor_FL_Encoder_Reverse 0
#define Motor_FR_Encoder_Reverse 1
#define Motor_BL_Encoder_Reverse 0
#define Motor_BR_Encoder_Reverse 1
//2.打开Motor_MotorTestMode观察轮子运动方向. 调整宏定义使轮胎对应转向:0->向前转的,1->向后转的.
#define Motor_MotorTestMode 0
#define Motor_FL_ON  1
#define Motor_FR_ON  1
#define Motor_BL_ON  1
#define Motor_BR_ON  1
#define Motor_FL_Motor_Reverse 1
#define Motor_FR_Motor_Reverse 0
#define Motor_BL_Motor_Reverse 0
#define Motor_BR_Motor_Reverse 0
//3.如果需要Motor_Rotaion_Enable.需要调整陀螺仪为至顺时针为负值.



#if Motor_Rotation_Enable 
  //在这个文件外部只需要,定时调用Motor_update即可.对这些变量赋值即可改变电机运动状态.
  //注意!Motor_update调用的时间间隔将会对PID参数产生影响!
  //如果启用,rotation_ptr需要绑定航偏角. 务必调用Rotaion_bind!
  extern float g_f32Motor_Speed_Index_FL;
  extern float g_f32Motor_Speed_Index_FR;
  extern float g_f32Motor_Speed_Index_BL;
  extern float g_f32Motor_Speed_Index_BR;

  extern float g_f32Motor_RealAngle;
  
  extern float g_f32Motor_Rotation;
  
  extern uint8 g_u8RotationEnable;
  //旋转时电机最高的转速!不同的电机和编码器的组合会产生不同! 
  #define Motor_Rotation_Speed_Max 35
  
  void Rotation_PID_init(float (*rotation_ptr)(float));
  void Rotation_update();
#else
  //在这个文件外部只需要,定时调用Motor_update即可.对这些变量赋值即可改变电机运动状态.
  //注意!Motor_update调用的时间间隔将会对PID参数产生影响!
  extern float g_f32Motor_Speed_FL;
  extern float g_f32Motor_Speed_FR;
  extern float g_f32Motor_Speed_BL;
  extern float g_f32Motor_Speed_BR;
#endif

  
//PWM记数上限(其设置上限需要参考数据手册,几乎没有更改的必要)
#define Motor_Encoder_Speed_Max 50000
  
#define Speed_Death_Zone 2.0
#define Section_Rotation_PID 1

#if Section_Rotation_PID
  #define Section_Point_Angle_Error 1.0
#endif

void Speed_clear();
void Motor_PID_buffer_clear();
//不推荐用户调用..
#if Motor_Rotation_Enable
extern ST_PID_POS  m_stMotor_Direction;
#endif
extern ST_PID_POS  m_stMotor_FL;
extern ST_PID_POS  m_stMotor_FR;
extern ST_PID_POS  m_stMotor_BL;
extern ST_PID_POS  m_stMotor_BR;
