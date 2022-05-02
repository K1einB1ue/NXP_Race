#pragma once

#include"../Dependency.h"

//���õ���Ƿ���Ҫ��תPID*******
//�����Ҫ���Ӿ�ȷ ��Ҫ�ȵ���Rotation_update�ٵ���Motor_update,����ʵ���Ͽ��Ժ��Բ���;
#define Motor_Rotation_Enable 1

//���ýӿ�************************************
//�����ĵ��ֻҪ��򵥵���⿷�ʽ(?��
void Motor_init();
//������Motor�ı������ٶȻ�(ע��:��ҪΪһ����ʱ������!������PID�ǲ��ɿ���.)
void Motor_update();


//������ʽ************************************
//(��Ҫ����Motor_init��Motor_update)
//0.��������ΪX�ΰ�װ
//1.��Motor_EncoderTestMode����ʹ���ӷ���Ϊ��ǰ. �����궨��ʹ���е�PID�������Ϊ��.
#define Motor_EncoderTestMode 0
#define Motor_FL_Encoder_Reverse 0
#define Motor_FR_Encoder_Reverse 1
#define Motor_BL_Encoder_Reverse 0
#define Motor_BR_Encoder_Reverse 1
//2.��Motor_MotorTestMode�۲������˶�����. �����궨��ʹ��̥��Ӧת��:0->��ǰת��,1->���ת��.
#define Motor_MotorTestMode 0
#define Motor_FL_ON  1
#define Motor_FR_ON  1
#define Motor_BL_ON  1
#define Motor_BR_ON  1
#define Motor_FL_Motor_Reverse 1
#define Motor_FR_Motor_Reverse 0
#define Motor_BL_Motor_Reverse 0
#define Motor_BR_Motor_Reverse 0
//3.�����ҪMotor_Rotaion_Enable.��Ҫ����������Ϊ��˳ʱ��Ϊ��ֵ.



#if Motor_Rotation_Enable 
  //������ļ��ⲿֻ��Ҫ,��ʱ����Motor_update����.����Щ������ֵ���ɸı����˶�״̬.
  //ע��!Motor_update���õ�ʱ���������PID��������Ӱ��!
  //�������,rotation_ptr��Ҫ�󶨺�ƫ��. ��ص���Rotaion_bind!
  extern float g_f32Motor_Speed_Index_FL;
  extern float g_f32Motor_Speed_Index_FR;
  extern float g_f32Motor_Speed_Index_BL;
  extern float g_f32Motor_Speed_Index_BR;

  extern float g_f32Motor_RealAngle;
  
  extern float g_f32Motor_Rotation;
  
  extern uint8 g_u8RotationEnable;
  //��תʱ�����ߵ�ת��!��ͬ�ĵ���ͱ���������ϻ������ͬ! 
  #define Motor_Rotation_Speed_Max 35
  
  void Rotation_PID_init(float (*rotation_ptr)(float));
  void Rotation_update();
#else
  //������ļ��ⲿֻ��Ҫ,��ʱ����Motor_update����.����Щ������ֵ���ɸı����˶�״̬.
  //ע��!Motor_update���õ�ʱ���������PID��������Ӱ��!
  extern float g_f32Motor_Speed_FL;
  extern float g_f32Motor_Speed_FR;
  extern float g_f32Motor_Speed_BL;
  extern float g_f32Motor_Speed_BR;
#endif

  
//PWM��������(������������Ҫ�ο������ֲ�,����û�и��ĵı�Ҫ)
#define Motor_Encoder_Speed_Max 50000
  
#define Speed_Death_Zone 2.0
#define Section_Rotation_PID 1

#if Section_Rotation_PID
  #define Section_Point_Angle_Error 1.0
#endif

void Speed_clear();
void Motor_PID_buffer_clear();
//���Ƽ��û�����..
#if Motor_Rotation_Enable
extern ST_PID_POS  m_stMotor_Direction;
#endif
extern ST_PID_POS  m_stMotor_FL;
extern ST_PID_POS  m_stMotor_FR;
extern ST_PID_POS  m_stMotor_BL;
extern ST_PID_POS  m_stMotor_BR;
