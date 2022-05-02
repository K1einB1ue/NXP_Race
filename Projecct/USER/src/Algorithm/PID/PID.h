#pragma once

#include"headfile.h"
//author:@Klein_Blue QQ:524036364

//考虑使用一个面向对象的结构

/*   位置式PID结构体   */
//优点:
//位置式PID是一种非递推式算法,可直接控制执行机构（如平衡小车）,u(k)的值和执行机构的实际位置(如小车当前角度)是一一对应的,因此在执行机构不带积分部件的对象中可以很好应用
//缺点:
//每次输出均与过去的状态有关，计算时要对e(k)进行累加，运算工作量大

//注意!如果对输出限幅度,务必给积分项限制幅度!即:积分饱和过深.除非使用PD式PID

typedef struct _st_pid_pos_{
  float   m_f32Kp;                      //比例系数
  float   m_f32Ki;                      //积分系数
  float   m_f32Kd;                      //微分系数
  
  float  private_m_f32pre_error;        //上次误差
  float  private_m_f32error;            //误差项
  float  private_m_f32integral;         //积分项
  float  private_m_f32derivative;       //微分项
  
  float* m_f32ptr_output;               //输出
  float* m_f32ptr_real;                 //实际值
  float* m_f32ptr_ideal;                //理想值
  
} ST_PID_POS;

//理论上完全等价,但是在实际操作中增量似乎会更好一些?
/*   增量式PID结构体   */
//针对带积分的系统 比如电机
//优点:
//误动作时影响小，必要时可用逻辑判断的方法去掉出错数据
//手动/自动切换时冲击小，便于实现无扰动切换。当计算机故障时，仍能保持原值
//算式中不需要累加。控制增量Δu(k)的确定仅与最近3次的采样值有关
//缺点:
//积分截断效应大，有稳态误差
//溢出的影响大。有的被控对象用增量式则不太好

typedef struct _st_pid_inc_{
  float   m_f32Kp;                      //比例系数
  float   m_f32Ki;                      //积分系数
  float   m_f32Kd;                      //微分系数
  
  float  private_m_f32pre_pre_error;    //上上次误差
  float  private_m_f32pre_error;        //上次误差
  float  private_m_f32error;            //误差项
  
  float* m_f32ptr_output;               //输出
  float* m_f32ptr_real;                 //实际值
  float* m_f32ptr_ideal;                //理想值
  
} ST_PID_INC;


//使用时需要在update之前绑定结构体内部指针.
//并且调用PID_***_init初始化结构体(如果手动赋值 需要对error等值做清零初始化)
void PID_POS_init(ST_PID_POS* PID,float P,float I,float D);
void PID_POS_update_PID(ST_PID_POS* PID);
void PID_POS_update_PD(ST_PID_POS* PID);
void PID_POS_update_P(ST_PID_POS* PID);
void PID_POS_buffer_clear(ST_PID_POS* PID);

void PID_INC_init(ST_PID_INC* PID,float P,float I,float D);
void PID_INC_update_PID(ST_PID_INC* PID);
void PID_INC_update_PD(ST_PID_INC* PID);
void PID_INC_update_P(ST_PID_INC* PID);
void PID_INC_buffer_clear(ST_PID_INC* PID);


