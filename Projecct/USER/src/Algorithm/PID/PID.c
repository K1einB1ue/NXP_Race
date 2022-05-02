#include"./PID.h"

void PID_POS_init(ST_PID_POS* PID,float P,float I,float D){
  PID->private_m_f32error=0;
  PID->private_m_f32derivative=0;
  PID->private_m_f32integral=0;
  PID->private_m_f32pre_error=0;
  PID->m_f32Kp=P;
  PID->m_f32Ki=I;
  PID->m_f32Kd=D;
}

void PID_POS_update_PID(ST_PID_POS* PID){
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //绝对误差计算
  PID->private_m_f32integral+=PID->private_m_f32error;                                     //累积误差作为积分项)(当存在微小误差时,可以累积起来,使用KI调整)
  PID->private_m_f32derivative=(PID->private_m_f32error-PID->private_m_f32pre_error);          //得到微分项(当距离很近时,降低KP的影响,防止KP震荡)
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*PID->private_m_f32error+
      PID->m_f32Ki*PID->private_m_f32integral+
        PID->m_f32Kd*PID->private_m_f32derivative;
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //保留误差
}

void PID_POS_update_PD(ST_PID_POS* PID){
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //绝对误差计算
  PID->private_m_f32derivative=(PID->private_m_f32error-PID->private_m_f32pre_error);          //得到微分项(当距离很近时,降低KP的影响,防止KP震荡)
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*PID->private_m_f32error+
      PID->m_f32Kd*PID->private_m_f32derivative;
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //保留误差
}

void PID_POS_update_P(ST_PID_POS* PID){
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                              
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*PID->private_m_f32error;
}

void PID_POS_buffer_clear(ST_PID_POS* PID){
  PID->private_m_f32integral=0;
  PID->private_m_f32error=0;
  PID->private_m_f32derivative=0;
  *PID->m_f32ptr_output=0;
}

void PID_INC_init(ST_PID_INC* PID,float P,float I,float D){
  PID->private_m_f32error=0;
  PID->private_m_f32pre_error=0;
  PID->private_m_f32pre_pre_error=0;
  PID->m_f32Kp=P;
  PID->m_f32Ki=I;
  PID->m_f32Kd=D;
}

void PID_INC_update_PID(ST_PID_INC* PID){
  PID->private_m_f32pre_pre_error=PID->private_m_f32pre_error;                                          //保留上次误差
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //保留误差
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //绝对误差计算
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*(PID->private_m_f32error-PID->private_m_f32pre_error)+
      PID->m_f32Ki*(PID->private_m_f32error)+
        PID->m_f32Kd*(PID->private_m_f32error-2*PID->private_m_f32pre_error+PID->private_m_f32pre_pre_error);
}

void PID_INC_update_PD(ST_PID_INC* PID){
  PID->private_m_f32pre_pre_error=PID->private_m_f32pre_error;                                          //保留上次误差
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //保留误差
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //绝对误差计算
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*(PID->private_m_f32error-PID->private_m_f32pre_error)+
      PID->m_f32Kd*(PID->private_m_f32error-2*PID->private_m_f32pre_error+PID->private_m_f32pre_pre_error);
}

void PID_INC_update_P(ST_PID_INC* PID){
  PID->private_m_f32pre_pre_error=PID->private_m_f32pre_error;                                          //保留上次误差
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //保留误差
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //绝对误差计算
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*(PID->private_m_f32error-PID->private_m_f32pre_error);
}

void PID_INC_buffer_clear(ST_PID_INC* PID){
  PID->private_m_f32pre_pre_error=0;
  PID->private_m_f32pre_error=0;
  PID->private_m_f32error=0;
  *PID->m_f32ptr_output=0;
}

