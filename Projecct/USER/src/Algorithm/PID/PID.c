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
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //����������
  PID->private_m_f32integral+=PID->private_m_f32error;                                     //�ۻ������Ϊ������)(������΢С���ʱ,�����ۻ�����,ʹ��KI����)
  PID->private_m_f32derivative=(PID->private_m_f32error-PID->private_m_f32pre_error);          //�õ�΢����(������ܽ�ʱ,����KP��Ӱ��,��ֹKP��)
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*PID->private_m_f32error+
      PID->m_f32Ki*PID->private_m_f32integral+
        PID->m_f32Kd*PID->private_m_f32derivative;
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //�������
}

void PID_POS_update_PD(ST_PID_POS* PID){
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //����������
  PID->private_m_f32derivative=(PID->private_m_f32error-PID->private_m_f32pre_error);          //�õ�΢����(������ܽ�ʱ,����KP��Ӱ��,��ֹKP��)
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*PID->private_m_f32error+
      PID->m_f32Kd*PID->private_m_f32derivative;
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //�������
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
  PID->private_m_f32pre_pre_error=PID->private_m_f32pre_error;                                          //�����ϴ����
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //�������
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //����������
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*(PID->private_m_f32error-PID->private_m_f32pre_error)+
      PID->m_f32Ki*(PID->private_m_f32error)+
        PID->m_f32Kd*(PID->private_m_f32error-2*PID->private_m_f32pre_error+PID->private_m_f32pre_pre_error);
}

void PID_INC_update_PD(ST_PID_INC* PID){
  PID->private_m_f32pre_pre_error=PID->private_m_f32pre_error;                                          //�����ϴ����
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //�������
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //����������
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*(PID->private_m_f32error-PID->private_m_f32pre_error)+
      PID->m_f32Kd*(PID->private_m_f32error-2*PID->private_m_f32pre_error+PID->private_m_f32pre_pre_error);
}

void PID_INC_update_P(ST_PID_INC* PID){
  PID->private_m_f32pre_pre_error=PID->private_m_f32pre_error;                                          //�����ϴ����
  PID->private_m_f32pre_error=PID->private_m_f32error;                                                  //�������
  PID->private_m_f32error=*(PID->m_f32ptr_ideal)-*(PID->m_f32ptr_real);                                 //����������
  *(PID->m_f32ptr_output)=
    PID->m_f32Kp*(PID->private_m_f32error-PID->private_m_f32pre_error);
}

void PID_INC_buffer_clear(ST_PID_INC* PID){
  PID->private_m_f32pre_pre_error=0;
  PID->private_m_f32pre_error=0;
  PID->private_m_f32error=0;
  *PID->m_f32ptr_output=0;
}

