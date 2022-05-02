#pragma once

#include"headfile.h"
//author:@Klein_Blue QQ:524036364

//����ʹ��һ���������Ľṹ

/*   λ��ʽPID�ṹ��   */
//�ŵ�:
//λ��ʽPID��һ�ַǵ���ʽ�㷨,��ֱ�ӿ���ִ�л�������ƽ��С����,u(k)��ֵ��ִ�л�����ʵ��λ��(��С����ǰ�Ƕ�)��һһ��Ӧ��,�����ִ�л����������ֲ����Ķ����п��Ժܺ�Ӧ��
//ȱ��:
//ÿ����������ȥ��״̬�йأ�����ʱҪ��e(k)�����ۼӣ����㹤������

//ע��!���������޷���,��ظ����������Ʒ���!��:���ֱ��͹���.����ʹ��PDʽPID

typedef struct _st_pid_pos_{
  float   m_f32Kp;                      //����ϵ��
  float   m_f32Ki;                      //����ϵ��
  float   m_f32Kd;                      //΢��ϵ��
  
  float  private_m_f32pre_error;        //�ϴ����
  float  private_m_f32error;            //�����
  float  private_m_f32integral;         //������
  float  private_m_f32derivative;       //΢����
  
  float* m_f32ptr_output;               //���
  float* m_f32ptr_real;                 //ʵ��ֵ
  float* m_f32ptr_ideal;                //����ֵ
  
} ST_PID_POS;

//��������ȫ�ȼ�,������ʵ�ʲ����������ƺ������һЩ?
/*   ����ʽPID�ṹ��   */
//��Դ����ֵ�ϵͳ ������
//�ŵ�:
//����ʱӰ��С����Ҫʱ�����߼��жϵķ���ȥ����������
//�ֶ�/�Զ��л�ʱ���С������ʵ�����Ŷ��л��������������ʱ�����ܱ���ԭֵ
//��ʽ�в���Ҫ�ۼӡ�����������u(k)��ȷ���������3�εĲ���ֵ�й�
//ȱ��:
//���ֽض�ЧӦ������̬���
//�����Ӱ����еı��ض���������ʽ��̫��

typedef struct _st_pid_inc_{
  float   m_f32Kp;                      //����ϵ��
  float   m_f32Ki;                      //����ϵ��
  float   m_f32Kd;                      //΢��ϵ��
  
  float  private_m_f32pre_pre_error;    //���ϴ����
  float  private_m_f32pre_error;        //�ϴ����
  float  private_m_f32error;            //�����
  
  float* m_f32ptr_output;               //���
  float* m_f32ptr_real;                 //ʵ��ֵ
  float* m_f32ptr_ideal;                //����ֵ
  
} ST_PID_INC;


//ʹ��ʱ��Ҫ��update֮ǰ�󶨽ṹ���ڲ�ָ��.
//���ҵ���PID_***_init��ʼ���ṹ��(����ֶ���ֵ ��Ҫ��error��ֵ�������ʼ��)
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


