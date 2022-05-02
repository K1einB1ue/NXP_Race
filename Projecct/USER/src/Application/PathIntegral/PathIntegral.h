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

//Ϊ�˷�ֹ��������������..����ʹ��ָ�����������ݹ�ͨ..�����һ������ӿڵĽṹ.������Zero-cost abstraction(��ɱ�����),��Ȼ����Щ�洢ָ��Ŀ������32Byte.  ������Ҫ���ͷ�ļ�ʱֻҪɾ�����ͷ�ĵ��ü���.
void PathIntegral_init(float* rotation,float* rotationTo,float* Encoder_FL,float* Encoder_FR,float* Encoder_BL,float* Encoder_BR);
//�������º�������Ҫ���õĶ�ʱ����(����SliceManager�е���)
void PathIntegral_update();
void PathIntegral_PID_buffer_clear();
//ע��!!!!��Ҫ����������Ϊ��˳ʱ��Ϊ��ֵ.
//�趨�Ƕ���Χ�ڶ���ʱ��ʼ��ֱ��(����б��)
//�����Ͻ�ֹ����90 ���������С��2 ����0.01(��������Ӧ��������)
#define Angle_error_range 0.1
//��û�г����ϵ�����,�侫�Ȳ�û�е�λ,��Ҫ�������.��.�� ������.jpg
#define Path_error_range 200
//��ò�Ҫֱ�ӵ���,�Ƽ�ʹ��slice_move,����SliceManager
void PathIntegral_reset();
void PathIntegral_move(Vector2* Position,uint8 PathType);
void PathIntegral_moveLen(float lenth,uint8 PathType);
void PathIntegral_rotateAngle(float angle);
//�������Ϊ���Ϻ����ķ���ֵ.�����Ƕ�����ֵ��.
//������Ʒ���ֵ,�����Ż�PathIntegral_move�����������һ��.(��Ϊ�����ĸı䲻�ᵼ�����������Ϸ����ı� �����䷵��ֵ����һ���������,��Ϊ����ṹ����SliceManager���������.)
//��ò�Ҫֱ�ӵ���,�Ƽ�ʹ��slice_move,����SliceManager
uint8 PathIntegral_getFlag();
uint8 PathIntegral_getMoveFlag();
uint8 PathIntegral_getRotateFlag();





extern ST_PID_POS  m_stMotor_Pos;
extern ST_PID_POS m_stGrid_X_Pos;
extern ST_PID_POS m_stGrid_Y_Pos;


extern float m_f32CurrentTarget_X;
extern float m_f32CurrentTarget_Y;

//�����ƶ�,ע��!��Ҫͬʱ��ʼ��PathIntegral_init!������������ȷִ��.
void Grid_init(float* Tracer_X,float* Tracer_Y,uint8* Tracer_X_line_cnt,uint8* Tracer_Y_line_cnt);
//�������º�������Ҫ���õĶ�ʱ����(����SliceManager�е���)
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
