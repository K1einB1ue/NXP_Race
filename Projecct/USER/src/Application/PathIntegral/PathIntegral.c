#include"./PathIntegral.h"
//控制电机指针
float* m_f32Motor_FL;
float* m_f32Motor_FR;
float* m_f32Motor_BL;
float* m_f32Motor_BR;

float m_f32SpeedBuffer_FL;
float m_f32SpeedBuffer_FR;
float m_f32SpeedBuffer_BL;
float m_f32SpeedBuffer_BR;

void Speed_Buffer_Bind(float* Motor_FL,float* Motor_FR,float* Motor_BL,float* Motor_BR){
  m_f32Motor_FL=Motor_FL;
  m_f32Motor_FR=Motor_FR;
  m_f32Motor_BL=Motor_BL;
  m_f32Motor_BR=Motor_BR;
}
void Speed_Buffer_Clear(){
  m_f32SpeedBuffer_FL=0;
  m_f32SpeedBuffer_FR=0;
  m_f32SpeedBuffer_BL=0;
  m_f32SpeedBuffer_BR=0;
}
void Speed_Buffer_Load(){
  *m_f32Motor_FL=m_f32SpeedBuffer_FL;
  *m_f32Motor_FR=m_f32SpeedBuffer_FR;
  *m_f32Motor_BL=m_f32SpeedBuffer_BL;
  *m_f32Motor_BR=m_f32SpeedBuffer_BR;
}




//路径积分启用标志位
uint8 PathLock;
//移动启用标志位
uint8 MoveLock;
//旋转启用标志位
uint8 RotateLock;

typedef enum{
  IDLE     = 0,
  ROTATE   = 1,
  MOVE     = 2,
}PathIntegral_State; 

ST_PID_POS  m_stMotor_Pos;   

//当前位置
Vector2 g_vec2_currentPos;
//获得陀螺仪偏航角的指针
float* m_f32RealRotation;
//PID旋转环控制指针
float* m_f32ToRotation;
//编码器读数指针
float* m_f32Encoder_FL;
float* m_f32Encoder_FR;
float* m_f32Encoder_BL;
float* m_f32Encoder_BR;




//位置PID输出.每次都是先旋转然后进行位置PID调整(可能不同方向有不同的积分系数)
float m_f32SpeedOutput; //PID接口的输出项
//积分距离
float m_f32PathIntegral;//PID接口的实际项
//目标距离
float m_f32TargetPathBuffer; //缓存
float m_f32TargetPath;  //PID接口的目标项
//移动方式.
uint8  m_u8PathType;

uint64 Path_Cnt=0;
uint8 Path_Cnt_Lock=0;


uint8 Path_time_check(unsigned int checkTime){
  if(!Path_Cnt_Lock){
    Path_Cnt_Lock=1;
    Path_Cnt=0;
  }
  if(Path_Cnt>=checkTime){
    Path_Cnt_Lock=0;
    Path_Cnt=0;
    return 1;
  }
  return 0;
}

void Path_time_break(){
  Path_Cnt_Lock=0;
  Path_Cnt=0;
}

//状态机
PathIntegral_State m_enumState;


void PathIntegral_init(float* rotation,float* rotationTo,
                       float* Encoder_FL,float* Encoder_FR,float* Encoder_BL,float* Encoder_BR){
  m_f32RealRotation=rotation;
  m_f32ToRotation=rotationTo;
  m_f32Encoder_FL=Encoder_FL;
  m_f32Encoder_FR=Encoder_FR;
  m_f32Encoder_BL=Encoder_BL;
  m_f32Encoder_BR=Encoder_BR;
                         
  PID_POS_init(&m_stMotor_Pos,0.03,0,00.2);
  m_stMotor_Pos.m_f32ptr_output=&m_f32SpeedOutput;
  m_stMotor_Pos.m_f32ptr_real=&m_f32PathIntegral;
  m_stMotor_Pos.m_f32ptr_ideal=&m_f32TargetPath;
}

void PathIntegral_reset(){
  g_vec2_currentPos.x=0;
  g_vec2_currentPos.y=0;
}

void Rotate_update_wait(){
  
  //到了允许的误差角度区间 才开始下一步的移动
  float maxRange=*m_f32ToRotation+Angle_error_range;
  float minRange=*m_f32ToRotation-Angle_error_range;
  
  float realRotation=*m_f32RealRotation;
  if(maxRange>180.0){
    if(realRotation<0){
      realRotation+=360;
    }
  }else if(minRange<-180.0){
    if(realRotation>0){
      realRotation-=360;
    }
  }
  
  
  //等待车转到特定的角度
  if(realRotation<maxRange&&realRotation>minRange){
    if(Path_time_check(20)){
      m_enumState=MOVE;
      m_f32TargetPath=m_f32TargetPathBuffer;
    }
  }else{
    Path_time_break();
  }
}

void Move_update(float forward,float leftward,float tempCos,float tempSin){
  //先根据前进方式进行校准..否则其积分距离在哪个向量上是不明确的..
  switch(m_u8PathType){
    //如果向前行驶
    case PathType_Forward:{
      //在前进时.
      //如果进行加和可以把旋转角度矫正所带来的部分误差中和.即两正两负.
      m_f32PathIntegral+=forward*PathIntegral_Index;
      PID_POS_update_PID(&m_stMotor_Pos);

      if(m_f32SpeedOutput>=PathIntegral_MaxSpeed){
        m_f32SpeedOutput=PathIntegral_MaxSpeed;
      }else if(m_f32SpeedOutput<=-PathIntegral_MaxSpeed){
        m_f32SpeedOutput=-PathIntegral_MaxSpeed;
      }

      m_f32SpeedBuffer_FL+=m_f32SpeedOutput;
      m_f32SpeedBuffer_FR+=m_f32SpeedOutput;
      m_f32SpeedBuffer_BL+=m_f32SpeedOutput;
      m_f32SpeedBuffer_BR+=m_f32SpeedOutput;
      break;
    }
    case PathType_Leftward:{
      //在前进时.
      //如果进行加和可以把旋转角度矫正所带来的部分误差中和.即两正两负.
      m_f32PathIntegral+=leftward*PathIntegral_Index;
      PID_POS_update_PID(&m_stMotor_Pos);
      
      if(m_f32SpeedOutput>=PathIntegral_MaxSpeed){
        m_f32SpeedOutput=PathIntegral_MaxSpeed;
      }else if(m_f32SpeedOutput<=-PathIntegral_MaxSpeed){
        m_f32SpeedOutput=-PathIntegral_MaxSpeed;
      }
      m_f32SpeedBuffer_FL+=-m_f32SpeedOutput;
      m_f32SpeedBuffer_FR+=m_f32SpeedOutput;
      m_f32SpeedBuffer_BL+=m_f32SpeedOutput;
      m_f32SpeedBuffer_BR+=-m_f32SpeedOutput;
      break;
    }
    case PathType_F_L:{
      //在前进时.
      //如果进行加和可以把旋转角度矫正所带来的部分误差中和.即两正两负.
      m_f32PathIntegral+=((leftward+forward)/2)*PathIntegral_Index;
      PID_POS_update_PID(&m_stMotor_Pos);
      
      if(m_f32SpeedOutput>=PathIntegral_MaxSpeed){
        m_f32SpeedOutput=PathIntegral_MaxSpeed;
      }else if(m_f32SpeedOutput<=-PathIntegral_MaxSpeed){
        m_f32SpeedOutput=-PathIntegral_MaxSpeed;
      }
      m_f32SpeedBuffer_FR+=m_f32SpeedOutput;
      m_f32SpeedBuffer_BL+=m_f32SpeedOutput;
      break;
    }
    case PathType_F_R:{
      //在前进时.
      //如果进行加和可以把旋转角度矫正所带来的部分误差中和.即两正两负.
      m_f32PathIntegral+=((-leftward+forward)/2)*PathIntegral_Index;
      PID_POS_update_PID(&m_stMotor_Pos);
      
      if(m_f32SpeedOutput>=PathIntegral_MaxSpeed){
        m_f32SpeedOutput=PathIntegral_MaxSpeed;
      }else if(m_f32SpeedOutput<=-PathIntegral_MaxSpeed){
        m_f32SpeedOutput=-PathIntegral_MaxSpeed;
      }
      m_f32SpeedBuffer_FL+=m_f32SpeedOutput;
      m_f32SpeedBuffer_BR+=m_f32SpeedOutput;
      break;
    }
  }
}

void PathIntegral_update(){
  if(Path_Cnt_Lock){
    Path_Cnt++;
  }
  
  //Fowward带方向加上Leftward带方向 加上YAW则可完成二维平面的映射
  float forward,leftward,tempCos,tempSin;
  float Speed_FL=(*m_f32Encoder_FL);
  float Speed_FR=(*m_f32Encoder_FR);
  float Speed_BL=(*m_f32Encoder_BL);
  float Speed_BR=(*m_f32Encoder_BR);
  
  // float SpeedCnt=0.0;
  // SpeedCnt+=fabsf(Speed_FL)>10?1.0:0.0;
  // SpeedCnt+=fabsf(Speed_FR)>10?1.0:0.0;
  // SpeedCnt+=fabsf(Speed_BL)>10?1.0:0.0;
  // SpeedCnt+=fabsf(Speed_BR)>10?1.0:0.0;

  float Speed_0 = Speed_FL + Speed_BR;
  float Speed_1 = Speed_FR + Speed_BL;
  
  forward = Speed_0+Speed_1;
  leftward = Speed_1-Speed_0;
  
  /*
  if(SpeedCnt<0.5){
    forward=0.0;
    leftward=0.0;
  }else{
    forward   = ((Speed_FL+Speed_FR+Speed_BL+Speed_BR)/SpeedCnt)*4;
    leftward  =((-Speed_FL+Speed_FR+Speed_BL-Speed_BR)/SpeedCnt)*4;
  }*/
  
  tempCos=cosf(3.14159*(*m_f32RealRotation)/180.0);
  tempSin=sinf(3.14159*(*m_f32RealRotation)/180.0);
  
  //完全按照正运动学解算得到的结果
  g_vec2_currentPos.x += forward*tempCos-leftward*tempSin;
  g_vec2_currentPos.y += forward*tempSin+leftward*tempCos;
  
  switch(m_enumState){
  case IDLE:
    break;
  case ROTATE:
    if(PathLock){
      Rotate_update_wait();
    }
    break;
  case MOVE:
    Move_update(forward,leftward,tempCos,tempSin);
    break;
  default:
    break;
  }
}

void PathIntegral_PID_buffer_clear(){
  PID_POS_buffer_clear(&m_stMotor_Pos);
}


void PathIntegral_move(Vector2* Position,uint8 PathType){
  //如果还在移动
  if(PathLock){
    return;
  }
  
  //说明车已经准备开始移动了,锁存状态
  PathLock=1;
  //初始化状态机
  m_enumState=ROTATE;
  //勾股定律算出距离
  float tempPath=sqrt((Position->y-g_vec2_currentPos.y)*(Position->y-g_vec2_currentPos.y)+(Position->x-g_vec2_currentPos.x)*(Position->x-g_vec2_currentPos.x));
  
  //归零路径积分
  m_f32PathIntegral=0;
  //存储移动形式(因为不同的移动会使用不同的积分方式,所以需要保存)
  m_u8PathType=PathType;
  
  switch(m_u8PathType){
    case PathType_Forward:{
      *m_f32ToRotation=atan2(Position->y-g_vec2_currentPos.y,Position->x-g_vec2_currentPos.x)*(180/3.14159);
      m_f32TargetPathBuffer=tempPath*PathType_Forward_and_Backward_Index*PathIntegral_Index;
      break;
    }
    case PathType_Leftward:{
      float tempAngle=atan2(Position->y-g_vec2_currentPos.y,Position->x-g_vec2_currentPos.x)*(180/3.14159)-90;
      *m_f32ToRotation=tempAngle<-180.0?tempAngle+360.0:tempAngle;
      m_f32TargetPathBuffer=tempPath*PathType_Leftward_and_Rightward_Index*PathIntegral_Index;
      break;
    }
    case PathType_F_L:{
      float tempAngle=atan2(Position->y-g_vec2_currentPos.y,Position->x-g_vec2_currentPos.x)*(180/3.14159)-45;
      *m_f32ToRotation=tempAngle<-180.0?tempAngle+360.0:tempAngle;
      m_f32TargetPathBuffer=tempPath*PathType_Angleward_Index*PathIntegral_Index;
      break;
    }
    case PathType_F_R:{
      float tempAngle=atan2(Position->y-g_vec2_currentPos.y,Position->x-g_vec2_currentPos.x)*(180/3.14159)+45;
      *m_f32ToRotation=tempAngle>180.0?tempAngle-360.0:tempAngle;
      m_f32TargetPathBuffer=tempPath*PathType_Angleward_Index*PathIntegral_Index;
      break;
    }
  }
  
}

void PathIntegral_moveLen(float lenth,uint8 PathType){
  //如果还在移动
  if(MoveLock){
    return;
  }
  //说明车已经准备开始移动了,锁存状态
  MoveLock=1;
  //绑定移动状态
  m_enumState=MOVE;
  //归零路径积分
  m_f32PathIntegral=0;
  //存储移动形式(因为不同的移动会使用不同的积分方式,所以需要保存)
  m_u8PathType=PathType;
  switch(m_u8PathType){
    case PathType_Forward:{
      m_f32TargetPath=lenth*PathIntegral_Index*PathType_Forward;
      break;
    }
    case PathType_Leftward:{
      m_f32TargetPath=lenth*PathIntegral_Index*PathType_Leftward_and_Rightward_Index;
      break;
    }
    case PathType_F_L:{
      m_f32TargetPath=lenth*PathIntegral_Index*PathType_Angleward_Index;
      break;
    }
    case PathType_F_R:{
      m_f32TargetPath=lenth*PathIntegral_Index*PathType_Angleward_Index;
      break;
    }
  }

}

void PathIntegral_rotateAngle(float angle){
  if(RotateLock){
    return;
  }
  //说明车已经准备开始转向了,锁存状态
  RotateLock=1;
  //绑定旋转状态
  m_enumState=ROTATE;
  
  *m_f32ToRotation=angle;
  
}


uint8 PathIntegral_getFlag(){
  if(PathLock&&m_enumState==MOVE){
    if(m_f32PathIntegral<m_f32TargetPathBuffer+Path_error_range&&m_f32PathIntegral>m_f32TargetPathBuffer-Path_error_range){
      if(Path_time_check(10)){
        PathLock=0;
        m_enumState = IDLE;
        return 1;
      }
    }else{
      Path_time_break();
    }
  }
  return 0;
}

uint8 PathIntegral_getMoveFlag(){
  if(MoveLock){
    if(m_f32PathIntegral<m_f32TargetPath+Path_error_range&&m_f32PathIntegral>m_f32TargetPath-Path_error_range){
      if(Path_time_check(10)){
        MoveLock=0;
        m_enumState = IDLE;
        return 1;
      }
    }else{
      Path_time_break();
    }
  }
  return 0;
}

uint8 PathIntegral_getRotateFlag(){
  //到了允许的误差角度区间 才开始下一步的移动
  float maxRange=*m_f32ToRotation+Angle_error_range;
  float minRange=*m_f32ToRotation-Angle_error_range;
/*
  float realRotation=*m_f32RealRotation;
  if(maxRange>180.0){
    if(realRotation<0){
      realRotation+=360;
    }
  }else if(minRange<-180.0){
    if(realRotation>0){
      realRotation-=360;
    }
  }
*/
  //等待车转到特定的角度
  if(*m_f32RealRotation<maxRange&&*m_f32RealRotation>minRange){
    RotateLock=0;
    m_enumState = IDLE;
    return 1;
  }
  return 0;
}


//********************************************Grid Controller*****************************************************//

#define Grid_Buffer_Size 10
#define Grid_Limit 9.5
#define Grid_check_timeout 5
float m_f32arr_Xbuffer[Grid_Buffer_Size];
uint8 m_f32arr_Xbuffer_Ptr=0;
float m_f32arr_Xbuffer_Load_Flag=0;
float m_f32Currrent_X_Target=0;
float Grid_Xbuffer_Push(float x_arg){
  uint8 i;float sum=0;
  if(!m_f32arr_Xbuffer_Load_Flag){
    for(i=0;i<Grid_Buffer_Size;i++){
      m_f32arr_Xbuffer[i]=x_arg;
    }
    m_f32arr_Xbuffer_Load_Flag=1;
  }else{
    if(m_f32arr_Xbuffer_Ptr>=Grid_Buffer_Size){
      m_f32arr_Xbuffer_Ptr=0;
    }
    m_f32arr_Xbuffer[m_f32arr_Xbuffer_Ptr++]=x_arg;
  }
  for(i=0;i<Grid_Buffer_Size;i++){
    sum+=m_f32arr_Xbuffer[i];
  }
  return sum;
}

float m_f32arr_Ybuffer[Grid_Buffer_Size];
uint8 m_f32arr_Ybuffer_Ptr=0;
float m_f32arr_Ybuffer_Load_Flag=0;
float m_f32Currrent_Y_Target=0;
float Grid_Ybuffer_Push(float y_arg){
  uint8 i;float sum=0;
  if(!m_f32arr_Ybuffer_Load_Flag){
    for(i=0;i<Grid_Buffer_Size;i++){
      m_f32arr_Ybuffer[i]=y_arg;
    }
    m_f32arr_Ybuffer_Load_Flag=1;
  }else{
    if(m_f32arr_Ybuffer_Ptr>=Grid_Buffer_Size){
      m_f32arr_Ybuffer_Ptr=0;
    }
    m_f32arr_Ybuffer[m_f32arr_Ybuffer_Ptr++]=y_arg;
  }
  for(i=0;i<Grid_Buffer_Size;i++){
    sum+=m_f32arr_Ybuffer[i];
  }
  return sum;
}

typedef enum{
  Grid_IDLE          = 0,
  Grid_CorrectCross  = 1,
  Grid_CorrectX      = 2,
  Grid_CorrectY      = 3,
  Grid_TracingX      = 4,
  Grid_TracingY      = 5,
}Grid_State; 


float m_f32_X_LineTarget;
float m_f32_Y_LineTarget;
ST_PID_POS  m_stGrid_X_Pos;
ST_PID_POS  m_stGrid_Y_Pos;
float m_f32Motor_X_Speed;
float m_f32Motor_Y_Speed;

float* m_f32DetectedTarget_X;
float* m_f32DetectedTarget_Y;
uint8* m_u8ptrTracer_X_line_cnt;
uint8* m_u8ptrTracer_Y_line_cnt;
float m_f32CurrentTarget_X;
float m_f32CurrentTarget_Y;

unsigned int Grid_Cnt=0;
uint8 Grid_Cnt_Lock=0;

uint8 m_u8Grid_X_Tracing_Enable=0;
uint8 m_u8Grid_Y_Tracing_Enable=0;

Grid_State m_enumGridState;

uint8 Grid_time_check(unsigned int checkTime){
  if(!Grid_Cnt_Lock){
    Grid_Cnt_Lock=1;
    Grid_Cnt=0;
  }
  if(Grid_Cnt>=checkTime){
    Grid_Cnt_Lock=0;
    Grid_Cnt=0;
    return 1;
  }
  return 0;
}

void Grid_time_break(){
  Grid_Cnt_Lock=0;
  Grid_Cnt=0;
}

void Grid_init(float* Tracer_X,float* Tracer_Y,uint8* Tracer_X_line_cnt,uint8* Tracer_Y_line_cnt){
  m_enumGridState=Grid_IDLE;
 
  m_u8ptrTracer_X_line_cnt=Tracer_X_line_cnt;
  m_u8ptrTracer_Y_line_cnt=Tracer_Y_line_cnt;
  m_f32DetectedTarget_X=Tracer_X;
  m_f32DetectedTarget_Y=Tracer_Y;
  
  m_stGrid_X_Pos.m_f32ptr_real=&m_f32CurrentTarget_X;
  m_stGrid_X_Pos.m_f32ptr_output=&m_f32Motor_X_Speed;
  m_stGrid_X_Pos.m_f32ptr_ideal=&m_f32_X_LineTarget;
  
  m_stGrid_Y_Pos.m_f32ptr_real=&m_f32CurrentTarget_Y;
  m_stGrid_Y_Pos.m_f32ptr_output=&m_f32Motor_Y_Speed;
  m_stGrid_Y_Pos.m_f32ptr_ideal=&m_f32_Y_LineTarget;
  
  PID_POS_init(&m_stGrid_X_Pos,20,0,25);
  PID_POS_init(&m_stGrid_Y_Pos,20,0,25);
}

void Grid_CorrectX_update(){
  PID_POS_update_PID(&m_stGrid_X_Pos);
  m_f32SpeedBuffer_FL+=m_f32Motor_X_Speed;
  m_f32SpeedBuffer_FR+=m_f32Motor_X_Speed;
  m_f32SpeedBuffer_BL+=m_f32Motor_X_Speed;
  m_f32SpeedBuffer_BR+=m_f32Motor_X_Speed;
}

void Grid_CorrectCross_update(){
  
  
  PID_POS_update_PID(&m_stGrid_X_Pos);
  PID_POS_update_PID(&m_stGrid_Y_Pos);
  int X_Direction=0;
  int Y_Direction=0;
  float Grid_Speed=0;
  
  if(fabsf(m_f32Motor_X_Speed)>10){
    X_Direction=m_f32Motor_X_Speed>10?1:-1;
    Grid_Speed=fabsf(m_f32Motor_X_Speed);
  }if(fabsf(m_f32Motor_Y_Speed)>10){
    Y_Direction=m_f32Motor_Y_Speed>10?1:-1;
    
    if(Grid_Speed<fabsf(m_f32Motor_Y_Speed)){
      Grid_Speed=fabsf(m_f32Motor_Y_Speed);
    }
    
  }
  
  
  if(!Y_Direction&&X_Direction){
    m_f32SpeedBuffer_FL+=X_Direction*Grid_Speed;
    m_f32SpeedBuffer_FR+=X_Direction*Grid_Speed;
    m_f32SpeedBuffer_BL+=X_Direction*Grid_Speed;
    m_f32SpeedBuffer_BR+=X_Direction*Grid_Speed;
  }else if(!X_Direction&&Y_Direction){
    m_f32SpeedBuffer_FL+=-Y_Direction*Grid_Speed;
    m_f32SpeedBuffer_FR+=Y_Direction*Grid_Speed;
    m_f32SpeedBuffer_BL+=Y_Direction*Grid_Speed;
    m_f32SpeedBuffer_BR+=-Y_Direction*Grid_Speed;
  }else{
    if((X_Direction>0&&Y_Direction>0)||(X_Direction<0&&Y_Direction<0)){
      m_f32SpeedBuffer_FR+=X_Direction*Grid_Speed;
      m_f32SpeedBuffer_BL+=X_Direction*Grid_Speed;
    }else if((X_Direction<0&&Y_Direction>0)||(X_Direction>0&&Y_Direction<0)){
      m_f32SpeedBuffer_FL+=X_Direction*Grid_Speed;
      m_f32SpeedBuffer_BR+=X_Direction*Grid_Speed;
    }
  }
}

void Grid_CorrectY_update(){
  PID_POS_update_PID(&m_stGrid_Y_Pos);
  m_f32SpeedBuffer_FL+=-m_f32Motor_Y_Speed;
  m_f32SpeedBuffer_FR+=m_f32Motor_Y_Speed;
  m_f32SpeedBuffer_BL+=m_f32Motor_Y_Speed;
  m_f32SpeedBuffer_BR+=-m_f32Motor_Y_Speed;
}
int m_i32Target_Y_Line=0;
int m_i32Line_Y_cnt=0;
int m_i32Line_Y_mid_flag=0;
void Grid_TracingX_update(){
  int needed_direction=(m_i32Target_Y_Line-m_i32Line_Y_cnt)>0?1:-1;
  if(*m_u8ptrTracer_X_line_cnt&&-m_f32Currrent_X_Target*needed_direction>7){
    if(!m_i32Line_Y_mid_flag){
      m_i32Line_Y_mid_flag=1;
      m_i32Line_Y_cnt+=needed_direction;
    }
  }else if(!(*m_u8ptrTracer_X_line_cnt)&&m_f32Currrent_X_Target*needed_direction>7&&m_i32Line_Y_mid_flag){
    m_i32Line_Y_mid_flag=0;
  }if(m_i32Line_Y_cnt!=m_i32Target_Y_Line){
    Grid_CorrectY_update();
    m_f32SpeedBuffer_FL+=Grid_Tracing_With_Line_Count_Speed*needed_direction;
    m_f32SpeedBuffer_FR+=Grid_Tracing_With_Line_Count_Speed*needed_direction;
    m_f32SpeedBuffer_BL+=Grid_Tracing_With_Line_Count_Speed*needed_direction;
    m_f32SpeedBuffer_BR+=Grid_Tracing_With_Line_Count_Speed*needed_direction;
  }else{
    Grid_CorrectCross_update();
  }
}

int m_i32Target_X_Line=0; 
int m_i32Line_X_cnt=0;
int m_i32Line_X_mid_flag=0;
void Grid_TracingY_update(){
  int needed_direction=(m_i32Target_X_Line-m_i32Line_X_cnt)>0?1:-1;
  if(*m_u8ptrTracer_Y_line_cnt&&-m_f32Currrent_Y_Target*needed_direction>7){
    if(!m_i32Line_X_mid_flag){
      m_i32Line_X_mid_flag=1;
      m_i32Line_X_cnt+=needed_direction;
    }
  }else if(!(*m_u8ptrTracer_Y_line_cnt)&&m_f32Currrent_Y_Target*needed_direction>7&&m_i32Line_X_mid_flag){
    m_i32Line_X_mid_flag=0;
  }if(m_i32Line_X_cnt!=m_i32Target_X_Line){
    Grid_CorrectX_update();
    m_f32SpeedBuffer_FL+=-Grid_Tracing_With_Line_Count_Speed*needed_direction;
    m_f32SpeedBuffer_FR+=Grid_Tracing_With_Line_Count_Speed*needed_direction;
    m_f32SpeedBuffer_BL+=Grid_Tracing_With_Line_Count_Speed*needed_direction;
    m_f32SpeedBuffer_BR+=-Grid_Tracing_With_Line_Count_Speed*needed_direction;
  }else{
    Grid_CorrectCross_update();
  }
}



void Grid_update(){
  if(Grid_Cnt_Lock){
    Grid_Cnt++;
  }
  
  if(*m_u8ptrTracer_X_line_cnt<=2){
    if(fabsf(*m_f32DetectedTarget_X)<0.5){                              //如果读到零
      if(fabsf(*m_f32DetectedTarget_X-m_f32Currrent_X_Target)<1.5){     //如果是连续变化的,则读入当前值,即判断它到达了循迹的中点
        m_f32Currrent_X_Target=*m_f32DetectedTarget_X;                  
      }
    }else{                                                              //非连续的变零,说明在两个循迹中间,保持原来的循迹值.
      m_f32Currrent_X_Target=*m_f32DetectedTarget_X;
    }
  }else{                                                                //如果多个点都命中了,说明在长条上,则不改变纠正.         

    m_f32Currrent_X_Target=0;
  }
  m_f32CurrentTarget_X = Grid_Xbuffer_Push(m_f32Currrent_X_Target);     //push到循环队列中进行累加,增加循迹区分度.
  if(*m_u8ptrTracer_Y_line_cnt<=2){
    if(fabsf(*m_f32DetectedTarget_Y)<0.5){
      if(fabsf(*m_f32DetectedTarget_Y-m_f32Currrent_Y_Target)<1.5){
        m_f32Currrent_Y_Target=*m_f32DetectedTarget_Y;
      }
    }else{
      m_f32Currrent_Y_Target=*m_f32DetectedTarget_Y;
    }
  }else{
    m_f32Currrent_Y_Target=0;
  }
  m_f32CurrentTarget_Y = Grid_Ybuffer_Push(m_f32Currrent_Y_Target);
  
  if(m_u8Grid_X_Tracing_Enable||m_u8Grid_Y_Tracing_Enable){            //如果启用了轴循迹.
    if(m_u8Grid_X_Tracing_Enable){                                      //X循迹启用
      Grid_CorrectX_update();
    }if(m_u8Grid_Y_Tracing_Enable){                                     //Y循迹启用
      Grid_CorrectY_update();
    }
  }else{
    switch(m_enumGridState){
    case Grid_IDLE:
      break;
    case Grid_CorrectCross:
      Grid_CorrectCross_update();
      break;
    case Grid_CorrectX:
      Grid_CorrectX_update();
      break;
    case Grid_CorrectY:
      Grid_CorrectY_update();
      break;
    case Grid_TracingX:
      Grid_TracingX_update();
      break;
    case Grid_TracingY:
      Grid_TracingY_update();
      break;
    }
  }
}

void Grid_PID_buffer_clear(){
  PID_POS_buffer_clear(&m_stGrid_X_Pos);
  PID_POS_buffer_clear(&m_stGrid_Y_Pos);
}

uint8 Grid_correctCross_Lock=0;
void Grid_correctCross(){
  if(Grid_correctCross_Lock){
    return;
  }
  Grid_correctCross_Lock=1;
  m_enumGridState=Grid_CorrectCross;
}

uint8 Grid_correctX_Lock=0;
void Grid_correctX(){
  if(Grid_correctCross_Lock){
    return;
  }
  Grid_correctX_Lock=1;
  m_enumGridState=Grid_CorrectX;
}

uint8 Grid_correctY_Lock=0;
void Grid_correctY(){
  if(Grid_correctCross_Lock){
    return;
  }
  Grid_correctY_Lock=1;
  m_enumGridState=Grid_CorrectY;
}

uint8 Grid_getCorrectCrossFlag(){
  if(fabsf(m_f32CurrentTarget_X)<Grid_Limit&&fabsf(m_f32CurrentTarget_Y)<Grid_Limit){
    if(Grid_time_check(Grid_check_timeout)){
      Grid_correctCross_Lock=0;
      m_enumGridState=Grid_IDLE;
      return 1;
    }
  }else{
    Grid_time_break();
  }
  return 0;
}

uint8 Grid_getCorrectXFlag(){
  if(fabsf(m_f32CurrentTarget_X)<Grid_Limit){
    if(Grid_time_check(Grid_check_timeout)){
      Grid_correctCross_Lock=0;
      m_enumGridState=Grid_IDLE;
      return 1;
    }
  }else{
    Grid_time_break();
  }
  return 0;
}

uint8 Grid_getCorrectYFlag(){
  if(fabsf(m_f32CurrentTarget_Y)<Grid_Limit){
    if(Grid_time_check(Grid_check_timeout)){
      Grid_correctCross_Lock=0;
      m_enumGridState=Grid_IDLE;
      return 1;
    }
  }else{
    Grid_time_break();
  }
  return 0;
}


void Grid_enable_X_tracing(){
  m_u8Grid_X_Tracing_Enable=1;
}

void Grid_disable_X_tracing(){
  m_u8Grid_X_Tracing_Enable=0;
}

void Grid_enable_Y_tracing(){
  m_u8Grid_Y_Tracing_Enable=1;
}

void Grid_disable_Y_tracing(){
  m_u8Grid_Y_Tracing_Enable=0;
}

uint8 Grid_X_tracing_Lock=0;
void Grid_X_tracing(int Line_Y_cnt){
  if(Grid_X_tracing_Lock){
    return;
  }
  m_i32Line_Y_mid_flag=0;
  m_i32Target_Y_Line=Line_Y_cnt;
  Grid_X_tracing_Lock=1;
  m_i32Line_Y_cnt=0;
  m_enumGridState=Grid_TracingX;
  

}

uint8 Grid_Y_tracing_Lock=0;
void Grid_Y_tracing(int Line_X_cnt){
  if(Grid_Y_tracing_Lock){
    return;
  }
  m_i32Line_X_mid_flag=0;
  m_i32Target_X_Line=Line_X_cnt;
  Grid_Y_tracing_Lock=1;
  m_i32Line_X_cnt=0;
  m_enumGridState=Grid_TracingY;
}

uint8 Grid_getX_tracingFlag(){
  if(fabsf(m_f32CurrentTarget_X)<Grid_Limit&&fabsf(m_f32CurrentTarget_Y)<Grid_Limit&&(m_i32Target_Y_Line==m_i32Line_Y_cnt)){
    if(Grid_time_check(Grid_check_timeout)){
      Grid_X_tracing_Lock=0;
      m_enumGridState=Grid_IDLE;
      return 1;
    }
  }else{
    Grid_time_break();
  }
  return 0;
}

uint8 Grid_getY_tracingFlag(){
  if(fabsf(m_f32CurrentTarget_X)<Grid_Limit&&fabsf(m_f32CurrentTarget_Y)<Grid_Limit&&(m_i32Target_X_Line==m_i32Line_X_cnt)){
    if(Grid_time_check(Grid_check_timeout)){
      Grid_Y_tracing_Lock=0;
      m_enumGridState=Grid_IDLE;
      return 1;
    }
  }else{
    Grid_time_break();
  }
  return 0;
}
