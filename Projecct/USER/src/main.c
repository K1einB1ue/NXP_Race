//装配车

#include"./Application/Application.h"
//硬件引脚初始化 参见 HardWare/Config.h
#include<stdio.h>
int32 stateNum;
int32 keyNum;

SliceManager Manager;
SliceManager Manager0;
SliceManager Manager1;
SliceUpdater Updater_Motor_PID;
SliceUpdater Updater_All_PID;
uint8 __MOTION__=1;
uint8 MapEnable=0;

/****一些基础算法补充*****/

/****SliceManager补充*****/
typedef struct{
  float Runlen;
  uint8 PathType;
}RunInfo;

void ToMove(void* arg){
  RunInfo* ptr = (RunInfo*)arg;
  PathIntegral_moveLen(ptr->Runlen,ptr->PathType);
}

void ToPosition(void* arg){
  Vector2* ptr = (Vector2*)arg;
  PathIntegral_move(ptr,PathType_Forward);
}

void ToRotate(void* arg){
  float* ptr = (float*)arg;
  PathIntegral_rotateAngle(*ptr);
}

void ToXLine(void* arg){
  int* ptr = (int*)arg;
  Grid_X_tracing(*ptr);
}

void ToYLine(void* arg){
  int* ptr = (int*)arg;
  Grid_Y_tracing(*ptr);
}
/****回调函数补充****/
#define PID_Error -1
#define PID_Motor_FL 1
#define PID_Motor_FR 2
#define PID_Motor_BL 3
#define PID_Motor_BR 4
#define PID_Rotation 5
#define PID_Path 6
#define PID_Grid 7

int PID_select = PID_Error;
char PID_sprintf_buffer[60];

uint8 action_flag=1;
//蓝牙接受            [info]
void BlueTooth_decode(const char* str,uint8 size){
  char* force = (char*)str;
  force[size]='\0';
  if(m_strcmp(str,"Select",6)){
    sscanf(str,"Select%d",&PID_select);
    sprintf(PID_sprintf_buffer,"PID select %d\n",PID_select);
    BlueTooth_sendStr(PID_sprintf_buffer);
  }else if(m_strcmp(str,"PID",3)){
    float P=0,I=0,D=0;
    uint8 i,cnt=0;
    for(i=0;i<size;i++){
      if(str[i]==','){
        cnt++;
      }
    }
    if(cnt==2){
      sscanf(str,"PID%f,%f,%f",&P,&I,&D);
      switch(PID_select){
        case PID_Motor_FL:
          PID_POS_init(&m_stMotor_FL,P,I,D);
        break;
        case PID_Motor_FR:
          PID_POS_init(&m_stMotor_FR,P,I,D);
        break;
        case PID_Motor_BL:
          PID_POS_init(&m_stMotor_BL,P,I,D);
        break;
        case PID_Motor_BR:
          PID_POS_init(&m_stMotor_BR,P,I,D);
        break;
        case PID_Rotation:
          PID_POS_init(&m_stMotor_Direction,P,I,D);
        break;
        case PID_Path:
          PID_POS_init(&m_stMotor_Pos,P,I,D);
        break;
        case PID_Grid:
          PID_POS_init(&m_stGrid_X_Pos,P,I,D);
          PID_POS_init(&m_stGrid_Y_Pos,P,I,D);
        break;
        default:
        case PID_Error:
          BlueTooth_send("no PID selected!\n",17);
        break;
      }
    }else{
      ST_PID_POS* ptr = 0;
      char* str = 0;
      switch(PID_select){
        case PID_Motor_FL:
          str="Motor1";
          ptr=&m_stMotor_FL;
        break;
        case PID_Motor_FR:
          str="Motor2";
          ptr=&m_stMotor_FR;
        break;
        case PID_Motor_BL:
          str="Motor3";
          ptr=&m_stMotor_BL;
        break;
        case PID_Motor_BR:
          str="Motor4";
          ptr=&m_stMotor_BR;
        break;
        case PID_Rotation:
          str="Rotation";
          ptr=&m_stMotor_Direction;
        break;
        case PID_Path:
          str="Path";
          ptr=&m_stMotor_Pos;
        break;
        case PID_Grid:
          str="Grid";
          ptr=&m_stGrid_X_Pos;
        break;
        default:
        case PID_Error:
          BlueTooth_sendStr("no PID selected!\n");
        return;
      }
      sprintf(PID_sprintf_buffer,"%s P:%f,I:%f,D:%f\n",str,ptr->m_f32Kp,ptr->m_f32Ki,ptr->m_f32Kd);
      BlueTooth_sendStr(PID_sprintf_buffer);
    }
  }else if(m_strcmp(str,"goto",4)){
    if(!MapEnable){
      MapEnable=1;
      Vector2 pos;
      PathIntegral_reset();
      sscanf(str,"goto%f,%f",&pos.x,&pos.y);
      ToPosition(&pos);
    }
  }else if(str[0]=='s'){
    float speed = 0;
    int num = 0;
    sscanf(str,"s%d,%f",&num,&speed);
    switch(num){
      case 1:*(m_stMotor_FL.m_f32ptr_ideal)=speed;break;
      case 2:*(m_stMotor_FR.m_f32ptr_ideal)=speed;break;
      case 3:*(m_stMotor_BL.m_f32ptr_ideal)=speed;break;
      case 4:*(m_stMotor_BR.m_f32ptr_ideal)=speed;break;
    }
  }else if(str[0]=='S'){
    if(__MOTION__){
      __MOTION__=0;
      Speed_clear();
      BlueTooth_sendStr("Stop!\n");
    }else{
      __MOTION__=1;
      BlueTooth_sendStr("Run!\n");
    }
  }else if(str[0]=='R'){
    float Rotation=0;
    sscanf(str,"R%f",&Rotation);
    g_f32Motor_Rotation=Rotation;
  }else if(str[0]=='G'){
    if(action_flag){
      SliceManager_prepare(&Manager0);
    }else{
      SliceManager_prepare(&Manager1);
    }
    action_flag=action_flag?0:1;
  }else if(str[0]=='F'){
    SliceManager_prepare(&Manager);
  }else if(str[0]=='Z'){
    HWT101_Z_reset();
    Motor_PID_buffer_clear();
    PathIntegral_PID_buffer_clear();
    Grid_PID_buffer_clear();
    BlueTooth_sendStr("Z clear!\n");
  }
}

//摄像头串口线接受    #info!
void OpenArt_decode(const char* str, uint8 size){

}



void AdditionInit(void){
  //SYN6288_init();
  //Tracer_init();
  BlueTooth_init();
  HWT101_init();
  Motor_init();
  Rotation_PID_init(getAngle);
  Speed_Buffer_Bind(&g_f32Motor_Speed_Index_FL,&g_f32Motor_Speed_Index_FR,&g_f32Motor_Speed_Index_BL,&g_f32Motor_Speed_Index_BR);
  PathIntegral_init(&g_f32Motor_RealAngle,&g_f32Motor_Rotation,m_stMotor_FL.m_f32ptr_real,m_stMotor_FR.m_f32ptr_real,m_stMotor_BL.m_f32ptr_real,m_stMotor_BR.m_f32ptr_real);
  //Grid_init(&g_f32TracerArg_X,&g_f32TracerArg_Y,&g_u8Tracer_X_line_cnt,&g_u8Tracer_Y_line_cnt);
  gpio_init(OpenKey,GPO,1,GPIO_PIN_CONFIG);
  gpio_interrupt_init(OpenKey,RISING,GPIO_INT_CONFIG);
  NVIC_SetPriority(KeyIrq,15);
}
 

           

#define OneBlockLen 800000/12

//以车头正前方为X正轴,以车头正左侧为Y正轴.

 
int main(void){
  //硬件初始化
  InitCallback=AdditionInit;
  Init();
  systick_delay_ms(1000);
  
  
  
  float RotationArr[4];
  RotationArr[0]=0.0;
  RotationArr[1]=-90;
  RotationArr[2]=-180;
  RotationArr[3]=90;
  RunInfo runInfoArr[1];
  runInfoArr[0].PathType=PathType_Forward;
  runInfoArr[0].Runlen=OneBlockLen;
  
  
  SliceManager_init(&Manager);
  SliceManager_push(&Manager,ToRotate,RotationArr+0,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+1,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+2,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+3,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+0,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+1,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+2,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+3,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  SliceManager_push(&Manager,ToRotate,RotationArr+0,PathIntegral_getRotateFlag);
  //SliceManager_push(&Manager,ToYLine,LineArr+0,Grid_getY_tracingFlag);  //到达装配点
  Manager.is_finish=1;
  
  SliceManager_init(&Manager0);
  SliceManager_push(&Manager0,ToRotate,RotationArr+0,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager0,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  Manager0.is_finish=1;
  SliceManager_init(&Manager1);
  SliceManager_push(&Manager1,ToRotate,RotationArr+2,PathIntegral_getRotateFlag);
  SliceManager_push(&Manager1,ToMove,runInfoArr+0,PathIntegral_getMoveFlag);
  Manager1.is_finish=1;
  while(keyNum<2);
  

  while(1);
}






