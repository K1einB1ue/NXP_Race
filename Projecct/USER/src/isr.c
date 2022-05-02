#include "isr.h"

uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f};

int position_buffer[60][2];
int position_size=0;

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     //调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
    __DSB();                    //数据同步隔离
}

void Race_decode_json(cJSON* root){
  if(root!=NULL){
      cJSON *Origin_json = cJSON_GetObjectItem(root, "Server");
      if(strcmp(Origin_json->valuestring,"Server")){
        cJSON *Coordinate_json = cJSON_GetObjectItem(root, "Coordinate");
        if (cJSON_IsArray(Coordinate_json)){
          int size = cJSON_GetArraySize(Coordinate_json);
          position_size=size;
          for(int i=0;i<size;i++){
            cJSON *Position_json=cJSON_GetArrayItem(Coordinate_json,i);
            cJSON* Position_x_json=cJSON_GetObjectItem(Position_json,"x");
            cJSON* Position_y_json=cJSON_GetObjectItem(Position_json,"y");
            int num_x = Position_x_json->valueint;
            int num_y = Position_y_json->valueint;
            position_buffer[position_size][0]=num_x;
            position_buffer[position_size][1]=num_y;
            position_size++;
          }
        }
      }
    }

}


//类似定时器中断的一个玩意 时间设置请看 Hardware.c
void PIT_IRQHandler(void)
{
  if(PIT_FLAG_GET(PIT_CH0))
  {
    PIT_FLAG_CLEAR(PIT_CH0);
    HWT101_update();
    //Tracer_update();
    BlueTooth_update();
    Race_decode_json(ESP8266_update());
    
    
    /*编码器方向测定*/
    /*
      BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_FR.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_BL.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_BR.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)tail,4);
    */
    
    /*方向测定*/
    /*
      Motor_update();
      BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_FR.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_BL.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_BR.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)tail,4);
    */
    
    /*速度环PID整定*/
    /*
    if(keyNum>1){
      BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_output,sizeof(float));
      BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_ideal,sizeof(float));
      BlueTooth_send((char*)tail,4);
    }
    Motor_update();
    */
    
    /*速度同步率整定*/
    /*
    if(keyNum>1){
      *m_stMotor_FL.m_f32ptr_ideal=10*keyNum;
      *m_stMotor_FR.m_f32ptr_ideal=10*keyNum;
      *m_stMotor_BL.m_f32ptr_ideal=10*keyNum;
      *m_stMotor_BR.m_f32ptr_ideal=10*keyNum;
      BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_FR.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_BL.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)m_stMotor_BR.m_f32ptr_real,sizeof(float));
      BlueTooth_send((char*)tail,4);
    }
    Motor_update();
    */
    
    /*角度环PID整定*/
    /*
    if(keyNum>1){
      BlueTooth_send((char*)&g_f32HWT101_Direction,sizeof(float));
      BlueTooth_send((char*)&g_f32Motor_Rotation,sizeof(float));
      BlueTooth_send((char*)tail,4);
    }if(keyNum>1){
      Rotation_update();
    }
    Motor_update();
    */
    
    /*串口检测*/
    
      

    /*
    PathIntegral_update();
    Rotation_update();
    Motor_update();
    
    BlueTooth_send((char*)&g_f32Motor_RealAngle,sizeof(float));
    BlueTooth_send((char*)&g_f32Motor_Rotation,sizeof(float));
    BlueTooth_send((char*)tail,4);
    */
      
    if(__MOTION__){
      Speed_Buffer_Clear();
        PathIntegral_update();
      Speed_Buffer_Load();
      Rotation_update(); 
      Motor_update();
      if(!Manager.is_finish){
        SliceManager_update(&Manager);
      }else if(!Manager0.is_finish){
        SliceManager_update(&Manager0);
      }else if(!Manager1.is_finish){
        SliceManager_update(&Manager1);
      }
    }
    
    static uint16 num=0;
    num++;
    
    /*
    switch(num){
      case 1:BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_ideal,sizeof(float));          break;
      case 2:BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_output,sizeof(float));         break;
      case 3:BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_real,sizeof(float));           break;
      case 4:BlueTooth_send((char*)m_stMotor_Pos.m_f32ptr_ideal,sizeof(float));         break;
      case 5:BlueTooth_send((char*)m_stMotor_Pos.m_f32ptr_output,sizeof(float));        break;
      case 6:BlueTooth_send((char*)m_stMotor_Pos.m_f32ptr_real,sizeof(float));          break;
      case 7:BlueTooth_send((char*)m_stMotor_Direction.m_f32ptr_ideal,sizeof(float));   break;
      case 8:BlueTooth_send((char*)m_stMotor_Direction.m_f32ptr_output,sizeof(float));  break;
      case 9:BlueTooth_send((char*)m_stMotor_Direction.m_f32ptr_real,sizeof(float));    break;
      default:BlueTooth_send((char*)tail,4);num=0;
    }
    */
    
    switch(num){
      case 1:BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_ideal,sizeof(float));          break;
      case 2:BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_real,sizeof(float));           break;
      case 3:BlueTooth_send((char*)m_stMotor_FR.m_f32ptr_ideal,sizeof(float));          break;
      case 4:BlueTooth_send((char*)m_stMotor_FR.m_f32ptr_real,sizeof(float));           break;
      case 5:BlueTooth_send((char*)m_stMotor_BL.m_f32ptr_ideal,sizeof(float));          break;
      case 6:BlueTooth_send((char*)m_stMotor_BL.m_f32ptr_real,sizeof(float));           break;
      case 7:BlueTooth_send((char*)m_stMotor_BR.m_f32ptr_ideal,sizeof(float));          break;
      case 8:BlueTooth_send((char*)m_stMotor_BR.m_f32ptr_real,sizeof(float));           break;
      case 9:BlueTooth_send((char*)&g_f32Motor_Rotation,sizeof(float));                break;
      case 10:BlueTooth_send((char*)&g_f32Motor_RealAngle,sizeof(float));                break;
      default:BlueTooth_send((char*)tail,4);num=0;
    }
    
    
    switch(MapEnable){
      case 1:if(PathIntegral_getFlag()) {MapEnable=0;}break;
      default:break;
    }
    /*
    switch(num){
      case 1:BlueTooth_send((char*)m_stMotor_FL.m_f32ptr_real,sizeof(float));        break;
      case 2:BlueTooth_send((char*)m_stMotor_FR.m_f32ptr_real,sizeof(float));        break;
      case 3:BlueTooth_send((char*)m_stMotor_BL.m_f32ptr_real,sizeof(float));        break;
      case 4:BlueTooth_send((char*)m_stMotor_BR.m_f32ptr_real,sizeof(float));        break;
      default:BlueTooth_send((char*)tail,4);num=0;
    }
    */
    
    
    
    
    
    
    
    //BlueTooth_send((char*)&g_f32Motor_RealAngle,sizeof(float));
    //BlueTooth_send((char*)&g_f32Motor_Rotation,sizeof(float));
    
    
  }
  
  if(PIT_FLAG_GET(PIT_CH1))
  {
    PIT_FLAG_CLEAR(PIT_CH1);
  }
  
  if(PIT_FLAG_GET(PIT_CH2))
  {
    
    PIT_FLAG_CLEAR(PIT_CH2);
  }
  
  if(PIT_FLAG_GET(PIT_CH3))
  {
    PIT_FLAG_CLEAR(PIT_CH3);
  }

  __DSB();
}


void KeyIrqHandler(void){
  if(GET_GPIO_FLAG(OpenKey)){
    keyNum++;
  }
  CLEAR_GPIO_FLAG(OpenKey);
}


/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器中断
void PIT_IRQHandler(void)
{
    //务必清除标志位
    __DSB();
}
记得进入中断后清除标志位
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/



