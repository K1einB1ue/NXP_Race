#include"./HWT101.h"

lpuart_handle_t HWT101_Handle;

#define Buffer_Size 50

uint8 HWT101_buffer[Buffer_Size];
uint8 HWT101_chr_buffer;
uint8 HWT101_num=0;

float g_f32HWT101_Direction;

#ifdef HWT101_ResetPin
void HWT101_Z_reset(){
  gpio_set(HWT101_ResetPin,1);
  systick_delay_ms(3000);
  gpio_set(HWT101_ResetPin,0);
  systick_delay_ms(1000);
}
#endif

void HWT101_update(){
  uint8 num=HWT101_num;
  int i;
  float target=0;
  uint8 flag=0;
  for(i=num;i>=0;i--){
    if(HWT101_buffer[i-10<0?Buffer_Size+i-10:i-10]==0x53&&HWT101_buffer[i-11<0?Buffer_Size+i-11:i-11]==0x55){
      target=((float)(HWT101_buffer[i-4<0?Buffer_Size+i-4:i-4]<<8|HWT101_buffer[i-5<0?Buffer_Size+i-5:i-5]))/32768*180;
      flag=1;
      break;
    }
  }if(!flag){
    for(i=Buffer_Size;i>num;i--){
      if(HWT101_buffer[i-10<0?Buffer_Size+i-10:i-10]==0x53&&HWT101_buffer[i-11<0?Buffer_Size+i-11:i-11]==0x55){
        target=((float)(HWT101_buffer[i-4<0?Buffer_Size+i-4:i-4]<<8|HWT101_buffer[i-5<0?Buffer_Size+i-5:i-5]))/32768*180;
        break;
      }
    }
  }
  g_f32HWT101_Direction=target>180?target-360.0:target;
}

void HWT101_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){
  if(status == kStatus_LPUART_RxIdle){
    if(HWT101_num>=Buffer_Size){
      //如果发生了错码.则舍弃一次结果.
      HWT101_num=0;
    }
    
    HWT101_buffer[HWT101_num++]=HWT101_chr_buffer;
    
    //UART会指针自增,所以要复位指针.(并不确定 但是STM32是这么设计的)
    handle->rxDataSize=1;
    handle->rxData=&HWT101_chr_buffer;
  }
}

float getLoopAngle(float angle,float interestAngle){
/*
  if(interestAngle<-90.0&&angle>90.0){
    return angle-360.0;
  }else if(interestAngle>90.0&&angle<-90.0){
    return angle+360.0;
  }
  return angle;
*/
  float PosAngle=angle+360.0;
  float MidAngle=angle;
  float NegAngle=angle-360.0;
  
  float PosDistance = fabsf(PosAngle-interestAngle);//180+90 270 270
  float MidDistance = fabsf(MidAngle-interestAngle);
  float NegDistance = fabsf(NegAngle-interestAngle);//-180+90 -90 90
  
  float min=PosDistance;
  uint8 num=0;
  if(min>MidDistance){
    min=MidDistance;
    num=1;
  }if(min>NegDistance){
    min=NegDistance;
    num=2;
  }
  
  switch(num){
  case 0:return PosAngle;
  case 1:return MidAngle;
  case 2:return NegAngle;
  }
  return 0;
}

float getAngle(float interestAngle){
/*
  if(interestAngle<-90.0&&g_f32HWT101_Direction>90.0){
    return g_f32HWT101_Direction-360.0;
  }else if(interestAngle>90.0&&g_f32HWT101_Direction<-90.0){
    return g_f32HWT101_Direction+360.0;
  }
  return g_f32HWT101_Direction;     
*/
  

  float PosAngle=g_f32HWT101_Direction+360.0;
  float MidAngle=g_f32HWT101_Direction;
  float NegAngle=g_f32HWT101_Direction-360.0;
  
  float PosDistance = fabsf(PosAngle-interestAngle);//180+90 270 270
  float MidDistance = fabsf(MidAngle-interestAngle);
  float NegDistance = fabsf(NegAngle-interestAngle);//-180+90 -90 90
  
  float min=PosDistance;
  uint8 num=0;
  if(min>MidDistance){
    min=MidDistance;
    num=1;
  }if(min>NegDistance){
    min=NegDistance;
    num=2;
  }
  
  switch(num){
  case 0:return PosAngle;
  case 1:return MidAngle;
  case 2:return NegAngle;
  }
  return 0;
  
}

void HWT101_init(){
  
  uart_init(HWT101_UART,HWT101_Baud,HWT101_UART_TX,HWT101_UART_RX);
  
  NVIC_SetPriority(HWT101_UART_IQRn,HWT101_Priority);
  uart_rx_irq(HWT101_UART,1);
  //设置中断回调,设置RX缓存,使其Size为1;
  uart_set_handle(HWT101_UART,&HWT101_Handle,HWT101_Callback,NULL,0,&HWT101_chr_buffer,1);
  
#ifdef HWT101_ResetPin
  gpio_init(HWT101_ResetPin,GPO,0,GPIO_PIN_CONFIG);
#endif
}