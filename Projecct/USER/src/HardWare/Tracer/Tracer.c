#include"Tracer.h"

float g_f32TracerArg_X;
float g_f32TracerArg_Y;
uint8 g_u8Tracer_X_line_cnt;
uint8 g_u8Tracer_Y_line_cnt;

#define Buffer_Size 30

lpuart_handle_t TracerX_Handle;
uint8 TracerX_buffer[Buffer_Size];
uint8 TracerX_chr_buffer;
uint8 TracerX_num=0;

lpuart_handle_t TracerY_Handle;
uint8 TracerY_buffer[Buffer_Size];
uint8 TracerY_chr_buffer;
uint8 TracerY_num=0;

uint8 bit4Reverse(uint8 input){
  uint8 temp=0,i;
  for(i=0;i<4;i++){
    if(input&(0x01<<i)){
      temp|=(0x01<<(3-i));
    }
  }
  return temp;
}


uint8 selectBiggestBit(uint8 input,uint8* pos,uint8* neg){
  uint8 i,line_cnt=0;
  for(i=0;i<4;i++){
    if(input&(0x01<<i)){
      if(!(*pos)){
        *pos=0x01<<(3-i);
      }
      line_cnt++;
    }
  }
  for(i=7;i>3;i--){
    if(input&(0x01<<i)){
      if(!(*neg)){
        *neg=0x01<<(i-4);
      }
      line_cnt++;
    }
  }
  return line_cnt;
}

void TracerX_update(){
  uint8 num=TracerX_num;
  int i;
  uint8 target=0,flag=0;
  for(i=num;i>=0;i--){
    if(TracerX_buffer[i]==0x02&&TracerX_buffer[i-2<0?Buffer_Size+i-2:i-2]==0x75){
      target=TracerX_buffer[i-1<0?Buffer_Size+i-1:i-1];
      flag=1;
      break;
    }
  }if(!flag){
    for(i=Buffer_Size;i>num;i--){
      if(TracerX_buffer[i]==0x02&&TracerX_buffer[i-2<0?Buffer_Size+i-2:i-2]==0x75){
        target=TracerX_buffer[i-1<0?Buffer_Size+i-1:i-1];
        break;
      }
    }
  }
  uint8 pos_part=0,neg_part=0;
  g_u8Tracer_X_line_cnt=selectBiggestBit(target,&pos_part,&neg_part);
#if Tracer_Test
      g_f32TracerArg_X=pos_part-neg_part;
#else
  #if Tracer_Y_Reverse
      g_f32TracerArg_X=pos_part-neg_part;
  #else
      g_f32TracerArg_X=neg_part-pos_part;
  #endif
#endif
}

void TracerX_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){
  if(kStatus_LPUART_RxIdle == status){
    if(TracerX_num>=Buffer_Size){
      TracerX_num=0;
    }
    TracerX_buffer[TracerX_num++]=TracerX_chr_buffer;
  }
  
  handle->rxDataSize = 1;  //还原缓冲区长度
  handle->rxData = &TracerX_chr_buffer;          //还原缓冲区地址  
}

void TracerY_update(){
  uint8 num=TracerY_num;
  int i;
  uint8 target=0,flag=0;
  for(i=num;i>=0;i--){
    if(TracerY_buffer[i]==0x02&&TracerY_buffer[i-2<0?Buffer_Size+i-2:i-2]==0x75){
      target=TracerY_buffer[i-1<0?Buffer_Size+i-1:i-1];
      flag=1;
      break;
    }
  }if(!flag){
    for(i=Buffer_Size;i>num;i--){
      if(TracerY_buffer[i]==0x02&&TracerY_buffer[i-2<0?Buffer_Size+i-2:i-2]==0x75){
        target=TracerY_buffer[i-1<0?Buffer_Size+i-1:i-1];
        break;
      }
    }
  }
  uint8 pos_part=0,neg_part=0;
  g_u8Tracer_Y_line_cnt=selectBiggestBit(target,&pos_part,&neg_part);
#if Tracer_Test
      g_f32TracerArg_Y=pos_part-neg_part;
#else
  #if Tracer_Y_Reverse
      g_f32TracerArg_Y=pos_part-neg_part;
  #else
      g_f32TracerArg_Y=neg_part-pos_part;
  #endif
#endif
}

void TracerY_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){
  if(kStatus_LPUART_RxIdle == status){
    if(TracerY_num>=Buffer_Size){
      TracerY_num=0;
    }
    TracerY_buffer[TracerY_num++]=TracerY_chr_buffer;
  }
  
  handle->rxDataSize = 1;  //还原缓冲区长度
  handle->rxData = &TracerY_chr_buffer;          //还原缓冲区地址  
}


void Tracer_init(){
  uart_init(TracerX_UART,TracerX_Baud,TracerX_UART_TX,TracerX_UART_RX);
  uart_init(TracerY_UART,TracerY_Baud,TracerY_UART_TX,TracerY_UART_RX);
  
    NVIC_SetPriority(TracerX_UART_IQRn,TracerX_Priority);         //设置串口中断优先级 范围0-15 越小优先级越高    
    NVIC_SetPriority(TracerY_UART_IQRn,TracerY_Priority);         //设置串口中断优先级 范围0-15 越小优先级越高
        
    uart_rx_irq(TracerX_UART,1);
    uart_rx_irq(TracerY_UART,1);
    
    //设置中断函数及其参数
    uart_set_handle(TracerX_UART, &TracerX_Handle, TracerX_Callback, NULL, 0, &TracerX_chr_buffer, 1);
    uart_set_handle(TracerY_UART, &TracerY_Handle, TracerY_Callback, NULL, 0, &TracerY_chr_buffer, 1);
}

void Tracer_update(){
  uart_putchar(TracerX_UART,0x57);
  uart_putchar(TracerY_UART,0x57);
  TracerX_update();
  TracerY_update();
}