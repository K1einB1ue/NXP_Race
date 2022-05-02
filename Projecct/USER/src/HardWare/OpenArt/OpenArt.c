#include"./OpenArt.h"

#define Buffer_Size 30

lpuart_handle_t OpenArt_Handle;
uint8 OpenArt_buffer[Buffer_Size];
uint8 OpenArt_chr_buffer;
uint8 OpenArt_num=0;
uint8 OpenArt_recv_enable=0;
uint8 OpenArt_recv_really=0;

__attribute__((weak)) void OpenArt_decode(const char* str,uint8 size){}


void OpenArt_update(){
  if(OpenArt_recv_really){
    OpenArt_decode((const char*)OpenArt_buffer,OpenArt_num);
    OpenArt_recv_really=0;
    OpenArt_num=0;
  }
}

void OpenArt_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){
  if(status == kStatus_LPUART_RxIdle){
    if(OpenArt_num>=Buffer_Size){
      OpenArt_num=0;
    }
    if(OpenArt_chr_buffer==']'){
      OpenArt_recv_enable=0;
      OpenArt_recv_really=1;
    }
    if(OpenArt_recv_enable&&!OpenArt_recv_really){
      OpenArt_buffer[OpenArt_num++]=OpenArt_chr_buffer;
    }
    if(OpenArt_chr_buffer=='['){
      OpenArt_recv_enable=1;
    }
    handle->rxDataSize=1;
    handle->rxData=&OpenArt_chr_buffer;
  }
}

void OpenArt_init(){
  uart_init(OPENART_UART,OPENART_Baud,OPENART_UART_TX,OPENART_UART_RX);
  
  NVIC_SetPriority(OPENART_UART_IQRn,OPENART_Priority);
  uart_rx_irq(OPENART_UART,1);
  
  uart_set_handle(OPENART_UART,&OpenArt_Handle,OpenArt_Callback,NULL,0,&OpenArt_chr_buffer,1);

}

void OpenArt_send(char* str,uint16 size){
  uint16 cnt;
  for(cnt=0;cnt<size;cnt++){
    uart_putchar(OPENART_UART,*(str+cnt));
  }
}