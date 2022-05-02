#include"./SYN6288.h"

uint8 SYN6288_chr_buffer;
lpuart_handle_t SYN6288_Handle;


void SYN6288_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){
  if(status == kStatus_LPUART_RxIdle){
    //UART会指针自增,所以要复位指针.(并不确定 但是STM32是这么设计的)
    handle->rxDataSize=1;
    handle->rxData=&SYN6288_chr_buffer;
  }
}

void SYN6288_init(){
  uart_init(SYN6288_UART,SYN6288_Baud,SYN6288_UART_TX,SYN6288_UART_RX);
  
  NVIC_SetPriority(SYN6288_UART_IQRn,SYN6288_Priority);
  uart_rx_irq(SYN6288_UART,1);
  //设置中断回调,设置RX缓存,使其Size为1;
  uart_set_handle(SYN6288_UART,&SYN6288_Handle,SYN6288_Callback,NULL,0,&SYN6288_chr_buffer,1);

}
char UnicodeBuffer[200];
uint16 UnicodeSize=0;
char SYNDataFrameBuffer[200];
uint16 SYNDataFrameSize=0;


static void convertUTF8ToUnicode(char* str,uint16 size){
  uint16 i;
  UnicodeSize=0;
  for(i=0;i<size/3;i++){
    UnicodeBuffer[UnicodeSize++]=((str[i*3]&0xF)<<4)+((str[i*3+1]>>2)&0xF);
    UnicodeBuffer[UnicodeSize++]=((str[i*3+1]&0x3)<<6)+(str[i*3+2]&0x3F);
  }
}

static void BuildFrame(char* str,uint16 size){
  uint16 i;uint8 XORCheck=0;
  SYNDataFrameSize=0;
  SYNDataFrameBuffer[SYNDataFrameSize++]=(0xFD);
  SYNDataFrameBuffer[SYNDataFrameSize++]=(0x00);
  SYNDataFrameBuffer[SYNDataFrameSize++]=(size+3);
  SYNDataFrameBuffer[SYNDataFrameSize++]=(0x01);
  uint8 Music=0x00;
  SYNDataFrameBuffer[SYNDataFrameSize++]=(0x01|(Music<<3));
  for(i=0;i<5;i++){
    XORCheck = XORCheck^(SYNDataFrameBuffer[i]);
  }
  for(i=0;i<size;i++){
    SYNDataFrameBuffer[SYNDataFrameSize++]=(str[i]);
    XORCheck=XORCheck^str[i];
  }
  SYNDataFrameBuffer[SYNDataFrameSize++]=(XORCheck);
}

void SYN6288_send(char* str,uint16 size){
  uint16 cnt;
  //convertUTF8ToUnicode(str,size);
  BuildFrame(str,size);
  for(cnt=0;cnt<SYNDataFrameSize;cnt++){
    uart_putchar(SYN6288_UART,*(SYNDataFrameBuffer+cnt));
  }
}