#include"./ESP8266.h"

lpuart_handle_t ESP8266_Handle;

#define Buffer_Size 200

uint8 ESP8266_buffer[Buffer_Size];
uint8 ESP8266_chr_buffer;
uint8 ESP8266_num=0;

uint8 json_flag_num=0;
uint8 json_start_flag=0;

uint8 ESP8266_recv_enable=0;
uint8 ESP8266_recv_really=0;

uint8 Config_AT_mode=1;

#define NONE 0
#define OK 1
#define ERROR 2

uint8 ESP8266_State=NONE;

#define Disconnect 0
#define Got_IP 1
#define Connected 2

uint8 ESP8266_Wifi_State=Disconnect;

#ifdef ESP8266_ResetPin
void ESP8266_hardware_reset(){
  gpio_set(ESP8266_ResetPin,1);
  systick_delay_ms(3000);
  gpio_set(ESP8266_ResetPin,0);
  systick_delay_ms(1000);
}
#endif



cJSON* ESP8266_update(){
  cJSON *root = NULL;
  if(ESP8266_recv_really){
    cJSON *root = cJSON_Parse((const char*)ESP8266_buffer);
  }
  return root;
}

void ESP8266_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){
  if(status == kStatus_LPUART_RxIdle){
    if(ESP8266_num>=Buffer_Size){
      //如果发生了错码.则舍弃一次结果.
      ESP8266_num=0;
    }
    /*
    if(Config_AT_mode){
      ESP8266_buffer[ESP8266_num++]=ESP8266_chr_buffer;
      if(m_strcmp(ESP8266_buffer+ESP8266_num-1,"\r\n",2)){
        if(m_strcmp(ESP8266_buffer,"OK",2)){
          ESP8266_State=OK;
          ESP8266_num=0;
          return;
        }
        if(m_strcmp(ESP8266_buffer,"ERROR",5)){
          ESP8266_State=ERROR;
          ESP8266_num=0;
          return;
        }
        if(m_strcmp(ESP8266_buffer,"WIFI",4)){
          if(m_strcmp(ESP8266_buffer+5),"GOT IP",6){
            ESP8266_Wifi_State=Got_IP;
            ESP8266_num=0;
            return; 
          }
          if(m_strcmp(ESP8266_buffer+5),"CONNECTED",9){
            ESP8266_Wifi_State=Connected;
            ESP8266_num=0;
            return; 
          }
          if(m_strcmp(ESP8266_buffer+5),"DISCONNECT",10){
            ESP8266_Wifi_State=Disconnect;
            ESP8266_num=0;
            return; 
          }
        }
        if(ESP8266_buffer[0]=='+'){
          if(str::strcmp(ESP8266_buffer+1,"IPD",3)){
          
          }
        }
      }
    }else{
      if(ESP8266_chr_buffer=='{'){
        json_flag_num++;
        json_start_flag=1;
      }
      if(!ESP8266_recv_really){
        ESP8266_buffer[ESP8266_num++]=ESP8266_chr_buffer;
      }
      if(ESP8266_chr_buffer=='}'){
        json_flag_num--;
        if(json_start_flag&&!json_flag_num){
          json_start_flag=0;
          ESP8266_recv_really=1;
          ESP8266_buffer[ESP8266_num]='\0';
          ESP8266_num=0;
        }
      }
    }
    */
    //UART会指针自增,所以要复位指针.(并不确定 但是STM32是这么设计的)
    handle->rxDataSize=1;
    handle->rxData=&ESP8266_chr_buffer;
  }
}

void ESP8266_init(){
  
  uart_init(ESP8266_UART,ESP8266_Baud,ESP8266_UART_TX,ESP8266_UART_RX);
  
  NVIC_SetPriority(ESP8266_UART_IQRn,ESP8266_Priority);
  uart_rx_irq(ESP8266_UART,1);
  //设置中断回调,设置RX缓存,使其Size为1;
  uart_set_handle(ESP8266_UART,&ESP8266_Handle,ESP8266_Callback,NULL,0,&ESP8266_chr_buffer,1);
  
#ifdef ESP8266_ResetPin
  gpio_init(ESP8266_ResetPin,GPO,0,GPIO_PIN_CONFIG);
#endif
}


void ESP8266_send(char* str,uint16 size){
  uint16 cnt;
  for(cnt=0;cnt<size;cnt++){
    uart_putchar(ESP8266_UART,*(str+cnt));
  }
}

void ESP8266_sendStr(char* str){
  while(*str){
    uart_putchar(ESP8266_UART, *str++);
  }
}

// 返回值为打印字符的个数
// 支持%d，%o, %x，%s，%c，%f（只打印6位数字）
uint32 ESP8266_printf(const char *format, ...){
  uint32 buff_len=0;
  int i=0;
  va_list arg;
  va_start(arg, format);
  while (*format){
    int8 ret = *format;
    if (ret == '%'){
      switch (*++format){
        case 'a':// 十六进制p计数法输出浮点数 暂未实现
        {}break;

        case 'c':{// 一个字符
          char ch = (char)va_arg(arg, uint32);
          uart_putchar(ESP8266_UART, ch);
          buff_len++;
        }break;

        case 'd':
        case 'i':{// 有符号十进制整数
          int8 vstr[33];
          int32 ival = (int32)va_arg(arg, int32);
          uint8 vlen = number_to_ascii((uint32)ival, vstr, 1, 10);
          if(ival<0){
            vstr[vlen] = '-';
            vlen++;
          }
          //reverse_order(vstr,vlen);
          for(i=vlen-1;i>=0;i--){
            uart_putchar(ESP8266_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'f':// 浮点数，输出小数点后六位  不能指定输出精度
        case 'F':{// 浮点数，输出小数点后六位  不能指定输出精度
          int8 vstr[33];
          double ival = (double)va_arg(arg, double);
          uint8 vlen = number_to_ascii((uint32)(int32)ival, vstr, 1, 10);

          if(ival<0) {
            vstr[vlen] = '-';
            vlen++;
          }
          //reverse_order(vstr,vlen);
          for(i=vlen-1;i>=0;i--){
            uart_putchar(ESP8266_UART,(char)vstr[i]);
          }
          buff_len += vlen;

          ival = ((double)ival - (int32)ival)*1000000;
          if(ival) {
            vlen = number_to_ascii((uint32)(int32)ival, vstr, 1, 10);
          }else {
            vstr[0] = vstr[1] = vstr[2] = vstr[3] = vstr[4] = vstr[5] = '0';
            vlen = 6;
          }
                  
          while(6>vlen) {
            vstr[vlen] = '0';
            vlen++;
          }
                  
          vstr[vlen] = '.';
          vlen++;

          for(i=vlen-1;i>=0;i--){
            uart_putchar(ESP8266_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'u':{// 无符号十进制整数
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 10);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(ESP8266_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'o':{// 无符号八进制整数 
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 8);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(ESP8266_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'x':// 无符号十六进制整数
        case 'X':{// 无符号十六进制整数
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 16);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(ESP8266_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;


        case 's':{// 字符串
          int8 *pc = va_arg(arg, int8 *);
          while (*pc) {
            uart_putchar(ESP8266_UART,(char)*pc);
            buff_len++;
            pc++;
          }
        }break;

        case 'p':{// 以16进制形式输出指针
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 16);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(ESP8266_UART,(char)vstr[i]);
          }
          buff_len += 8;
        }break;


        case '%':{// 输出字符% 
          uart_putchar(ESP8266_UART,'%');
          buff_len++;
        }break;

        default:break;
      }
    }else {
      uart_putchar(ESP8266_UART,(char)*format);
      buff_len++;
    }
    format++;
  }
  va_end(arg);

  return buff_len;
}


void ESP8266_reset(){
  ESP8266_sendStr("AT+RST/r/n");
}

void ESP8266_connect(const char* IPaddr,uint32 port);
void ESP8266_tryExitPassThrough();
uint8 ESP8266_ping(const char* IPaddr);
void ESP8266_Socket_start();
void ESP8266_Connect_wifi(const char* SSID,const char* Password);