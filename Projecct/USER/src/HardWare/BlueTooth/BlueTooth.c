#include"./BlueTooth.h"
#define Buffer_Size 30

lpuart_handle_t BlueTooth_Handle;
uint8 BlueTooth_buffer[Buffer_Size];
uint8 BlueTooth_chr_buffer;
uint8 BlueTooth_num=0;
uint8 BlueTooth_recv_enable=0;
uint8 BlueTooth_recv_really=0;

__attribute__((weak)) void BlueTooth_decode(const char* str,uint8 size){}

void BlueTooth_update(){
  if(BlueTooth_recv_really){
    BlueTooth_decode((const char*)BlueTooth_buffer,BlueTooth_num);
    BlueTooth_recv_really=0;
    BlueTooth_num=0;
  }
}

void BlueTooth_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData){
  if(status == kStatus_LPUART_RxIdle){
    if(BlueTooth_num>=Buffer_Size){
      BlueTooth_num=0;
    }
    if(BlueTooth_chr_buffer=='}'){
      BlueTooth_recv_enable=0;
      BlueTooth_recv_really=1;
    }
    if(BlueTooth_recv_enable&&!BlueTooth_recv_really){
      BlueTooth_buffer[BlueTooth_num++]=BlueTooth_chr_buffer;
    }
    if(BlueTooth_chr_buffer=='{'){
      BlueTooth_recv_enable=1;
    }
    
    handle->rxDataSize=1;
    handle->rxData=&BlueTooth_chr_buffer;
  }
}

void BlueTooth_init(){
  uart_init(BlueTooth_UART,BlueTooth_Baud,BlueTooth_UART_TX,BlueTooth_UART_RX);
  
  NVIC_SetPriority(BlueTooth_UART_IQRn,BlueTooth_Priority);
  uart_rx_irq(BlueTooth_UART,1);
  //�����жϻص�,����RX����,ʹ��SizeΪ1;
  uart_set_handle(BlueTooth_UART,&BlueTooth_Handle,BlueTooth_Callback,NULL,0,&BlueTooth_chr_buffer,1);
}


void BlueTooth_send(char* str,uint16 size){
  uint16 cnt;
  for(cnt=0;cnt<size;cnt++){
    uart_putchar(BlueTooth_UART,*(str+cnt));
  }
}

void BlueTooth_sendStr(char* str){
  while(*str){
    uart_putchar(BlueTooth_UART, *str++);
  }
}



// ����ֵΪ��ӡ�ַ��ĸ���
// ֧��%d��%o, %x��%s��%c��%f��ֻ��ӡ6λ���֣�
uint32 BlueTooth_printf(const char *format, ...){
  uint32 buff_len=0;
  int i=0;
  va_list arg;
  va_start(arg, format);
  while (*format){
    int8 ret = *format;
    if (ret == '%'){
      switch (*++format){
        case 'a':// ʮ������p��������������� ��δʵ��
        {}break;

        case 'c':{// һ���ַ�
          char ch = (char)va_arg(arg, uint32);
          uart_putchar(BlueTooth_UART, ch);
          buff_len++;
        }break;

        case 'd':
        case 'i':{// �з���ʮ��������
          int8 vstr[33];
          int32 ival = (int32)va_arg(arg, int32);
          uint8 vlen = number_to_ascii((uint32)ival, vstr, 1, 10);
          if(ival<0){
            vstr[vlen] = '-';
            vlen++;
          }
          //reverse_order(vstr,vlen);
          for(i=vlen-1;i>=0;i--){
            uart_putchar(BlueTooth_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'f':// �����������С�������λ  ����ָ���������
        case 'F':{// �����������С�������λ  ����ָ���������
          int8 vstr[33];
          double ival = (double)va_arg(arg, double);
          uint8 vlen = number_to_ascii((uint32)(int32)ival, vstr, 1, 10);

          if(ival<0) {
            vstr[vlen] = '-';
            vlen++;
          }
          //reverse_order(vstr,vlen);
          for(i=vlen-1;i>=0;i--){
            uart_putchar(BlueTooth_UART,(char)vstr[i]);
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
            uart_putchar(BlueTooth_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'u':{// �޷���ʮ��������
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 10);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(BlueTooth_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'o':{// �޷��Ű˽������� 
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 8);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(BlueTooth_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;

        case 'x':// �޷���ʮ����������
        case 'X':{// �޷���ʮ����������
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 16);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(BlueTooth_UART,(char)vstr[i]);
          }
          buff_len += vlen;
        }break;


        case 's':{// �ַ���
          int8 *pc = va_arg(arg, int8 *);
          while (*pc) {
            uart_putchar(BlueTooth_UART,(char)*pc);
            buff_len++;
            pc++;
          }
        }break;

        case 'p':{// ��16������ʽ���ָ��
          int8 vstr[33];
          uint32 ival = (uint32)va_arg(arg, uint32);
          uint8 vlen = number_to_ascii(ival, vstr, 0, 16);

          for(i=vlen-1;i>=0;i--){
            uart_putchar(BlueTooth_UART,(char)vstr[i]);
          }
          buff_len += 8;
        }break;


        case '%':{// ����ַ�% 
          uart_putchar(BlueTooth_UART,'%');
          buff_len++;
        }break;

        default:break;
      }
    }else {
      uart_putchar(BlueTooth_UART,(char)*format);
      buff_len++;
    }
    format++;
  }
  va_end(arg);

  return buff_len;
}