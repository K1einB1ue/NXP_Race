#include"../Application/Application.h"

void (*InitCallback)(void);

void Init(){
  board_init();
  systick_delay_ms(300);
  
  DisableGlobalIRQ();
  oled_init();
  InitCallback();
  
  pit_init();
  //ͨ��0Ϊ5ms
  pit_interrupt_ms(PIT_CH0,5);
  //ͨ��1Ϊ2ms
  pit_interrupt_ms(PIT_CH1,2);
  NVIC_SetPriority(PIT_IRQn,PIT_Priority);
  
  
  systick_delay_ms(300);
  EnableGlobalIRQ(0);
}


