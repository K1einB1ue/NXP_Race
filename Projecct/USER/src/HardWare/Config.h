//¶¨Òå°´¼üÒý½Å
//#define Key1    C16
//#define Key2    C15
//#define Key3    C14
#define KeyIrqHandler           GPIO2_Combined_0_15_IRQHandler//GPIO3_Combined_0_15_IRQHandler
#define KeyIrq                  GPIO2_Combined_0_15_IRQn
#define OpenKey                 C4

////¶¨Òå²¦Âë¿ª¹ØÒý½Å
//#define SW1     C22
//#define SW2     C20

#define PIT_Priority             15
//Ñ­¼£´«¸ÐÆ÷X´®¿ÚÅäÖÃ
#define TracerX_Baud            115200
#define TracerX_UART            USART_1
#define TracerX_UART_TX         UART1_TX_B12
#define TracerX_UART_RX         UART1_RX_B13
#define TracerX_UART_IQRn       LPUART1_IRQn
#define TracerX_Priority        15
//Ñ­¼£´«¸ÐÆ÷Y´®¿ÚÅäÖÃ
#define TracerY_Baud            115200
#define TracerY_UART            USART_6
#define TracerY_UART_TX         UART6_TX_B2
#define TracerY_UART_RX         UART6_RX_B3
#define TracerY_UART_IQRn       LPUART6_IRQn
#define TracerY_Priority        15
//SYN6288´®¿ÚÅäÖÃ
#define SYN6288_Baud            9600
#define SYN6288_UART            USART_4
#define SYN6288_UART_TX         UART4_TX_C16
#define SYN6288_UART_RX         UART4_RX_C17
#define SYN6288_UART_IQRn       LPUART4_IRQn
#define SYN6288_Priority        15
//OpenArt´®¿ÚÅäÖÃ
#define OPENART_Baud            115200
#define OPENART_UART            USART_3
#define OPENART_UART_TX         UART3_TX_C8
#define OPENART_UART_RX         UART3_RX_C9
#define OPENART_UART_IQRn       LPUART3_IRQn
#define OPENART_Priority        14
//½Ç¶È´«¸ÐÆ÷´®¿ÚÅäÖÃ
#define HWT101_Baud             115200
#define HWT101_UART             USART_5
#define HWT101_UART_TX          UART5_TX_C28
#define HWT101_UART_RX          UART5_RX_C29
#define HWT101_UART_IQRn        LPUART5_IRQn
#define HWT101_Priority         13
#define HWT101_ResetPin         C20
//À¶ÑÀ´®¿ÚÅäÖÃ
#define BlueTooth_Baud          115200
#define BlueTooth_UART          USART_8
#define BlueTooth_UART_TX       UART8_TX_D16
#define BlueTooth_UART_RX       UART8_RX_D17
#define BlueTooth_UART_IQRn     LPUART8_IRQn
#define BlueTooth_Priority      14
//ESP8266
#define ESP8266_Baud            115200
#define ESP8266_UART            USART_3
#define ESP8266_UART_TX         UART3_TX_C8
#define ESP8266_UART_RX         UART3_RX_C9
#define ESP8266_UART_IQRn       LPUART3_IRQn
#define ESP8266_Priority        14
//µç»úÇý¶¯
#define MOTOR_BL_PWM     PWM1_MODULE3_CHA_D0
#define MOTOR_BL_DIR     D1
#define MOTOR_BR_PWM     PWM2_MODULE3_CHA_D2
#define MOTOR_BR_DIR     D3
#define MOTOR_FL_PWM     PWM1_MODULE0_CHA_D12
#define MOTOR_FL_DIR     D13
#define MOTOR_FR_PWM     PWM1_MODULE1_CHA_D14
#define MOTOR_FR_DIR     D15

//#define BEEP B11

//Õý½»±àÂëÆ÷
//Öð·É°å±àÂëÆ÷1
#define QTIMER_BL       QTIMER_1
#define QTIMER_BL_A     QTIMER1_TIMER0_C0
#define QTIMER_BL_B     QTIMER1_TIMER1_C1
//Öð·É°å±àÂëÆ÷2
#define QTIMER_BR       QTIMER_1
#define QTIMER_BR_A     QTIMER1_TIMER2_C2
#define QTIMER_BR_B     QTIMER1_TIMER3_C24
//Öð·É°å±àÂëÆ÷3
#define QTIMER_FR       QTIMER_2
#define QTIMER_FR_A     QTIMER2_TIMER0_C3
#define QTIMER_FR_B     QTIMER2_TIMER3_C25
//Öð·É°å±àÂëÆ÷4
#define QTIMER_FL       QTIMER_3
#define QTIMER_FL_A     QTIMER3_TIMER2_B18
#define QTIMER_FL_B     QTIMER3_TIMER3_B19


