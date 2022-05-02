#include"./Motor.h"



//编码器速度环***********
ST_PID_POS  m_stMotor_FL;       
ST_PID_POS  m_stMotor_FR;       
ST_PID_POS  m_stMotor_BL;       
ST_PID_POS  m_stMotor_BR;       

float m_f32PIDout_FL=0.0;
float m_f32PIDout_FR=0.0;
float m_f32PIDout_BL=0.0;
float m_f32PIDout_BR=0.0;

float m_f32PIDin_FL=0.0;
float m_f32PIDin_FR=0.0;
float m_f32PIDin_BL=0.0;
float m_f32PIDin_BR=0.0;

float g_f32Motor_Speed_FL;
float g_f32Motor_Speed_FR;
float g_f32Motor_Speed_BL;
float g_f32Motor_Speed_BR;
//编码器速度环***********

float m_f32MotorOut_FL=0.0;
float m_f32MotorOut_FR=0.0;
float m_f32MotorOut_BL=0.0;
float m_f32MotorOut_BR=0.0;


void Motor_update(){
  //获得PID中的实际项
#if Motor_FL_Encoder_Reverse
  m_f32PIDin_FL=(float)(-qtimer_quad_get(QTIMER_FL,QTIMER_FL_A));
#else
  m_f32PIDin_FL=(float)( qtimer_quad_get(QTIMER_FL,QTIMER_FL_A));
#endif
  
#if Motor_FR_Encoder_Reverse
  m_f32PIDin_FR=(float)(-qtimer_quad_get(QTIMER_FR,QTIMER_FR_A));
#else
  m_f32PIDin_FR=(float)( qtimer_quad_get(QTIMER_FR,QTIMER_FR_A));
#endif
  
#if Motor_BL_Encoder_Reverse
  m_f32PIDin_BL=(float)(-qtimer_quad_get(QTIMER_BL,QTIMER_BL_A));
#else
  m_f32PIDin_BL=(float)( qtimer_quad_get(QTIMER_BL,QTIMER_BL_A));
#endif
  
#if Motor_BR_Encoder_Reverse
   m_f32PIDin_BR=(float)(-qtimer_quad_get(QTIMER_BR,QTIMER_BR_A));
#else
   m_f32PIDin_BR=(float)( qtimer_quad_get(QTIMER_BR,QTIMER_BR_A));
#endif
  //清除编码器内部记数
  qtimer_quad_clear(QTIMER_FL,QTIMER_FL_A);
  qtimer_quad_clear(QTIMER_FR,QTIMER_FR_A);
  qtimer_quad_clear(QTIMER_BL,QTIMER_BL_A);
  qtimer_quad_clear(QTIMER_BR,QTIMER_BR_A);
  //位置式PID更新.
  PID_POS_update_PID(&m_stMotor_FL);
  PID_POS_update_PID(&m_stMotor_FR);
  PID_POS_update_PID(&m_stMotor_BL);
  PID_POS_update_PID(&m_stMotor_BR);
  
  m_f32MotorOut_FL=m_f32PIDout_FL;
  m_f32MotorOut_FR=m_f32PIDout_FR;
  m_f32MotorOut_BL=m_f32PIDout_BL;
  m_f32MotorOut_BR=m_f32PIDout_BR;

  //死区
  if(fabsf(*m_stMotor_FL.m_f32ptr_ideal)<=Speed_Death_Zone){
    m_f32MotorOut_FL=0;m_stMotor_FL.private_m_f32integral=0;
  }if(fabsf(*m_stMotor_FR.m_f32ptr_ideal)<=Speed_Death_Zone){
    m_f32MotorOut_FR=0;m_stMotor_FR.private_m_f32integral=0;
  }if(fabsf(*m_stMotor_BL.m_f32ptr_ideal)<=Speed_Death_Zone){
    m_f32MotorOut_BL=0;m_stMotor_BL.private_m_f32integral=0;
  }if(fabsf(*m_stMotor_BR.m_f32ptr_ideal)<=Speed_Death_Zone){
    m_f32MotorOut_BR=0;m_stMotor_BR.private_m_f32integral=0;
  }
  
  //对输出值进行限幅操作.(使PWM占空比在0.0~1.0之间)
  if (m_f32MotorOut_FL>=Motor_Encoder_Speed_Max){
    m_f32MotorOut_FL = Motor_Encoder_Speed_Max;
  }
  else if (m_f32MotorOut_FL<=-Motor_Encoder_Speed_Max){
    m_f32MotorOut_FL= -Motor_Encoder_Speed_Max;
  }
  if (m_f32MotorOut_FR>=Motor_Encoder_Speed_Max){
    m_f32MotorOut_FR = Motor_Encoder_Speed_Max;
  }
  else if (m_f32MotorOut_FR<=-Motor_Encoder_Speed_Max){
    m_f32MotorOut_FR= -Motor_Encoder_Speed_Max;
  }
  if (m_f32MotorOut_BL>=Motor_Encoder_Speed_Max){
    m_f32MotorOut_BL = Motor_Encoder_Speed_Max;
  }
  else if (m_f32MotorOut_BL<=-Motor_Encoder_Speed_Max){
    m_f32MotorOut_BL= -Motor_Encoder_Speed_Max;
  }
  if (m_f32MotorOut_BR>=Motor_Encoder_Speed_Max){
    m_f32MotorOut_BR = Motor_Encoder_Speed_Max;
  }
  else if (m_f32MotorOut_BR<=-Motor_Encoder_Speed_Max){
    m_f32MotorOut_BR= -Motor_Encoder_Speed_Max;
  }
  //负数值.则改变电机方向.
  
#if Motor_MotorTestMode
  #if Motor_FL_ON
    pwm_duty(MOTOR_FL_PWM, (int32)( Motor_Encoder_Speed_Max/8));
    gpio_set(MOTOR_FL_DIR,0);
  #endif
  #if Motor_FR_ON
    pwm_duty(MOTOR_FR_PWM, (int32)( Motor_Encoder_Speed_Max/8));
    gpio_set(MOTOR_FR_DIR,0);
  #endif
  #if Motor_BL_ON
    pwm_duty(MOTOR_BL_PWM, (int32)( Motor_Encoder_Speed_Max/8));
    gpio_set(MOTOR_BL_DIR,0);
  #endif
  #if Motor_BR_ON
    pwm_duty(MOTOR_BR_PWM, (int32)( Motor_Encoder_Speed_Max/8));
    gpio_set(MOTOR_BR_DIR,0);
  #endif
#else 
  #if Motor_FL_Motor_Reverse
    if(m_f32MotorOut_FL>=0){
      pwm_duty(MOTOR_FL_PWM, (int32)( m_f32MotorOut_FL));
      gpio_set(MOTOR_FL_DIR,1);
    }
    else{        
      pwm_duty(MOTOR_FL_PWM, (int32)(-m_f32MotorOut_FL));
      gpio_set(MOTOR_FL_DIR,0);
    }
  #else
    if(m_f32MotorOut_FL>=0){
      pwm_duty(MOTOR_FL_PWM, (int32)( m_f32MotorOut_FL));
      gpio_set(MOTOR_FL_DIR,0);
    }
    else{        
      pwm_duty(MOTOR_FL_PWM, (int32)(-m_f32MotorOut_FL));
      gpio_set(MOTOR_FL_DIR,1);
    }
  #endif
  #if Motor_FR_Motor_Reverse
    if(m_f32MotorOut_FR>=0){
      pwm_duty(MOTOR_FR_PWM, (int32)( m_f32MotorOut_FR));
      gpio_set(MOTOR_FR_DIR,1);
    }
    else{        
      pwm_duty(MOTOR_FR_PWM, (int32)(-m_f32MotorOut_FR));
      gpio_set(MOTOR_FR_DIR,0);
    } 
  #else
    if(m_f32MotorOut_FR>=0){
      pwm_duty(MOTOR_FR_PWM, (int32)( m_f32MotorOut_FR));
      gpio_set(MOTOR_FR_DIR,0);
    }
    else{        
      pwm_duty(MOTOR_FR_PWM, (int32)(-m_f32MotorOut_FR));
      gpio_set(MOTOR_FR_DIR,1);
    } 
  #endif
  #if Motor_BL_Motor_Reverse
    if(m_f32MotorOut_BL>=0){
      pwm_duty(MOTOR_BL_PWM, (int32)( m_f32MotorOut_BL));
      gpio_set(MOTOR_BL_DIR,1);
    }
    else{        
      pwm_duty(MOTOR_BL_PWM, (int32)(-m_f32MotorOut_BL));
      gpio_set(MOTOR_BL_DIR,0);
    }
  #else
    if(m_f32MotorOut_BL>=0){
      pwm_duty(MOTOR_BL_PWM, (int32)( m_f32MotorOut_BL));
      gpio_set(MOTOR_BL_DIR,0);
    }
    else{        
      pwm_duty(MOTOR_BL_PWM, (int32)(-m_f32MotorOut_BL));
      gpio_set(MOTOR_BL_DIR,1);
    }
  #endif
  #if Motor_BR_Motor_Reverse
    if(m_f32MotorOut_BR>=0){
      pwm_duty(MOTOR_BR_PWM, (int32)( m_f32MotorOut_BR));
      gpio_set(MOTOR_BR_DIR,1);
    }
    else{        
      pwm_duty(MOTOR_BR_PWM, (int32)(-m_f32MotorOut_BR));
      gpio_set(MOTOR_BR_DIR,0);
    }
  #else
    if(m_f32MotorOut_BR>=0){
      pwm_duty(MOTOR_BR_PWM, (int32)( m_f32MotorOut_BR));
      gpio_set(MOTOR_BR_DIR,0);
    }
    else{        
      pwm_duty(MOTOR_BR_PWM, (int32)(-m_f32MotorOut_BR));
      gpio_set(MOTOR_BR_DIR,1);
    }
  #endif
#endif
}

//初始化PWM引脚
void Pwm_init(){
  gpio_init(MOTOR_FR_DIR,GPO,0,FAST_GPIO_PIN_CONFIG);
  pwm_init( MOTOR_FR_PWM,Motor_Encoder_Speed_Max/2,0);

  gpio_init(MOTOR_FL_DIR,GPO,0,FAST_GPIO_PIN_CONFIG);
  pwm_init( MOTOR_FL_PWM,Motor_Encoder_Speed_Max/2,0);

  gpio_init(MOTOR_BR_DIR,GPO,0,FAST_GPIO_PIN_CONFIG);
  pwm_init( MOTOR_BR_PWM,Motor_Encoder_Speed_Max/2,0);

  gpio_init(MOTOR_BL_DIR,GPO,0,FAST_GPIO_PIN_CONFIG);
  pwm_init( MOTOR_BL_PWM,Motor_Encoder_Speed_Max/2,0);
}

//初始化正交编码器
void Encoder_init(){
  qtimer_quad_init(QTIMER_FL,QTIMER_FL_A,QTIMER_FL_B);
  qtimer_quad_init(QTIMER_FR,QTIMER_FR_A,QTIMER_FR_B);
  qtimer_quad_init(QTIMER_BL,QTIMER_BL_A,QTIMER_BL_B);
  qtimer_quad_init(QTIMER_BR,QTIMER_BR_A,QTIMER_BR_B);
}

void Motor_PID_init(){
  //绑定PID输出指针
  m_stMotor_FL.m_f32ptr_output=&m_f32PIDout_FL;
  m_stMotor_FR.m_f32ptr_output=&m_f32PIDout_FR;
  m_stMotor_BL.m_f32ptr_output=&m_f32PIDout_BL;
  m_stMotor_BR.m_f32ptr_output=&m_f32PIDout_BR;
  //绑定PID输出指针
  m_stMotor_FL.m_f32ptr_real=&m_f32PIDin_FL;
  m_stMotor_FR.m_f32ptr_real=&m_f32PIDin_FR;
  m_stMotor_BL.m_f32ptr_real=&m_f32PIDin_BL;
  m_stMotor_BR.m_f32ptr_real=&m_f32PIDin_BR;
  //绑定PID目标速度
  m_stMotor_FL.m_f32ptr_ideal=&g_f32Motor_Speed_FL;
  m_stMotor_FR.m_f32ptr_ideal=&g_f32Motor_Speed_FR;
  m_stMotor_BL.m_f32ptr_ideal=&g_f32Motor_Speed_BL;
  m_stMotor_BR.m_f32ptr_ideal=&g_f32Motor_Speed_BR;
  //PID参数初始化 
  //90:300
  PID_POS_init(&m_stMotor_FL,750,7,30);
  PID_POS_init(&m_stMotor_FR,750,7,30);
  PID_POS_init(&m_stMotor_BL,750,7,30);
  PID_POS_init(&m_stMotor_BR,750,7,30);
}

  
void Motor_init(){
  Pwm_init();
  Encoder_init();
  Motor_PID_init();
}



#if Motor_Rotation_Enable 
  //陀螺仪角度环***********
  ST_PID_POS  m_stMotor_Direction;
  float (*m_funcGetAngle)(float);

  float g_f32Motor_Rotation;
  
  float g_f32Motor_RealAngle;
  //陀螺仪角度环***********
  
  uint8 g_u8RotationEnable;
   
  float g_f32Motor_Speed_Index_FL=0;
  float g_f32Motor_Speed_Index_FR=0;
  float g_f32Motor_Speed_Index_BL=0;
  float g_f32Motor_Speed_Index_BR=0;
  
  
  float m_f32Motor_RotationOut=0;

  void Rotation_PID_init(float (*rotation_ptr)(float)){
    m_funcGetAngle=rotation_ptr;
    m_stMotor_Direction.m_f32ptr_real=&g_f32Motor_RealAngle;
    m_stMotor_Direction.m_f32ptr_ideal=&g_f32Motor_Rotation;
    m_stMotor_Direction.m_f32ptr_output=&m_f32Motor_RotationOut;
    g_u8RotationEnable=1;
    /*850~800*/
    PID_POS_init(&m_stMotor_Direction,7,0.1,80);
  }

  void Rotation_update(){
    if(g_u8RotationEnable){
      g_f32Motor_RealAngle=m_funcGetAngle(g_f32Motor_Rotation);
#if Section_Rotation_PID
      if(fabsf(g_f32Motor_Rotation-g_f32Motor_RealAngle)<=Section_Point_Angle_Error){
        PID_POS_update_PID(&m_stMotor_Direction);
      }else{
        m_stMotor_Direction.private_m_f32integral=0;
        PID_POS_update_PD(&m_stMotor_Direction);
      }
#else
      PID_POS_update_PID(&m_stMotor_Direction);
#endif
      
      if(m_f32Motor_RotationOut>=Motor_Rotation_Speed_Max){
        m_f32Motor_RotationOut=Motor_Rotation_Speed_Max;
      }if(m_f32Motor_RotationOut<=-Motor_Rotation_Speed_Max){
        m_f32Motor_RotationOut=-Motor_Rotation_Speed_Max;
      }
      
      g_f32Motor_Speed_FL = g_f32Motor_Speed_Index_FL - m_f32Motor_RotationOut;
      g_f32Motor_Speed_FR = g_f32Motor_Speed_Index_FR + m_f32Motor_RotationOut;
      g_f32Motor_Speed_BL = g_f32Motor_Speed_Index_BL - m_f32Motor_RotationOut;
      g_f32Motor_Speed_BR = g_f32Motor_Speed_Index_BR + m_f32Motor_RotationOut;
      
    }else{
      
      g_f32Motor_Speed_FL = g_f32Motor_Speed_Index_FL;
      g_f32Motor_Speed_FR = g_f32Motor_Speed_Index_FR;
      g_f32Motor_Speed_BL = g_f32Motor_Speed_Index_BL;
      g_f32Motor_Speed_BR = g_f32Motor_Speed_Index_BR;
    }
  }
  
  
#endif

void Speed_clear(){
  pwm_duty(MOTOR_FL_PWM, 0);
  pwm_duty(MOTOR_FR_PWM, 0);
  pwm_duty(MOTOR_BL_PWM, 0);
  pwm_duty(MOTOR_BR_PWM, 0);
}

void Motor_PID_buffer_clear(){
  PID_POS_buffer_clear(&m_stMotor_FL);
  PID_POS_buffer_clear(&m_stMotor_FR);
  PID_POS_buffer_clear(&m_stMotor_BL);
  PID_POS_buffer_clear(&m_stMotor_BR);
#if Motor_Rotation_Enable
  PID_POS_buffer_clear(&m_stMotor_Direction);
#endif
}
