/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 电机库文件
 * 
 *   创建日期： 20180606
 *   作者：     宋博伟
 *   邮箱：     songbw123@163.com
 *
 *   版本：     v0.2
 *   修改日期   20180721
 *   修改：     宋博伟
 *   邮箱：     songbw123@163.com
 *   修改内容： 
 * 
 *   
 * 
 * 功能列表：
 *  // 定时器
 *  1.  void Setup_PID_Timer(void);         // 配置PID用定时器
 *  2.  void Uninstall_PID_Timer(void);     // 移除PID用定时器(会影响其它用到此定时器的功能)
 * 
 *  // 开环电机
 *  3.  void Setup_Motor(void);                                 // 配置电机
 *  4.  void Motor_Move(int motor, int speed, int direction);   // 开环电机控制函数
 * 
 *  // 闭环电机
 *  5.  void PID_Reset(struct PID_Struct_t *pid);                               // 重置PID计算过程值
 *  6.  void PID_Init(struct PID_Struct_t *pid, float Kp, float Ki, float Kd);  // PID参数初始化
 *  7.  void All_PID_Init();                                                    // 按默认PID参数初始化左右电机
 *  8.  void Setup_Motor_PID(void);                                             // 配置左右两个电机编码器
 *  9.  void PID_Speed(void);                                                   // 按PID输出控制左右两个电机
 *  10. void Set_L_Target(float target);                                        // 设定左轮目标速度(编码器计数值)
 *  11. void Set_R_Target(float target);                                        // 设定右轮目标速度(编码器计数值)
 *  12. int16_t Get_L_Speed(void);                                              // 获取左轮速度(编码器计数值)
 *  13. int16_t Get_R_Speed(void);                                              // 获取右轮速度(编码器计数值)
 *  14. int16_t Get_L_Target(void);                                             // 获取左轮目标(编码器计数值)
 *  15. int16_t Get_R_Target(void);                                             // 获取右轮目标(编码器计数值)
 * 
 * 
 * 
 ************************************************/

#include <Thunder_Motor.h>

#include "driver/periph_ctrl.h"
#include "driver/pcnt.h"

// #define PRINT_DEBUG_INFO

/***********************************************************/
#ifdef ENABLE_ENCODER_INT
// 编码器
volatile uint16_t Encoder_Counter_A = 0x7FFF; //计数器A
volatile uint16_t Encoder_Counter_B = 0x7FFF; //计数器B

// 左轮编码器中断
void ISR_Encoder_L(void)
{
  if(digitalRead(EN_L_B) != 1)
  {
    Encoder_Counter_A++;
  }
  else
  {
    Encoder_Counter_A--;
  }
}

// 右轮编码器中断
void ISR_Encoder_R(void)
{
  if(digitalRead(EN_R_B) == 1)
  {
    Encoder_Counter_B++;
  }
  else
  {
    Encoder_Counter_B--;
  }
}

// 左轮编码器中断2
void ISR_Encoder_L2(void)
{
  if(digitalRead(EN_L_A) != 1)
  {
    Encoder_Counter_A--;
  }
  else
  {
    Encoder_Counter_A++;
  }
}

// 右轮编码器中断2
void ISR_Encoder_R2(void)
{
  if(digitalRead(EN_R_A) == 1)
  {
    Encoder_Counter_B--;
  }
  else
  {
    Encoder_Counter_B++;
  }
}
#else

void Encoder_Counter_Clear(void);

// 编码器计数器数值
int16_t Encoder_Counter_Left;
int16_t Encoder_Counter_Right;

#endif

void Update_Rotate_Value();
void Get_Encoder_Value();

// PID时间中断
volatile SemaphoreHandle_t Timer_PID_Flag;
volatile uint32_t lastIsrAt = 0;

// 
volatile int32_t rotate_RawValue_Left;
volatile int32_t rotate_RawValue_Right;

// timer中断
void IRAM_ATTR PID_Timer_Handle()
{
  lastIsrAt = millis();
  Get_Encoder_Value();
  Update_Rotate_Value();
  xSemaphoreGiveFromISR(Timer_PID_Flag, NULL);
}

// 配置PID定时器
void THUNDER_MOTOR::Setup_PID_Timer()
{
  Timer_PID_Flag = xSemaphoreCreateBinary();
  PID_Timer = timerBegin(3, 80, true);      // 使用定时器3,80预分频
  timerAttachInterrupt(PID_Timer, &PID_Timer_Handle, true);  //Attach中断Handle
  timerAlarmWrite(PID_Timer, PID_dt * 1000, true);  // 50ms 中断
  timerAlarmEnable(PID_Timer);  // 使能
}

// 移除PID定时器(会影响其它用到此定时器的功能)
void THUNDER_MOTOR::Uninstall_PID_Timer(void)
{
  timerDetachInterrupt(PID_Timer);  //Detach中断Handle
}

// 配置电机
void THUNDER_MOTOR::Setup_Motor()
{
  ledcSetup(MOTOR_CHANNEL_2, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
  ledcSetup(MOTOR_CHANNEL_3, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);

  ledcAttachPin(MOTOR_L_PWM, MOTOR_CHANNEL_2);
  ledcAttachPin(MOTOR_R_PWM, MOTOR_CHANNEL_3);
  
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
}

// 配置电机速度
// 参数1-->通道
// 参数2-->功率
void THUNDER_MOTOR::set_speed(uint8_t channel, uint32_t value) 
{
  if(value > valueMax)
  {
    value = valueMax;
  }
  uint32_t duty = (MOTOR_MAX_DUTY / valueMax) * value;

  ledcWrite(channel, duty); //0-MOTOR_MAX_DUTY
}

// 开环电机控制函数
// 参数1-->电机编号；1或者2
// 参数2-->速度；范围为0-255
// 参数3-->方向
void THUNDER_MOTOR::Motor_Move(int motor, int speed, int direction)
{
  if(direction == 1)
  {  
    inPin1 = HIGH;  
    inPin2 = LOW;  
  }
  else
  {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  
  if(motor == 1)
  {
    digitalWrite(MOTOR_L_IN1, inPin1);  
    digitalWrite(MOTOR_L_IN2, inPin2);  

    // digitalWrite(MOTOR_L_IN1, inPin2);  
    // digitalWrite(MOTOR_L_IN2, inPin1);  

    set_speed(MOTOR_CHANNEL_2, speed);
  }
  else if(motor == 2)
  {  
    digitalWrite(MOTOR_R_IN1, inPin2);
    digitalWrite(MOTOR_R_IN2, inPin1);

    // digitalWrite(MOTOR_R_IN1, inPin1);
    // digitalWrite(MOTOR_R_IN2, inPin2);

    set_speed(MOTOR_CHANNEL_3, speed);
  }
}

// 重置PID计算过程值
void THUNDER_MOTOR::PID_Reset(struct PID_Struct_t *pid)
{
  pid->Err = 0;
  pid->SumErr = 0; 
  pid->LastErr = 0; 
    
  pid->LastOutP = 0;
  pid->LastOutI = 0;
  pid->LastOutD = 0;

  // pid->Out = 0;  
}

// PID参数初始化
void THUNDER_MOTOR::PID_Init(struct PID_Struct_t *pid, float Kp, float Ki, float Kd)
{
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;

  pid->Err = 0;
  pid->SumErr = 0; 
  pid->LastErr = 0; 

  pid->Out = 0;
}

// 按默认PID参数初始化左右电机
void THUNDER_MOTOR::All_PID_Init()
{
    float Kp,Ki,Kd;
    Kp = PID_Default_Kp;
    Ki = PID_Default_Ki;
    Kd = PID_Default_Kd;

    PID_Init(&Motor_L_Speed_PID, Kp, Ki, Kd);
    PID_Init(&Motor_R_Speed_PID, Kp, Ki, Kd);
}

float THUNDER_MOTOR::motor_PID(struct PID_Struct_t *pid)    //PID计算
{
  //P计算
  pid->Err = pid->Ref - pid->Fdb;   //计算误差
  pid->OutP = pid->Kp * pid->Err;     

  //I计算
  pid->SumErr += pid->Err;     //计算积累误差
  pid->OutI = pid->Ki * pid->SumErr;
  // if ( pid->Err == 0 )pid->SumErr /= 2;

  // if(pid->OutI > pid->IMax)         pid->OutI = pid->IMax;    //限制最大I值
  // else if(pid->OutI < (-1.0 * pid->IMax))   pid->OutI = -1.0 * pid->IMax;

  //D计算
  pid->OutD = pid->Kd * (pid->Err - pid->LastErr);
  pid->LastErr = pid->Err;
  // if(pid->OutD > pid->DMax)         pid->OutD = pid->DMax;    //限制最大D值
  // else if(pid->OutD < (-1.0 * pid->DMax))   pid->OutD = -1.0 * pid->DMax;

  //计算结果值
  pid->Out += pid->OutP + pid->OutI + pid->OutD - pid->LastOutP - pid->LastOutI - pid->LastOutD;  //已经加过的部分减去
  if(pid->Out > pid->OutMax)    pid->Out = pid->OutMax; //限制结果值范围
  else if (pid->Out < pid->OutMin)  pid->Out = pid->OutMin;

  if((pid->Out < pid->OutLeast) & (pid->Out > -pid->OutLeast) & (pid->Ref == 0))  pid->Out = 0; //限制最小输出值

  pid->LastOutP = pid->OutP;
  pid->LastOutI = pid->OutI;
  pid->LastOutD = pid->OutD;

  return pid->Out;
}

// 配置左右两个电机编码器
void THUNDER_MOTOR::Setup_Motor_PID()
{
  // Serial.printf("SSSSSSSSSS Setup_Motor_PID SSSSSSSSSS\n");

  pinMode(EN_L_A,INPUT);
  pinMode(EN_L_B,INPUT);
  pinMode(EN_R_A,INPUT);
  pinMode(EN_R_B,INPUT);

  Setup_PID_Timer();    //PID时间中断
  All_PID_Init();     //PID参数初始化

#ifdef ENABLE_ENCODER_INT
  attachInterrupt(EN_L_A,ISR_Encoder_L,RISING);
  attachInterrupt(EN_R_A,ISR_Encoder_R,RISING);

  attachInterrupt(EN_L_B,ISR_Encoder_L2,RISING);
  attachInterrupt(EN_R_B,ISR_Encoder_R2,RISING);
#else
  // set pulse cnt Operate
  pinMatrixInAttach(EN_L_A, PCNT_SIG_CH0_IN0_IDX, false);
  pinMatrixInAttach(EN_L_B, PCNT_CTRL_CH0_IN0_IDX, false);
  pinMatrixInAttach(EN_R_A, PCNT_SIG_CH0_IN1_IDX, false);
  pinMatrixInAttach(EN_R_B, PCNT_CTRL_CH0_IN1_IDX, false);

  /* Prepare configuration for the PCNT unit */
  pcnt_config_t pcnt_config;
  {
      // Set PCNT input signal and control GPIOs
      pcnt_config.pulse_gpio_num = EN_L_A;
      pcnt_config.ctrl_gpio_num = EN_L_B;
      pcnt_config.channel = PCNT_CHANNEL_0;
      pcnt_config.unit = PCNT_UNIT_0;
      // What to do on the positive / negative edge of pulse input?
      pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
      pcnt_config.neg_mode = PCNT_COUNT_DEC;   // Keep the counter value on the negative edge
      // What to do when control input is low or high?
      pcnt_config.lctrl_mode = PCNT_MODE_KEEP; // Reverse counting direction if low
      pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;    // Keep the primary counter mode if high
      // Set the maximum and minimum limit values to watch
      pcnt_config.counter_h_lim = 1000;
      pcnt_config.counter_l_lim = -1000;
  }
  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);
  pcnt_config.pulse_gpio_num=EN_R_A;
  pcnt_config.ctrl_gpio_num=EN_R_B;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
  pcnt_config.unit=PCNT_UNIT_1;
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_set_filter_value(PCNT_UNIT_1, 1000);
  pcnt_filter_enable(PCNT_UNIT_1);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_1);
#endif
}

// 按PID输出控制左右两个电机
void THUNDER_MOTOR::PID_Speed()
{
  // 在定时器里面获取了 计数器的数值，存在Encoder_Counter_Left Encoder_Counter_Right
  // Get_Encoder_Value(); 

  Motor_L_Speed_PID.Fdb = Encoder_Counter_Left;
  Motor_R_Speed_PID.Fdb = Encoder_Counter_Right;

#ifdef PRINT_DEBUG_INFO
  xQueueSend(Task_Mesg.Queue_encoder_left, &Motor_L_Speed_PID.Fdb, 0);
  xQueueSend(Task_Mesg.Queue_encoder_right, &Motor_R_Speed_PID.Fdb, 0);
#endif

  //SSSSSSSSSS ___ 左轮 ___ SSSSSSSSSS
  if(Motor_L_Speed_PID.Ref != 0)
  {
    motor_PID(&Motor_L_Speed_PID);
  }
  else
  {
    Motor_L_Speed_PID.Out = 0;
    PID_Reset(&Motor_L_Speed_PID);
  }

  if(Motor_L_Speed_PID.Out > 0)  //正转
  {
    Motor_Move(1, Motor_L_Speed_PID.Out, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
  }
  else
  {
    Motor_Move(1, -Motor_L_Speed_PID.Out, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
  }

  //SSSSSSSSSS ___ 右轮 ___ SSSSSSSSSS
  if(Motor_R_Speed_PID.Ref != 0)
  {
    motor_PID(&Motor_R_Speed_PID);  //参数2：目标；参数3：Encode值；参数4：时间
  }
  else
  {
    Motor_R_Speed_PID.Out = 0;
    PID_Reset(&Motor_R_Speed_PID);
  }
  
  if(Motor_R_Speed_PID.Out > 0)  //正转
  {
    Motor_Move(2, Motor_R_Speed_PID.Out, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
  }
  else
  {
    Motor_Move(2, -Motor_R_Speed_PID.Out, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
  }
}

// 设定左轮目标速度(编码器计数值)
void THUNDER_MOTOR::Set_L_Target(float target)
{
  if(Motor_L_Speed_PID.Ref != target)
  {
    PID_Reset(&Motor_L_Speed_PID);
    Motor_L_Speed_PID.Ref = target;
  }
}

// 设定右轮目标速度(编码器计数值)
void THUNDER_MOTOR::Set_R_Target(float target)
{
  if(Motor_R_Speed_PID.Ref != target)
  {
    PID_Reset(&Motor_R_Speed_PID);
    Motor_R_Speed_PID.Ref = target;
  }
}

// 获取左轮速度(编码器计数值), 这个数值是每个PID周期采集到的计数器数值
// 获取这个值，需要打开PID定时器
// 在一个PID周期内获取的值是相同的
int16_t THUNDER_MOTOR::Get_L_Speed(void)
{
  return Encoder_Counter_Left;
}

// 获取右轮速度(编码器计数值), 这个数值是每个PID周期采集到的计数器数值
// 获取这个值，需要打开PID定时器
// 在一个PID周期内获取的值是相同的
int16_t THUNDER_MOTOR::Get_R_Speed(void)
{
  return Encoder_Counter_Right;
}

// 获取左轮目标(编码器计数值)
int16_t THUNDER_MOTOR::Get_L_Target(void)
{
  return Motor_L_Speed_PID.Ref;
}

// 获取右轮目标(编码器计数值)
int16_t THUNDER_MOTOR::Get_R_Target(void)
{
  return Motor_R_Speed_PID.Ref;
}

/* 
 * 清零电机编码器，在初始化或者获取完计数器数值 等等情况之后可以调用此函数
 * 
 * 
 */
inline void Encoder_Counter_Clear()
{
  pcnt_counter_clear(PCNT_UNIT_0); // Left
  pcnt_counter_clear(PCNT_UNIT_1); // Right
}

/* 
 * 获取编码器计数器数值，保存到相应变量里面
 * 每次获取都会清零计数器
 */
inline void Get_Encoder_Value()
{
  
  pcnt_get_counter_value(PCNT_UNIT_0, &Encoder_Counter_Left);
  pcnt_get_counter_value(PCNT_UNIT_1, &Encoder_Counter_Right);
  
  Encoder_Counter_Clear();

}

/*
 * 在Encoder_Counter_Left Encoder_Counter_Right更新之后才能调用此更新过程
 *   不然会产生重复累积，导致数据错误
 * 
 */
inline void Update_Rotate_Value()
{
  rotate_RawValue_Left += Encoder_Counter_Left;
  rotate_RawValue_Right += Encoder_Counter_Right;
}

/*
 * 获取左轮旋转量，一定要有配置PID定时器 Setup_PID_Timer()，此调用才有效
 * 
 * @parameters: 
 * @return: 
 */
int32_t THUNDER_MOTOR::Get_L_RotateValue()
{
  return rotate_RawValue_Left;
}
/*
 * 获取右轮旋转量，一定要有配置PID定时器 Setup_PID_Timer()，此调用才有效
 * 
 * @parameters: 
 * @return: 
 */
int32_t THUNDER_MOTOR::Get_R_RotateValue()
{
  return rotate_RawValue_Right;
}
/*
 * 清零左轮旋转量记录，一定要有配置PID定时器 Setup_PID_Timer()，此调用才有效
 * 
 * @parameters: 
 * @return: 
 */
void THUNDER_MOTOR::Clear_L_RotateValue()
{
  rotate_RawValue_Left = 0;
}
/*
 * 清零右轮旋转量记录，一定要有配置PID定时器 Setup_PID_Timer()，此调用才有效
 * 
 * @parameters: 
 * @return: 
 */
void THUNDER_MOTOR::Clear_R_RotateValue()
{
  rotate_RawValue_Right = 0;
}
