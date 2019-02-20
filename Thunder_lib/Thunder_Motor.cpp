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
#include <Thunder_lib.h>

#include "driver/periph_ctrl.h"
#include "driver/pcnt.h"

// #define PRINT_DEBUG_INFO
// #define ENABLE_ENCODER_TIMER

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

// 
volatile int32_t rotate_RawValue_Left;
volatile int32_t rotate_RawValue_Right;

volatile uint32_t PID_Timer_Enable = 0;

#ifdef ENABLE_ENCODER_TIMER
// timer中断
void IRAM_ATTR PID_Timer_Handle()
{
  if(PID_Timer_Enable == 1){
    Get_Encoder_Value();
    Update_Rotate_Value();
    xSemaphoreGiveFromISR(Timer_PID_Flag, NULL);
  }
}

#else
void THUNDER_MOTOR::Update_Encoder_Value()
{
  Get_Encoder_Value();
  Update_Rotate_Value();
}

#endif


// 配置PID定时器
void THUNDER_MOTOR::Setup_PID_Timer()
{
  #ifdef ENABLE_ENCODER_TIMER
  Timer_PID_Flag = xSemaphoreCreateBinary();
  PID_Timer = timerBegin(3, 80, true);      // 使用定时器3,80预分频
  timerAttachInterrupt(PID_Timer, &PID_Timer_Handle, true);  //Attach中断Handle
  timerAlarmWrite(PID_Timer, PID_dt * 1000, true);  // 50ms 中断
  timerAlarmEnable(PID_Timer);  // 使能
  PID_Timer_Enable = 1;
  #endif
}

// 移除PID定时器(会影响其它用到此定时器的功能)
void THUNDER_MOTOR::Disable_PID_Timer(void)
{
  PID_Timer_Enable = 0;
}
// 重启PID定时器
void THUNDER_MOTOR::Enable_PID_Timer(void)
{
  PID_Timer_Enable = 1;
}

#if (MOTOR_DRIVER_IC == MOTOR_DRIVER_IC_TB6612)

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
void THUNDER_MOTOR::set_speed(uint8_t motor_channel,uint32_t speed)
{
  if(speed > valueMax)
  {
    speed = valueMax;
  }
  uint32_t duty = (speed * MOTOR_MAX_DUTY) / valueMax;

  ledcWrite(motor_channel, duty); //0-MOTOR_MAX_DUTY
}

// 开环电机控制函数
// 参数1-->电机编号；1或者2
// 参数2-->速度；范围为0-255
// 参数3-->方向, 0为反向，1为正向
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
    if(speed > LIMIT_SPEED){
      digitalWrite(MOTOR_L_IN1, inPin1);  
      digitalWrite(MOTOR_L_IN2, inPin2);  
      set_speed(MOTOR_CHANNEL_2, speed);
    }else{
      digitalWrite(MOTOR_L_IN1, LOW);
      digitalWrite(MOTOR_L_IN2, LOW);
      ledcWrite(MOTOR_CHANNEL_2, 0);
    }
  }
  else if(motor == 2)
  {  
    if(speed > LIMIT_SPEED){
      digitalWrite(MOTOR_R_IN1, inPin2);
      digitalWrite(MOTOR_R_IN2, inPin1);
      set_speed(MOTOR_CHANNEL_3, speed);
    }else{
      digitalWrite(MOTOR_R_IN1, LOW);
      digitalWrite(MOTOR_R_IN2, LOW);
      ledcWrite(MOTOR_CHANNEL_3, 0);
    }
  }
}

void THUNDER_MOTOR::Motor_Brake(int motor)
{
  if(motor == 1){
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, HIGH);
    ledcWrite(MOTOR_CHANNEL_2, 0);
  }else if(motor == 2){
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, HIGH);
    ledcWrite(MOTOR_CHANNEL_3, 0);
  }
}
// 开环电机滑行函数
// 参数1-->电机编号；1或者2
void THUNDER_MOTOR::Motor_Free(int motor)
{
  if(motor == 1)
  {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    ledcWrite(MOTOR_CHANNEL_2, 0);
  }
  else if(motor == 2)
  {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
    ledcWrite(MOTOR_CHANNEL_3, 0);
  }
}

#elif(MOTOR_DRIVER_IC == MOTOR_DRIVER_IC_PT5126)

// 配置电机
void THUNDER_MOTOR::Setup_Motor()
{
  pinMode(PWM_L_A, INPUT_PULLUP);
  if( digitalRead(PWM_L_A) == HIGH ){
    Serial.println("Motor IC is PT5126");
  }else{
    Serial.println("Motor IC is not PT5126");
  }

  ledcSetup(MOTOR_CHANNEL_2,MOTOR_BASE_FREQ,MOTOR_TIMER_13_BIT);
  ledcSetup(MOTOR_CHANNEL_3,MOTOR_BASE_FREQ,MOTOR_TIMER_13_BIT);
  ledcSetup(MOTOR_CHANNEL_4,MOTOR_BASE_FREQ,MOTOR_TIMER_13_BIT);
  ledcSetup(MOTOR_CHANNEL_5,MOTOR_BASE_FREQ,MOTOR_TIMER_13_BIT);

  ledcAttachPin(PWM_L_A,MOTOR_CHANNEL_2);
  ledcAttachPin(PWM_L_B,MOTOR_CHANNEL_3);
  ledcAttachPin(PWM_R_A,MOTOR_CHANNEL_4);
  ledcAttachPin(PWM_R_B,MOTOR_CHANNEL_5);

  pinMode(PWM_L_A, OUTPUT);
  pinMode(PWM_L_B, OUTPUT);
  pinMode(PWM_R_A, OUTPUT);
  pinMode(PWM_R_B, OUTPUT);
}

// 配置电机速度
// 参数1-->通道
// 参数2-->功率
void THUNDER_MOTOR::set_speed(uint8_t motor_channel, uint32_t speed) 
{
  uint8_t channel = 0;
  uint16_t speed_pulse = 0;
  if(speed > MOTOR_INPUT_MAX)
  {
    speed = MOTOR_INPUT_MAX;
  }
  switch(motor_channel)
  {
    case PWM_L_A:channel = MOTOR_CHANNEL_2;
                 break;
    case PWM_L_B:channel = MOTOR_CHANNEL_3;
                 break;
    case PWM_R_A:channel = MOTOR_CHANNEL_4;
                 break;
    case PWM_R_B:channel = MOTOR_CHANNEL_5;
                 break;
    default: break;       
  }
  speed_pulse = ( (speed * MOTOR_MAX_DUTY) / MOTOR_INPUT_MAX );
  ledcWrite(channel,speed_pulse);
}

// 开环电机控制函数
// 参数1-->电机编号；1或者2
// 参数2-->速度；范围为0-255
// 参数3-->方向, 0为反向，1为正向
void THUNDER_MOTOR::Motor_Move(int motor, int speed, int direction)
{
  if(speed > 255)
  {
    speed = 255;
  }
  else if(speed < 0)
  {
    speed = 0;
  }
  if(motor == 1)
  {
    if(speed > LIMIT_SPEED)
    {
      if(direction == 1)
      {  
        set_speed(PWM_L_A, MOTOR_INPUT_MAX);
        set_speed(PWM_L_B, MOTOR_INPUT_MAX - speed);
      }
      else if(direction == 2)
      {
        set_speed(PWM_L_A, MOTOR_INPUT_MAX - speed);
        set_speed(PWM_L_B, MOTOR_INPUT_MAX);
      }
    }
    else
    {
      set_speed(PWM_L_A,0);
      set_speed(PWM_L_B,0);
    }
  }
  else if(motor == 2)
  {
    if(speed > LIMIT_SPEED)
    {
      if(direction == 1)
      {  
        set_speed(PWM_R_A,MOTOR_INPUT_MAX - speed);
        set_speed(PWM_R_B,MOTOR_INPUT_MAX);
      }
      else if(direction == 2)
      {
        set_speed(PWM_R_A,MOTOR_INPUT_MAX);
        set_speed(PWM_R_B,MOTOR_INPUT_MAX - speed);
      }
    }
    else
    {
      set_speed(PWM_R_A,0);
      set_speed(PWM_R_B,0);
    }
  }
}

// 开环电机刹车函数
// 参数1-->电机编号；1或者2
void THUNDER_MOTOR::Motor_Brake(int motor)
{
  if(motor == 1)
  {
    set_speed(PWM_L_A,MOTOR_INPUT_MAX);
    set_speed(PWM_L_B,MOTOR_INPUT_MAX);
  }
  else if(motor == 2)
  {
    set_speed(PWM_R_A,MOTOR_INPUT_MAX);
    set_speed(PWM_R_B,MOTOR_INPUT_MAX);
  }
}

// 开环电机滑行函数
// 参数1-->电机编号；1或者2
void THUNDER_MOTOR::Motor_Free(int motor)
{
  if(motor == 1)
  {
    set_speed(PWM_L_A,0);
    set_speed(PWM_L_B,0);
  }
  else if(motor == 2)
  {
    set_speed(PWM_R_A,0);
    set_speed(PWM_R_B,0);
  }
}
#endif

// 参数1-->输出的值；范围为-255 ~ 255（负数为反向转，正为正向转；没有做速度PID控制）
void THUNDER_MOTOR::Set_L_Motor_Output( int M_output ){

  if( M_output >= 0 ){
    Motor_Move(1, M_output, 1);
  }else{
    Motor_Move(1, -1*M_output, 2);
  }
}
// 参数1-->输出的值；范围为-255 ~ 255（负数为反向转，正为正向转；没有做速度PID控制）
void THUNDER_MOTOR::Set_R_Motor_Output( int M_output ){

  if( M_output >= 0 ){
    Motor_Move(2, M_output, 1);
  }else{
    Motor_Move(2, -1*M_output, 2);
  }
}
// 参数1-->功率，会随电压浮动；范围为-100 ~ 100（负数为反向转，正为正向转；没有做速度PID控制）
void THUNDER_MOTOR::Set_L_Motor_Power( int Lpower ){
  float M_power;
  M_power = Lpower;
  M_power = (float)MAX_DRIVE_OUTPUT * ( (float)BATTERY_LOW_VALUE / Thunder.Get_Battery_Value() ) * ( M_power / 100 );

  if( M_power >= 0 ){
    Motor_Move(1, M_power, 1);
  }else{
    Motor_Move(1, -1*M_power, 2);
  }
}
// 参数1-->功率，会随电压浮动；范围为-100 ~ 100（负数为反向转，正为正向转；没有做速度PID控制）
void THUNDER_MOTOR::Set_R_Motor_Power( int Rpower ){
  float M_power;
  M_power = Rpower;
  M_power = (float)MAX_DRIVE_OUTPUT * ( (float)BATTERY_LOW_VALUE / Thunder.Get_Battery_Value() ) * ( M_power / 100 );

  if( M_power >= 0 ){
    Motor_Move(2, (int)M_power, 1);
  }else{
    Motor_Move(2, -1*(int)M_power, 2);
  }
}

// 重置PID计算过程值
void THUNDER_MOTOR::PID_Reset(struct PID_Struct_t *pid)
{
  pid->Err = 0;
  pid->SumErr = 0; 
  pid->LastErr = 0; 
    
  pid->OutP = 0;
  pid->OutI = 0;
  pid->OutD = 0;

  pid->LastOutP = 0;
  pid->LastOutI = 0;
  pid->LastOutD = 0;
  
  drive_car_pid.last_pid_time = 0;
  drive_car_pid.OutP_left = 0;
  drive_car_pid.OutI_left = 0;
  drive_car_pid.OutI_left_last = 0;
  drive_car_pid.OutD_left = 0;
  drive_car_pid.OutP_right = 0;
  drive_car_pid.OutI_right = 0;
  drive_car_pid.OutI_right_last = 0;
  drive_car_pid.OutD_right = 0;
  drive_car_pid.Out_left = 0;
  drive_car_pid.Out_right = 0;

  // pid->Out = 0;  
}
void THUNDER_MOTOR::PID_Reset()
{
  PID_Reset(&Motor_L_Speed_PID);
  PID_Reset(&Motor_R_Speed_PID);
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

  pid->OutP = 0;
  pid->OutI = 0;
  pid->OutD = 0;

  pid->LastOutP = 0;
  pid->LastOutI = 0;
  pid->LastOutD = 0;

  pid->Out = 0;
}

// 按默认PID参数初始化电机闭环控制的变量
void THUNDER_MOTOR::All_PID_Init()
{
    float Kp,Ki,Kd;
    Kp = PID_Default_Kp;
    Ki = PID_Default_Ki;
    Kd = PID_Default_Kd;
    
    drive_car_pid.last_pid_time = 0;
    drive_car_pid.OutP_left = 0;
    drive_car_pid.OutI_left = 0;
    drive_car_pid.OutI_left_last = 0;
    drive_car_pid.OutD_left = 0;
    drive_car_pid.Out_left = 0;
    
    drive_car_pid.OutP_right = 0;
    drive_car_pid.OutI_right = 0;
    drive_car_pid.OutI_right_last = 0;
    drive_car_pid.OutD_right = 0;
    drive_car_pid.Out_right = 0;

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

/* 
 * 控制电机运行，可以设置控制模式：0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
 * 
 * @parameters:
 * 
 * struct MotorRunning_Struct{
  //0代表left 和 right的控制都起效；1代表left控制起效，right无效; 2代表right控制起效，left无效
  byte motor_select;

  //0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
  byte running_mode;

  //左电机的转动速度
  float left_motor_speed;
  //右电机的转动速度
  float right_motor_speed;
  }; 
 *
 * @return: 
 */
void THUNDER_MOTOR::Control_Motor_Running(MotorRunning_Struct &running_data)
{
  switch(running_data.motor_select){
    case 0:{
      Set_L_Motor_Output(running_data.left_motor_speed * MAX_DRIVE_OUTPUT / 100);
      Set_R_Motor_Output(running_data.right_motor_speed * MAX_DRIVE_OUTPUT / 100);
      break;
    }
    case 1:{
      Set_L_Motor_Output(running_data.left_motor_speed * MAX_DRIVE_OUTPUT / 100);
      break;
    }
    case 2:{
      Set_R_Motor_Output(running_data.right_motor_speed * MAX_DRIVE_OUTPUT / 100);
      break;
    }
    default:
      return;
    break;
  }
  
  // 运行模式，0: 无模式；1：控制时间（秒）；2：控制角度（度）；3:控制圈数（圈）
  switch(running_data.running_mode){
    case 0:{
      // 不用停止电机，立刻返回，
      return;
    break;
    }
    case 1:{
      delay(1000*running_data.mode_data);
    break;
    }
    case 2:
    case 3:
    {
      int32_t last_left_RotateValue;
      int32_t last_right_RotateValue;
      int32_t circle2degrees;

      if(running_data.running_mode == 2){
        circle2degrees = running_data.mode_data;
      }else{
        circle2degrees = running_data.mode_data * 360;
      }

      last_left_RotateValue = Get_L_RotateValue();
      last_right_RotateValue = Get_R_RotateValue();

      if(running_data.motor_select == 1){
        while( abs( Get_L_RotateValue() - last_left_RotateValue ) < circle2degrees 
              && running_data.left_motor_speed != 0 ){
        }
      }else if(running_data.motor_select == 2){
        while( abs( Get_R_RotateValue() - last_right_RotateValue ) < circle2degrees 
              && running_data.right_motor_speed != 0 ){
        }
      }else{
        while( ( abs( Get_L_RotateValue() - last_left_RotateValue ) < circle2degrees 
              && running_data.left_motor_speed != 0 ) || 
              ( abs( Get_R_RotateValue() - last_right_RotateValue ) < circle2degrees 
              && running_data.right_motor_speed != 0 )
             ){
          if( abs( Get_L_RotateValue() - last_left_RotateValue ) >= circle2degrees ){
              Set_L_Motor_Output(0);
              break;  // 任意一个转到位置，即刻去停止电机
          }
          if( abs( Get_R_RotateValue() - last_right_RotateValue ) >= circle2degrees ){
              Set_R_Motor_Output(0);
              break; // 任意一个转到位置，即刻去停止电机
          }
        }
      }
    }
    break;

    default:
    break;
  }
  
  //运行完后，停止被控制的电机
  switch(running_data.motor_select){
    case 0:{
      Set_L_Motor_Output(0);
      Set_R_Motor_Output(0);
      break;
    }
    case 1:{
      Set_L_Motor_Output(0);
      break;
    }
    case 2:{
      Set_R_Motor_Output(0);
      break;
    }
    default:
      return;
    break;
  }
}

/* 
 * 控制电机转向运行，可以设置控制模式：0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
 * 
 * @parameters:
 * 
 * struct MotorTurnning_Struct{
    //0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
    byte turnning_mode;

    float mode_data;

    // 取值范围 -100~100
    float turn_percent;
    float motor_speed;
  };
 *
 * @return: 
 */
void THUNDER_MOTOR::Control_Motor_Turnning(MotorTurnning_Struct &turnning_data)
{
  Set_Car_Speed_Direction(turnning_data.motor_speed, turnning_data.turn_percent);

  // 运行模式，0: 无模式；1：控制时间（秒）；2：控制角度（度）；3:控制圈数（圈）
  switch(turnning_data.turnning_mode){
    case 0:{
      // 不用停止电机，立刻返回，
      return;
    break;
    }
    case 1:{
      delay(1000*turnning_data.mode_data);
    break;
    }
    case 2:{
      int32_t last_RotateValue;
      if(turnning_data.motor_speed == 0) break;

      if(turnning_data.turn_percent >= 0){
        last_RotateValue = Get_L_RotateValue();
        while( abs( Get_L_RotateValue() - last_RotateValue ) < turnning_data.mode_data ){
        }
      }else{
        last_RotateValue = Get_R_RotateValue();
        while( abs( Get_R_RotateValue() - last_RotateValue ) < turnning_data.mode_data ){
        }
      }

    break;
    }
    case 3:{
      int32_t last_RotateValue;
      int32_t circle2degrees;
      if(turnning_data.motor_speed == 0) break;

      circle2degrees = turnning_data.mode_data * 360;
      
      if(turnning_data.turn_percent >= 0){
        last_RotateValue = Get_L_RotateValue();
        while( abs( Get_L_RotateValue() - last_RotateValue ) < circle2degrees ){
        }
      }else{
        last_RotateValue = Get_R_RotateValue();
        while( abs( Get_R_RotateValue() - last_RotateValue ) < circle2degrees ){
        }
      }
    break;
    }
    default:
    break;
  }

  Set_Car_Speed_Direction(0, 0);
}

void THUNDER_MOTOR::Calculate_Left_Control()
{
  // calculate left motor out power
  drive_car_pid.OutP_left = drive_car_pid.left_speed_diff * drive_car_pid.Kp;
  drive_car_pid.OutI_left += drive_car_pid.left_speed_diff * drive_car_pid.Ki;
  drive_car_pid.Out_left = drive_car_pid.OutP_left + drive_car_pid.OutI_left;

  if( drive_car_pid.Out_left > MAX_DRIVE_OUTPUT ){
    drive_car_pid.Out_left = MAX_DRIVE_OUTPUT;
    drive_car_pid.OutI_left = drive_car_pid.OutI_left_last;
  }else if( drive_car_pid.Out_left < -MAX_DRIVE_OUTPUT ){
    drive_car_pid.Out_left = -MAX_DRIVE_OUTPUT;
    drive_car_pid.OutI_left = drive_car_pid.OutI_left_last;
  }else{
    drive_car_pid.OutI_left_last = drive_car_pid.OutI_left;
  }
}
void THUNDER_MOTOR::Calculate_Right_Control()
{
  // calculate right motor out power
  drive_car_pid.OutP_right = drive_car_pid.right_speed_diff * drive_car_pid.Kp;
  drive_car_pid.OutI_right += drive_car_pid.right_speed_diff * drive_car_pid.Ki;
  drive_car_pid.Out_right = drive_car_pid.OutP_right + drive_car_pid.OutI_right;

  if( drive_car_pid.Out_right > MAX_DRIVE_OUTPUT ){
    drive_car_pid.Out_right = MAX_DRIVE_OUTPUT;
    drive_car_pid.OutI_right = drive_car_pid.OutI_right_last;
  }else if( drive_car_pid.Out_right < -MAX_DRIVE_OUTPUT ){
    drive_car_pid.Out_right = -MAX_DRIVE_OUTPUT;
    drive_car_pid.OutI_right = drive_car_pid.OutI_right_last;
  }else{
    drive_car_pid.OutI_right_last = drive_car_pid.OutI_right;
  }
}

/* 
 * 控制速度和方向的PID函数
 * 
 * @parameters: 
 * @return: 
 */
void THUNDER_MOTOR::Drive_Car_Control()
{
  float time_interval;
  float left_speed, right_speed;

  if(drive_car_pid.last_pid_time == 0){
    time_interval = MOTOR_CONTROL_PERIOD;
    drive_car_pid.last_pid_time = millis();
  }else{
    time_interval = millis() - drive_car_pid.last_pid_time;
    drive_car_pid.last_pid_time = millis();
  }
  left_speed = Encoder_Counter_Left;
  left_speed = left_speed * (float)MOTOR_CONTROL_PERIOD / time_interval;
  right_speed = Encoder_Counter_Right;
  right_speed = right_speed * (float)MOTOR_CONTROL_PERIOD / time_interval;

// Serial.printf("LeftSpeed:%f, RightSpeed:%f \n", left_speed, right_speed);

  drive_car_pid.left_speed_diff = drive_car_pid.left_speed_target - left_speed;
  drive_car_pid.right_speed_diff = drive_car_pid.right_speed_target - right_speed;
  if(drive_direction == 0.0){
    // drive_car_pid.new_left_target = drive_car_pid.left_speed_target;
    // drive_car_pid.new_right_target = drive_car_pid.right_speed_target;

    if( (drive_car_pid.Out_left == 255 && drive_car_pid.left_speed_diff > 0) || 
        ( drive_car_pid.Out_left == -255 && drive_car_pid.left_speed_diff < 0) ){
      drive_car_pid.new_right_target = left_speed;
      drive_car_pid.right_speed_diff = drive_car_pid.new_right_target - right_speed;
      // Serial.printf("Right target:%f \n", drive_car_pid.new_right_target);
    }
    
    if( (drive_car_pid.Out_right == 255 && drive_car_pid.right_speed_diff > 0) || 
        ( drive_car_pid.Out_right == -255 && drive_car_pid.right_speed_diff < 0) ){
      drive_car_pid.new_left_target = right_speed;
      drive_car_pid.left_speed_diff = drive_car_pid.new_left_target - left_speed;
      // Serial.printf("Left target:%f \n", drive_car_pid.new_left_target);
    }
      // Serial.printf("Left target:%f, Right target:%f \n", drive_car_pid.new_left_target, drive_car_pid.new_right_target);
  }else if((drive_direction > 0.0 && drive_car_pid.Out_left == 255 && drive_car_pid.left_speed_diff > 0) || 
    (drive_direction > 0.0 && drive_car_pid.Out_left == -255 && drive_car_pid.left_speed_diff < 0)){
      drive_car_pid.new_right_target = left_speed * (MAX_DRIVE_DIRECTION/2 - drive_direction)
                                       / (MAX_DRIVE_DIRECTION/2);
      drive_car_pid.right_speed_diff = drive_car_pid.new_right_target - right_speed;

      // Serial.printf("Left target:%f, Right target:%f \n", (float)left_speed, new_right_target);
  }else if((drive_direction < 0.0 && drive_car_pid.Out_right == 255 && drive_car_pid.right_speed_diff > 0) || 
    (drive_direction < 0.0 && drive_car_pid.Out_right == -255 && drive_car_pid.right_speed_diff < 0)){
      drive_car_pid.new_left_target = right_speed * (MAX_DRIVE_DIRECTION/2 + drive_direction)
                                       / (MAX_DRIVE_DIRECTION/2);
      drive_car_pid.left_speed_diff = drive_car_pid.new_left_target - left_speed;

      // Serial.printf("Left target:%f, Right target:%f \n", new_left_target, (float)right_speed);
  }

// Serial.printf("LeftDiff:%f, RightDiff:%f \n", drive_car_pid.left_speed_diff, drive_car_pid.right_speed_diff);  
  Calculate_Left_Control();
  Calculate_Right_Control();

// Serial.printf("LeftOut:%f, RightOut:%f \n\n", drive_car_pid.Out_left, drive_car_pid.Out_right); 

  Set_L_Motor_Output((int)drive_car_pid.Out_left);
  Set_R_Motor_Output((int)drive_car_pid.Out_right);

}

/* 
 * 控制驾驶的速度和方向，
 * 速度:
 * 设置全速的百分比，值为 -100~100，-100为全速后退，100为全速前进
 * 正速度时，0方向为正前方；
 * 负速度时，0方向为正后方
 * 方向：
 * 0~360为右转，向前时0为正前方，360为顺时针打转；向后时0为正后方，360为逆时针打转；
 * 0~-360为左转，向前时0为正前方，-360为逆时针打转；向后时0为正后方，-360为顺时针打转；
 * 右转时：direction = 180 * (L-R)/L 左转时：direction = 180 * (L-R)/R
 * 
 * @parameters: 
 * @return: 
 */
void THUNDER_MOTOR::Set_Car_Speed_Direction(float speed, float direction)
{
  Thunder.Enable_Drive_Car();

  // All_PID_Init();

  if( speed > 100.0 ){
    drive_speed = 100.0;
  }else if( speed < -100.0 ){
    drive_speed = -100.0;
  }else{
    drive_speed = speed;
  }
  drive_speed = (drive_speed / 100.0) * MAX_DRIVE_SPEED;

  if( direction > MAX_DRIVE_DIRECTION ){
    drive_direction = MAX_DRIVE_DIRECTION;
  }else if( direction < -MAX_DRIVE_DIRECTION ){
    drive_direction = -MAX_DRIVE_DIRECTION;
  }else{
    drive_direction = direction;
  }

  if(drive_direction >= 0.0){
    drive_car_pid.P_direction_divisor = &Encoder_Counter_Left;
    drive_car_pid.left_speed_target = drive_speed;
    drive_car_pid.right_speed_target = drive_speed * (MAX_DRIVE_DIRECTION/2 - drive_direction) / (MAX_DRIVE_DIRECTION/2);
  }else{
    drive_car_pid.P_direction_divisor = &Encoder_Counter_Right;
    drive_car_pid.right_speed_target = drive_speed;
    drive_car_pid.left_speed_target = drive_speed * (MAX_DRIVE_DIRECTION/2 + drive_direction) / (MAX_DRIVE_DIRECTION/2);
  }

  drive_car_pid.new_left_target = drive_car_pid.left_speed_target;
  drive_car_pid.new_right_target = drive_car_pid.right_speed_target;

  // Serial.printf("Left target:%f, Right target:%f \n", drive_car_pid.left_speed_target, drive_car_pid.right_speed_target);
}

/* 
 * 负数为反方向转
 * PID闭环控制电机转速，PID控制周期为 MOTOR_CONTROL_PERIOD
 * PID控制周期采集的编码器数值最大为 MAX_DRIVE_SPEED
 * 最大PID控制速度 = MAX_DRIVE_SPEED * 1000 / MOTOR_CONTROL_PERIOD * 60 / ENCODER_NUM_EVERY_CIRCLE
 *                = 277.78(转/分)
 *                = 4.63(转/秒)
 * 
 * @parameters: target是最大PID控制速度的百分比
 * @return: 
 */
void THUNDER_MOTOR::Set_L_Target(float target)
{
  float target_encoder_num;

  Thunder.Enable_En_Motor();

  target_encoder_num = target / 100 * MAX_DRIVE_SPEED;
  if(Motor_L_Speed_PID.Ref != target_encoder_num)
  {
    // PID_Reset(&Motor_L_Speed_PID);
    Motor_L_Speed_PID.Ref = target_encoder_num;
  }
}
void THUNDER_MOTOR::Set_R_Target(float target)
{
  float target_encoder_num;

  Thunder.Enable_En_Motor();

  target_encoder_num = target / 100 * MAX_DRIVE_SPEED;
  if(Motor_R_Speed_PID.Ref != target_encoder_num)
  {
    // PID_Reset(&Motor_R_Speed_PID);
    Motor_R_Speed_PID.Ref = target_encoder_num;
  }
}

/*
 * 获取左右轮速度, 这个数值是电机最大PID控制速度 的 百分比
 * 获取这个值，需要打开PID定时器
 */
int16_t THUNDER_MOTOR::Get_L_Speed(void)
{
  return Encoder_Counter_Left * 100 / MAX_DRIVE_SPEED;
}
int16_t THUNDER_MOTOR::Get_R_Speed(void)
{
  return Encoder_Counter_Right * 100 / MAX_DRIVE_SPEED;
}

// 获取左轮目标(编码器计数值)
int16_t THUNDER_MOTOR::Get_L_Target(void)
{
  return Motor_L_Speed_PID.Ref * 100 / MAX_DRIVE_SPEED;
}

// 获取右轮目标(编码器计数值)
int16_t THUNDER_MOTOR::Get_R_Target(void)
{
  return Motor_R_Speed_PID.Ref * 100 / MAX_DRIVE_SPEED;
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
 * 电机编码器磁铁是四对磁极对，减速比是27
 * 编码器的计数器，计算了信号上下沿的数量，所以转一圈，编码器数值为216，实际测量也是
 * 轮子(没有轮胎)周长：16cm
 * 
 * @parameters: 
 * @return: 
 */
int32_t THUNDER_MOTOR::Get_L_RotateValue()
{
  int32_t rotate_value;

  rotate_value = rotate_RawValue_Left;
  rotate_value = rotate_value * DEGREES_EVERY_CIRCLE / ENCODER_NUM_EVERY_CIRCLE;
  
  return rotate_value;
}
int32_t THUNDER_MOTOR::Get_R_RotateValue()
{
  int32_t rotate_value;

  rotate_value = rotate_RawValue_Right;
  rotate_value = rotate_value * DEGREES_EVERY_CIRCLE / ENCODER_NUM_EVERY_CIRCLE;
  
  return rotate_value;
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
