/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 雷霆库文件
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
 *  1.  void Setup_All(void);                           // 所有模块初始化
 *  2.  void Stop_All(void);                            // 全部终止(电机)
 *  3.  void Setup_Battery(void);                       // 电池电压检测初始化配置
 *  4.  float Get_Battery_Data(void);                   // 获取电池电压
 *  5.  void En_Motor(void);                            // 编码电机  闭环计算
 *  6.  void Enable_En_Motor(void);                     // 打开编码电机计算
 *  7.  void Disable_En_Motor(void);                    // 关闭编码电机计算
 *  8.  void Setup_Servo(void);                         // 舵机初始化配置
 *  9.  void Servo_Turn(int servo, int angle);          // 舵机角度控制
 *  10. void Setup_IR(void);                            // 巡线IR传感器初始化配置
 *  11. void Get_IR_Data(uint8_t data[]);               // 获取巡线IR数据
 *  12. void Line_Tracing(void);                        // 巡线模式
 *  13. void Start_Show(void);                          // 开机动画/声效
 *  14. void Wait_BLE(void);                            // 等待蓝牙连接动画 (有串口数据也跳出)
 *  15. void Set_LED_Show_No(uint8_t Show_No);          // 设置将要播放的内置动画编号
 *  16. void LED_Show(void);                            // 循环执行的内置动画控制程序
 *  17. void Check_Communication(void);                 // 通信确认，蓝牙/串口
 *  18. void Check_Protocol(void);                      // 协议解析
 *  19. void Reset_Rx_Data(void);                       // 清空接收数据
 *  20. void Get_Queue_Encoder(void);                   // 通过队列获取编码器数据
 *  21. uint8_t Set_I2C_Chanel(uint8_t channelData);    // 增加了TCA9548芯片，选通I2C端口，变量channelData相应位为1 是 选通
 * 
 * 
 ************************************************/

#include <Thunder_lib.h>

#define DEBUG_PRINT_INFO_COMMAND

THUNDER Thunder;

THUNDER_BLE Thunder_BLE;
THUNDER_MOTOR Thunder_Motor;

// 音频
WT588 Speaker = WT588(AUDIO,BUSY);    // 配置引脚16 --> AUDIO 和 4 --> BUSY

// 超声波
US_I2C US(ADD_I2C_US);

// 颜色识别
BH1745NUC Colour_Sensor(ADD_I2C_COLOUR);  //I2C从机地址

// 彩色LED
XT1511_I2C I2C_LED(0x11);

// 单色色LED
DOT_MATRIX_LED Dot_Matrix_LED;

// 蓝牙
uint8_t Rx_Data[6] = {0};
uint8_t Tx_Data[6] = {0};
bool deviceConnected = false;

// 固件版本号 2 bytes, 分别为整数和小数，
// 要同时修改头文件的 宏 VERSION
const uint8_t Version_FW[2] = {0,30};

// 所有模块初始化
void THUNDER::Setup_All(void)
{
  // delay(1000);
  Serial.begin(115200);
  while (!Serial);

  Serial.printf("\nSSS___ 雷霆固件版本 : V%.2f ___SSS\n",VERSION);
  Serial.printf("\nSSSSSSSSSS___ 各模块初始化 ___SSSSSSSSSS\n");
  
  Wire.begin(SDA_PIN, SCL_PIN, 100000); //Wire.begin();
  Select_Sensor_AllChannel();
  
  Thunder_BLE.Setup_EEPROM();         // 配置EEPROM
  Thunder_BLE.Setup_BLE();            // 配置BLE

  Colour_Sensor.Setup();              // 配置颜色传感器
  Thunder_Motor.Setup_Motor();        // 配置电机
  Thunder_Motor.Setup_Motor_PID();    // 配置左右两个电机编码器
  Enable_En_Motor();                  // 打开编码电机计算

  Setup_Servo();            // 舵机初始化配置
  Setup_IR();               // 巡线IR传感器初始化配置
  Setup_Battery();          // 电池电压检测初始化配置
  Dot_Matrix_LED.Setup();   // 初始化单色LED驱动IC配置

  I2C_LED.LED_OFF();        // 彩灯全关，立即刷新

  Serial.printf("SSSSSSSSSS___ 初始化完成 ___SSSSSSSSSS\n\n");

  // 开机动画/声效
  Start_Show(); 
}

// 全部终止(电机)
void THUNDER::Stop_All(void)
{
  Serial.printf("SSS___ 全部终止 ___SSS\n");

  Thunder_Motor.Set_L_Target(0);  // 50ms 最大40  //编码器计数值
  Thunder_Motor.Set_R_Target(0);

  Disable_En_Motor();   // 关闭编码电机计算

  Thunder_Motor.Motor_Move(1, 0, 1); // 参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
  Thunder_Motor.Motor_Move(2, 0, 2); // 参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
}

// 电池电压检测初始化配置
void THUNDER::Setup_Battery()
{
 pinMode(BATTERY_ADC_PIN,INPUT);
}

// 获取电池电压
float THUNDER::Get_Battery_Data()
{
  float Battery_Voltage = 0;
  
  ADC_Battery = analogRead(BATTERY_ADC_PIN);  // 3.3V --> FFF(4095)

  Battery_Voltage = ADC_Battery * 3.3 * (ADC_R_1 + ADC_R_2) / ADC_R_1 / 4095;  // 分压电阻为：51k;100k

  return Battery_Voltage;
}

// 编码电机  闭环计算
void THUNDER::En_Motor(void)
{
  if (xSemaphoreTake(Timer_PID_Flag, portMAX_DELAY ) == pdTRUE)  // 控制周期PID_dt[ms]
  {
    if(En_Motor_Flag == 1)
    {
      Thunder_Motor.PID_Speed();

      // Serial.printf("%d  %d  %d  %d  \n",Thunder_Motor.Get_L_Speed(),Thunder_Motor.Get_R_Speed(),Thunder_Motor.Get_L_Target(),Thunder_Motor.Get_R_Target());
      // Serial.printf("%d\n",Thunder_Motor.Get_L_Speed());
    }
  }
}

// 打开编码电机计算
void THUNDER::Enable_En_Motor(void)
{
  En_Motor_Flag = 1;
}

// 关闭编码电机计算
void THUNDER::Disable_En_Motor(void)
{
  En_Motor_Flag = 0;
}

// 舵机初始化配置
void THUNDER::Setup_Servo(void)
{
  ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);
  ledcSetup(SERVO_CHANNEL_1, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);
  
  ledcAttachPin(SERVO_A, SERVO_CHANNEL_0);
  ledcAttachPin(SERVO_B, SERVO_CHANNEL_1);
}

// 舵机角度控制
// 参数1 --> 舵机编号；1-->A口；2-->B口
// 参数2 --> 角度[°]；范围为0-180
void THUNDER::Servo_Turn(int servo, int angle)
{
  if(angle > 180)
  {
    angle = 180;
  }
  else if(angle < 0)
  {
    angle = 0;
  }

  if(servo == 1)  //A口
  {
    ledcWrite(SERVO_CHANNEL_0, Servo_MIN + Servo_Range * angle / 180); //大的舵机，最小260，最大950
  }
  else if(servo == 2) //B口
  {
    ledcWrite(SERVO_CHANNEL_1, Servo_MIN + Servo_Range * angle / 180); //大的舵机，最小260，最大950
  }
  else
  {
    Serial.printf("SSS___ 未定义舵机 ___SSS\n");
  }
}

// 巡线IR传感器初始化配置
void THUNDER::Setup_IR()
{
  pinMode(IR_1,INPUT);
  pinMode(IR_2,INPUT);
}

// 获取巡线IR数据
void THUNDER::Get_IR_Data(uint8_t data[])
{
  data[0] = digitalRead(IR_1);
  data[1] = digitalRead(IR_2);
}

// 巡线模式
void THUNDER::Line_Tracing(void)
{
  Line_last_time = millis();
  Line_last_led_time = millis();
  Line_last_sound_time = millis();
  LED_counter = 99;

  while(Rx_Data[1] == 1)
  {
    Get_IR_Data(IR_Data);  //更新IR数据 //0-->白; 1-->黑
    // Serial.printf("SSSSSSSSSS IR_Data[0] : %d ___ IR_Data[1] : %d SSSSSSSSSS\n",IR_Data[0],IR_Data[1]);

    current_time = millis();
    
    if((IR_Data[0] == 0) & (IR_Data[1] == 0))     //Serial.printf("SSSSSSSSSS 没线 SSSSSSSSSS\n");
    {
      // Speaker.Play_Song(7); //test用----------------------------------------------------------------------------
      if(((current_time - 5000) > Line_last_time)  & (current_time > 19000)) //超时未找到线停止
      {
        Thunder_Motor.Set_L_Target(0);  //50ms 最大40  //编码器计数值
        Thunder_Motor.Set_R_Target(0);
        line_state = 5;
      }
      else if((line_state == 0) | (line_state == 3) | (line_state == 4))  //直行/假左/假右的过程中出线后退200ms
      {
        // Speaker.Play_Song(5); //test用----------------------------------------------------------------------------
        Thunder_Motor.Set_L_Target(Line_B_Speed);  //50ms 最大40  //编码器计数值
        Thunder_Motor.Set_R_Target(Line_B_Speed);
        
        Line_last_time = millis();
        while((current_time - 200) < Line_last_time)
        {
          current_time = millis();
          En_Motor();
        }
      }
      else if(line_state == 1)
      {
        Thunder_Motor.Set_L_Target(Line_B_Speed);  //50ms 最大40  //编码器计数值
        Thunder_Motor.Set_R_Target(Line_L_Speed);
      }
      else if(line_state == 2)
      {
        Thunder_Motor.Set_L_Target(Line_L_Speed);  //50ms 最大40  //编码器计数值
        Thunder_Motor.Set_R_Target(Line_B_Speed);
      }
      else
      {
        Thunder_Motor.Set_L_Target(0);  //50ms 最大40  //编码器计数值
        Thunder_Motor.Set_R_Target(0);
      }
    }
    else if(IR_Data[0] == 0)      //Serial.printf("SSSSSSSSSS 右转 SSSSSSSSSS\n");
    {
      if((line_state == 1) | (line_state == 3)) //从左转过来的需要更新时间
      {
        Line_last_time = millis();
      }
      
      Thunder_Motor.Set_L_Target(Line_H_Speed);  //50ms 最大40  //编码器计数值
      Thunder_Motor.Set_R_Target(0);

      if(line_state == 2)
      {
        Line_last_time = millis();
      }
      else if((current_time - 50) > Line_last_time)
      {
        line_state = 2;
      }
      else
      {
        line_state = 4;
      }
    }
    else if(IR_Data[1] == 0)      //Serial.printf("SSSSSSSSSS 左转 SSSSSSSSSS\n");
    {
      if((line_state == 2) | (line_state == 4)) //从右转过来的需要更新时间
      {
        Line_last_time = millis();
      }
      
      Thunder_Motor.Set_L_Target(0);  //50ms 最大40  //编码器计数值
      Thunder_Motor.Set_R_Target(Line_H_Speed);

      if(line_state == 1)
      {
        Line_last_time = millis();
      }
      else if((current_time - 50) > Line_last_time)
      {
        line_state = 1;
      }
      else
      {
        line_state = 3;
      }
    }
    else  //Serial.printf("SSSSSSSSSS 线上 SSSSSSSSSS\n");
    {
      Thunder_Motor.Set_L_Target(Line_M_Speed);  //50ms 最大40  //编码器计数值
      Thunder_Motor.Set_R_Target(Line_M_Speed);
      line_state = 0;
      Line_last_time = millis();
    }

    //////////////////////////////////////////////// 巡线动作 /////////////////////////////////////////////////////////
    if((current_time - 50) > Line_last_led_time)
    {
      if((LED_counter > 98) & (line_state != 5))
      {
        LED_counter = 77;
      }
      else if((LED_counter > 100) & (line_state == 5))
      {
        LED_counter = 99;
        Line_last_sound_time = millis();
      }

      Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_counter);  //单色点阵图案
      LED_counter++;
      
      Line_last_led_time = millis();
    }

    if(((current_time - 19000) > Line_last_sound_time) & (current_time > 19000))
    {
      Speaker.Play_Song(139);
      Line_last_sound_time = millis();
    }
    
    En_Motor();
  }
}

// 开机动画/声效
void THUNDER::Start_Show(void)
{
  Speaker.Play_Song(79);  //声音编号79
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(1);  //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(2);  //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(3);  //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(4);  //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(5);  //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(4);  //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(5);  //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(4);  //单色点阵图案
  delay(200);
}

// 等待蓝牙连接动画 (有串口数据也跳出)
void THUNDER::Wait_BLE(void)
{
  while((deviceConnected == false) & (Usart_Communication == 0))
  {
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(6);   //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(7);   //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(8);   //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(9);   //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(10);  //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(11);  //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(12);  //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(13);  //单色点阵图案
    delay(200);

    if(Serial.available())
    {
      Usart_Communication = 1;
    }  
  }
}

// 设置将要播放的内置动画编号
void THUNDER::Set_LED_Show_No(uint8_t Show_No)
{
  LED_show_No = Show_No;
}

// 循环执行的内置动画控制程序
void THUNDER::LED_Show(void)
{
  switch(LED_show_No)
  {
    case 0:
      // Serial.printf("SSSSSSSSSS 无表情 SSSSSSSSSS\n");
      break;
    case 1:
      break;
    case 3:  //眨眼 8帧
      LED_delay_time = 200;
      if(LED_counter < 8)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_3[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 4:  //爱心  5帧
      LED_delay_time = 100;
      if(LED_counter < 5)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_4[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 5:  //眼镜  8帧
      if(LED_counter < 8)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_5[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_delay_time = LED_time_5[LED_counter];
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 6:  //严肃 4帧
      LED_delay_time = 200;
      if(LED_counter < 4)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_6[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 7:  //刹车
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(29);  //单色点阵图案
      break;
    case 8:  //流汗  3帧
      if(LED_counter < 3)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_8[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_delay_time = LED_time_8[LED_counter];
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 9:  //开心  5帧
      LED_delay_time = 200;
      if(LED_counter < 5)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_9[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 10:  //等待  3帧
      LED_delay_time = 1000;
      if(LED_counter < 3)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_10[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 11:  //眯眼 4帧
      LED_delay_time = 200;
      if(LED_counter < 4)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_11[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 12:  //启动 4帧
      LED_delay_time = 200;
      if(LED_counter < 4)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_12[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 13:  //凶  5帧
      LED_delay_time = 500;
      if(LED_counter < 5)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_13[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 14:  //哭泣  9帧
      LED_delay_time = 200;
      if(LED_counter < 9)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_14[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
    case 15:  //语音图  5帧
      LED_delay_time = 100;
      if(LED_counter < 5)
      {
        current_time = millis();
        if((current_time - last_led_time) > LED_delay_time)
        {
          Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_15[LED_counter]);  //单色点阵图案
          last_led_time = millis();
          LED_counter++;
        }
      }
      else
      {
        LED_show_No = 0;
        LED_counter = 0;
        last_led_time = 0;
        LED_delay_time = 0;
      }
      break;
  }
}

// 通信确认，蓝牙/串口
void THUNDER::Check_Communication(void)
{
  Wait_BLE();  //如果蓝牙没有连接，等待蓝牙连接 (有串口数据也跳出)

  if(Rx_Data[0] != 0)
  {
    Check_Protocol();
    if(Tx_Data[0] != 0)
    {
      Tx_Data[5] = Tx_Data[0] + Tx_Data[1] + Tx_Data[2] + Tx_Data[3] + Tx_Data[4];
      Thunder_BLE.Tx_BLE(Tx_Data, 6); //通过蓝牙发送数据;参数1 --> 数据数组；参数2 -->字节数
    }
  }
  else
  {
    while(Serial.available())
    {
      Rx_Data[0] = Serial.read(); //Serial.parseInt();  //读取整数
      
    #ifdef DEBUG_PRINT_INFO_COMMAND
      Serial.printf("SSS___ 收到串口指令: %x ___SSS\n",Rx_Data[0]);
    #endif
      //////////////////////////////////////// 其它特殊指令 //////////////////////////////////
      if(Rx_Data[0] == 0xC2)   //刷新左侧彩色灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 19; i++)
        {
          Thunder.I2C_LED_BUFF1[i-1] = Serial.read();
          SUM += Thunder.I2C_LED_BUFF1[i-1];
          // Serial.printf(": %x \n",Thunder.I2C_LED_BUFF1[i-1]);
        }
        
        if(SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\nSSS___ 0xC2 指令 校验和错误 ___SSS\n");
          Serial.printf("SSSSSSSSSS___ SUM: %x ___SSSSSSSSSS\n",SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xC2 Complete ___ SSS \n");
        }
      }
      else if(Rx_Data[0] == 0xC3)   //刷新右侧彩色灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 19; i++)
        {
          Thunder.I2C_LED_BUFF2[i-1] = Serial.read();
          SUM += Thunder.I2C_LED_BUFF2[i-1];
          // Serial.printf(": %x \n",Thunder.I2C_LED_BUFF2[i-1]);
        }

        if(SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\nSSS___ 0xC3 指令 校验和错误 ___SSS\n");
          Serial.printf("SSSSSSSSSS___ SUM: %x ___SSSSSSSSSS\n",SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xC3 Complete ___ SSS \n");
        }
      }
      else if(Rx_Data[0] == 0xD3)   //单色点阵灯一次性刷新前半部分灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 15; i++)
        {
          Thunder.LED_BUFF_Dot[i] = Serial.read();
          SUM += Thunder.LED_BUFF_Dot[i];
        }

        if(SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\nSSS___ 0xD3 指令 校验和错误 ___SSS\n");
          Serial.printf("SSSSSSSSSS___ SUM: %x ___SSSSSSSSSS\n",SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xD3 Complete ___ SSS \n");
        }
      }
      else if(Rx_Data[0] == 0xD4)   //单色点阵灯一次性刷新后半部分灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 15; i++)
        {
          Thunder.LED_BUFF_Dot[i+14] = Serial.read();
          SUM += Thunder.LED_BUFF_Dot[i+14];
        }

        if(SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\nSSS___ 0xD4 指令 校验和错误 ___SSS\n");
          Serial.printf("SSSSSSSSSS___ SUM: %x ___SSSSSSSSSS\n",SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xD4 Complete ___ SSS \n");
        }
      }
      else
      {
        Rx_Data[1] = Serial.read();
        Rx_Data[2] = Serial.read();
        Rx_Data[3] = Serial.read();
        Rx_Data[4] = Serial.read();
        Rx_Data[5] = Serial.read();

        if(Rx_Data[5] != (uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]))
        {
          Serial.printf("SSS __ Rx_Data[0]: %x 校验和错误 __ SSS \n",Rx_Data[0]);
          Serial.printf("SSSSSSSSSS__ SUM error __ Rx_Data[5]: %x __ sum: %x __SSSSSSSSSS\n",Rx_Data[5],(uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]));
          Reset_Rx_Data();
        }
      }
    }
    Check_Protocol();
    if(Tx_Data[0] != 0)
    {
      Tx_Data[5] = Tx_Data[0] + Tx_Data[1] + Tx_Data[2] + Tx_Data[3] + Tx_Data[4];

      //串口返回数据
      Serial.flush();
      Serial.write(Tx_Data[0]);
      Serial.write(Tx_Data[1]);
      Serial.write(Tx_Data[2]);
      Serial.write(Tx_Data[3]);
      Serial.write(Tx_Data[4]);
      Serial.write(Tx_Data[5]);
      Serial.flush();

      Tx_Data[0] = 0;
    }
  }
}

// 协议解析
void THUNDER::Check_Protocol(void)
{
  switch(Rx_Data[0])
  {
    case 0x00:
      // Serial.printf("SSSSSSSSSS 串口乱入 SSSSSSSSSS\n");
      break;

    case 0x51:    // IR数据读取
      Get_IR_Data(IR_Data);  //更新IR数据
      Tx_Data[0] = 0x51;
      Tx_Data[1] = IR_Data[0];
      Tx_Data[2] = IR_Data[1];
      Tx_Data[3] = 0x00;
      Tx_Data[4] = 0x00;
      // Serial.printf("SSSSSSSSSS ___ IR_Data[0] : %d ___ IR_Data[1] : %d ___ SSSSSSSSSS\n",IR_Data[0],IR_Data[1]);
      break;

    case 0x52:    // 获取US数据 16进制两个8位
      US.Get_US_Data(US_Data);  //更新US数据
      Tx_Data[0] = 0x52;
      Tx_Data[1] = US_Data[0];
      Tx_Data[2] = US_Data[1] >> 8;
      Tx_Data[3] = 0x00;
      Tx_Data[4] = 0x00;
      // Serial.printf("SSSSSSSSSS ___ US_Data[0] : %x ___ US_Data[1] : %x ___ SSSSSSSSSS\n",US_Data[0],US_Data[1]);
      break;

    case 0x53:    // 获取颜色传感器数据 RGBC分4个8位传
      Colour_Sensor.Get_RGBC_Data(RGBC);
      Colour_Sensor.RGBtoHSV(RGBC,HSV);   // 计算HSV

      Tx_Data[0] = 0x53;
      Tx_Data[1] = RGBC[0] >> 4;  //R >> 8
      Tx_Data[2] = RGBC[1] >> 4;  //G >> 8
      Tx_Data[3] = RGBC[2] >> 4;  //B >> 8
      // Tx_Data[4] = RGBC[3] >> 4;  //C >> 8
      Tx_Data[4] = (uint8_t)HSV[0];  //H

      // Serial.printf("SSSSSSSSSS R : %d ___ G : %d ___ B : %d ___ C : %d SSSSSSSSSS\n",RGBC[0],RGBC[1],RGBC[2],RGBC[3]);  //(RED)(GREEN)(BLUE)(CLEAR)
      // Serial.printf("SSSSSSSSSS H  %f ___ S : %f ___ V : %f ___ 颜色 : %d SSSSSSSSSS\n",HSV[0],HSV[1],HSV[2],Colour_Num);  //HSV
      break;

    case 0x54:    //获取电池电压数据
      ADC_Battery = analogRead(BATTERY_ADC_PIN);
      
      Tx_Data[0] = 0x54;
      Tx_Data[1] = ADC_Battery >> 8;    //读取一次电池电压  
      Tx_Data[2] = ADC_Battery;
      break;

    case 0x55:    //获取固件版本号
      Tx_Data[0] = 0x55;

      // 复制版本号的“整数”和“小数” 到 发送Buffer 
      Tx_Data[1] = Version_FW[0]; 
      Tx_Data[2] = Version_FW[1]; 
      Tx_Data[3] = 0x00;
      Tx_Data[4] = 0x00;

      break;

    case 0xA1:  //蓝牙命名  //仅限通过蓝牙，串口不支持重命名
        Thunder_BLE.Write_BLE_Name(0);  //从地址0开始写入命名的蓝牙
        break;

    case 0xA2:  //控制舵机
        Servo_Turn(Rx_Data[1], Rx_Data[2]); //参数1：1-->机械臂，2-->机械爪；参数2：角度[%](0~100)
        break;

    case 0xA3:  //控制声音播放
        Speaker.Play_Song(Rx_Data[1]);  //播放第编号段音频
        Speaker.Set_Sound_Volume(Rx_Data[2]);
        break;

    case 0xB1:  //控制单个电机
        Disable_En_Motor(); // En_Motor_Flag = 0;

        Thunder_Motor.Motor_Move(Rx_Data[1], Rx_Data[2], Rx_Data[3]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        break;

    case 0xB2:  //控制两个电机
        Disable_En_Motor(); // En_Motor_Flag = 0;

        Thunder_Motor.Motor_Move(1, Rx_Data[1], Rx_Data[2]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        Thunder_Motor.Motor_Move(2, Rx_Data[3], Rx_Data[4]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        break;

    case 0xB3:  //控制单个闭环电机
        Enable_En_Motor();  // En_Motor_Flag = 1;

        if(Rx_Data[1] == 1)
        {
          if(Rx_Data[3] == 1)
          {
            Thunder_Motor.Set_L_Target(Rx_Data[2]);
          }
          else
          {
            Thunder_Motor.Set_L_Target((-1) * Rx_Data[2]);
          }
        }
        else
        {
          if(Rx_Data[3] == 1)
          {
            Thunder_Motor.Set_R_Target(Rx_Data[2]);
          }
          else
          {
            Thunder_Motor.Set_R_Target((-1) * Rx_Data[2]);
          }
        }
        break;

    case 0xB4:  //控制两个闭环电机
        Enable_En_Motor();  // En_Motor_Flag = 1;

        if(Rx_Data[2] == 1)
        {
          Thunder_Motor.Set_L_Target(Rx_Data[1]);
        }
        else
        {
          Thunder_Motor.Set_L_Target((-1) * Rx_Data[1]);
        }

        if(Rx_Data[4] == 1)
        {
          Thunder_Motor.Set_R_Target(Rx_Data[3]);
        }
        else
        {
          Thunder_Motor.Set_R_Target((-1) * Rx_Data[3]);
        }
        break;

    case 0xB5:    //获取当前电机速度(编码器计数值)  需要在Enable_En_Motor()状态下
      L_Speed = Thunder_Motor.Get_L_Speed();
      R_Speed = Thunder_Motor.Get_R_Speed();

      Tx_Data[0] = 0xB5;
      if(L_Speed >= 0)
      {
        Tx_Data[1] = L_Speed;
        Tx_Data[2] = 1;
      }
      else
      {
        Tx_Data[1] = (-1) * L_Speed;
        Tx_Data[2] = 2;
      }

      if(R_Speed >= 0)
      {
        Tx_Data[3] = R_Speed;
        Tx_Data[4] = 1;
      }
      else
      {
        Tx_Data[3] = (-1) * R_Speed;
        Tx_Data[4] = 2;
      }
      // Serial.printf("L_Speed:%d; R_Speed:%d; \n",Thunder_Motor.Get_L_Speed(),Thunder_Motor.Get_R_Speed());
      break;

    case 0xC1:  //控制单个彩色灯颜色
        I2C_LED.Set_LED_Data(Rx_Data[1],Rx_Data[2],Rx_Data[3],Rx_Data[4]);   //第几个灯(1开始)，R,G,B
        I2C_LED.LED_Updata();  //按照现有数据刷新
        break;

    case 0xC2:  //控制左侧彩色灯颜色
        I2C_LED.Set_LEDs_Data(1, I2C_LED_BUFF1, sizeof(I2C_LED_BUFF1));   //写入多个寄存器数据
        I2C_LED.LED_Updata();  //按照现有数据刷新
        break;
    case 0xC3:  //控制右侧彩色灯颜色
        I2C_LED.Set_LEDs_Data(7, I2C_LED_BUFF2, sizeof(I2C_LED_BUFF2));   //写入多个寄存器数据
        I2C_LED.LED_Updata();  //按照现有数据刷新
        break;

    case 0xD1:  //控制单色点阵灯开关
        LED_BUFF_Dot[Rx_Data[1]] = Rx_Data[2];
        HT16D35B.LED_Show(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
        break;

    case 0xD2:  //显示预设的单色点阵灯图案
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(Rx_Data[1]);
        break;

    case 0xD3:  //单色点阵灯一次性刷新前半部分灯
        HT16D35B.LED_Show(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
        break;
    case 0xD4:  //单色点阵灯一次性刷新后半部分灯
        HT16D35B.LED_Show(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
        break;

    case 0xE1:  //内置单色点阵图案动画显示命令
        Set_LED_Show_No(Rx_Data[1]);
        break;

    case 0x66:  //_______________________ Demo ___________________
      if(Rx_Data[1] == 1)
      {
        // Serial.printf("SSSSSSSSSS___ 巡线 ___SSSSSSSSSS\n");
        Enable_En_Motor();  // En_Motor_Flag = 1;
        Line_Tracing(); 
      }
      Stop_All();
      break;

    default:
      Serial.printf("SSS___ 未定义指令 ___SSS\n");
      break;
  }
  Reset_Rx_Data();
}

// 清空接收数据
void THUNDER::Reset_Rx_Data()
{
    for (int i = 0; i < 6; i++)
    {
      Rx_Data[i] = 0;
    }
}

/* 
 * 开环控制电机时，获取编码器数据，用于测试电机编码器(一个PID控制周期才能获取一次)
 * 数据 通过队列发到打印线程
 *   打印线程在loop里面
 */
void THUNDER::Get_Queue_Encoder(void)
{
  if (xSemaphoreTake(Timer_PID_Flag, portMAX_DELAY ) == pdTRUE)  // 控制周期PID_dt[ms]
  {
    #ifdef PRINT_DEBUG_INFO
      float F_encoder_left;
      float F_encoder_right;

      // Serial.printf("left: %d\n", (int)Encoder_Counter_Left);
      F_encoder_left = Encoder_Counter_Left;
      xQueueSend(Task_Mesg.Queue_encoder_left, &F_encoder_left, 0);

      // Serial.printf("right: %d\n\n", (int)Encoder_Counter_Right);
      F_encoder_right = Encoder_Counter_Right;
      xQueueSend(Task_Mesg.Queue_encoder_right, &F_encoder_right, 0);
    #endif
  }
}

/* 
 * I2C端口选通，变量channelData 相应位(每一bit代表一个通道) 为1 是 选通，可以多通道选通
 * 
 * @parameter: 
 * @return: 返回的是IIC 操作状态码，0 为成功， 非0 为其他状态
 */
uint8_t THUNDER::Set_I2C_Chanel(uint8_t channelData)
{
  uint8_t ret;
  uint8_t regValue;

  // 重复连接 IIC 扩展芯片
  for(uint8_t i=0; i < 2; i++){
    // TCA9548的地址是 0x70, 因为它的地址位A0 A1 A2都接地了
    Wire.beginTransmission(0x70);
    Wire.write(channelData);
    ret = Wire.endTransmission(true);
    if(ret != 0){
      I2C_channel_opened = 0x00;
      Serial.printf("### TCA9548 Write I2C Channel Error: %d \n", ret);
      delay(100);
    }else{
      // read TCA9548
      Wire.requestFrom(0x70, 1, true);
      while(Wire.available()){
        regValue = Wire.read();
      }

      if(regValue == channelData){
        I2C_channel_opened = channelData;
        break;
      }else{
        Serial.println("### TCA9548 Read is not Write");
        delay(200);
      }
    }
  }

  return ret;
}

/*
 * 选择传感器通道1/2/3，选择后只有当前通道可使用，可用多个相同模块
 * 
 * @parameter：需要使用的传感器接口号
 * @return: 设置成功返回0，发生错误返回非0 的错误码
 *          错误码1：没有相应的传感器端口号
 *          错误码2：硬件不支持选择传感器端口号
 */
uint8_t THUNDER::Select_Sensor_Channel(uint8_t sensorChannel)
{
  uint8_t ret;

  switch(sensorChannel){
    case 1:
      ret = Set_I2C_Chanel(0x39);
      break;
    case 2:
      ret = Set_I2C_Chanel(0x3a);
      break;
    case 3:
      ret = Set_I2C_Chanel(0x3c);
      break;
    default:
      Serial.printf("### No Sensor Channel! ###");
      return 1;
      break;
  }

  if(ret != 0){
    return 2;
  }
  
  return 0;
}

/* 
 * 选通所有传感器通道，初始化时一定要调用后才能使用屏幕等等IIC接口的模块。
 *   调用后，不能使用多个相同模块。
 * 
 * @parameter:
 * @return: 0 表示操作成功， 1 为初始化 IIC 失败
 */
uint8_t THUNDER::Select_Sensor_AllChannel()
{
  uint8_t ret;

  // reset 
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(10);
  digitalWrite(15,HIGH);
  delay(10);

  ret = Set_I2C_Chanel(0x3f); //全选通

  if(ret != 0){
    return 1;
  }

  return 0;
}
