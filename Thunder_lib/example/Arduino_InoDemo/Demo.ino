#if 1
#include <bell_thunder.h>

#define PRINT_DEBUG_INFO
#define PRINT_DEBUG_ERROR

//Test数据
int ser_cmd;    //串口调试用

uint8_t LED_Counter = 1;

// 不断循环 查询通讯 与 控制电机 和 控制LED屏
void _loop()
{
  Thunder.Check_BLE_Communication();
  Thunder.Check_UART_Communication();
  Thunder.En_Motor();
  Thunder.LED_Show(); 
}

////////////////////////////////////////////// 单功能测试 ///////////////////////////////////////////////////////////
////////////////////////////////////////////// 舵机测试 ///////////////////////////////////////////////////////////
void Test_Servo()
{
//    float Battery_Voltage = 0;
    
//    Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//    Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//    int ser_cmd;
    if(Serial.available())
    {
      ser_cmd = Serial.read();
      Serial.printf("SSSSSSSSSS ___ %x ___ SSSSSSSSSS\n",ser_cmd);
      
      if(ser_cmd == 'a')
      {
        Thunder.Servo_Turn(2, 0);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 's')
      {
        Thunder.Servo_Turn(2, 30);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'd')
      {
        Thunder.Servo_Turn(2, 60);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'f')
      {
        Thunder.Servo_Turn(2, 90);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'g')
      {
        Thunder.Servo_Turn(2, 120);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'h')
      {
        Thunder.Servo_Turn(2, 150);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'j')
      {
        Thunder.Servo_Turn(2, 180);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }

      if(ser_cmd == 'q')
      {
        Thunder.Servo_Turn(1, 0);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'w')
      {
        Thunder.Servo_Turn(1, 65);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'e')
      {
        Thunder.Servo_Turn(1, 90);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 'r')
      {
        Thunder.Servo_Turn(1, 120);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
      else if(ser_cmd == 't')
      {
        Thunder.Servo_Turn(1, 180);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
      }
    }

//    Thunder.Servo_Turn(1, 100);  //爪 B
//    Thunder.Servo_Turn(2, 100);  //臂 A
//    delay(1000);
//    Thunder.Servo_Turn(1, 60);  //爪 B
//    Thunder.Servo_Turn(2, 60);  //臂 A
//    delay(1000);
//    Battery_Voltage = Thunder.Get_Battery_Data(); //读取一次电池电压
//    Serial.printf("SSSSSSSSSS Battery_Voltage : %.1f [V]SSSSSSSSSS\n",Battery_Voltage);
//    delay(2000);
//    Thunder.Servo_Turn(2, 130);
//    Thunder.Servo_Turn(1, 100);
//    Battery_Voltage = Thunder.Get_Battery_Data(); //读取一次电池电压
//    Serial.printf("SSSSSSSSSS Battery_Voltage : %.1f [V]SSSSSSSSSS\n",Battery_Voltage);
//    delay(2000);
//    Thunder.Servo_Turn(2, 170);
//    Thunder.Servo_Turn(1, 140);
//    Battery_Voltage = Thunder.Get_Battery_Data(); //读取一次电池电压
//    Serial.printf("SSSSSSSSSS Battery_Voltage : %.1f [V]SSSSSSSSSS\n",Battery_Voltage);
//    delay(2000);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 电机测试 ///////////////////////////////////////////////////////////
void Test_Open_Motor()
{
  if(Serial.available())
  {
    ser_cmd = Serial.read();

    if(ser_cmd == 'a')
    {
      Motor_Thunder.Motor_Move(1, 30, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 30, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: 30 ***");
    }
    else if(ser_cmd == 's')
    {
      Motor_Thunder.Motor_Move(1, 80, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 80, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: 80 ***");
    }
    else if(ser_cmd == 'd')
    {
      Motor_Thunder.Motor_Move(1, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: 150 ***");
    }
    else if(ser_cmd == 'f')
    {
      Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: 255 ***");
    }
    else if(ser_cmd == 'q')
    {
      Motor_Thunder.Motor_Move(1, 30, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 30, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: -30 ***");
    }
    else if(ser_cmd == 'w')
    {
      Motor_Thunder.Motor_Move(1, 80, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 80, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: -80 ***");
    }
    else if(ser_cmd == 'e')
    {
      Motor_Thunder.Motor_Move(1, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: -150 ***");
    }
    else if(ser_cmd == 'r')
    {
      Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: -255 ***");
    }
    else if(ser_cmd == 't')
    {
      Motor_Thunder.Motor_Move(1, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Serial.println("*** set motor PWM: 0 ***");
    }else if(ser_cmd == 'b')
    {
      Motor_Thunder.Motor_Brake(1);
      Motor_Thunder.Motor_Brake(2);
      Serial.println("*** motor brake ***");
    }
  }
}
//////////////////////////////////////////////// 电机测试结束 /////////////////////////////////////////////////////////

////////////////////////////////////////////// 编码电机测试 /////////////////////////////////////////////////////////// 
void Test_Close_Motor()
{
  if(Serial.available())
  {
    ser_cmd = Serial.read();

    if(ser_cmd == 'a')
    {
      Motor_Thunder.Set_L_Target(3);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(3);
      Serial.println("*** set motor: 3 ***");
    }
    else if(ser_cmd == 's')
    {
      Motor_Thunder.Set_L_Target(10);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(10);
      Serial.println("*** set motor: 10 ***");
    }
    else if(ser_cmd == 'd')
    {
      Motor_Thunder.Set_L_Target(20);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(20);
      Serial.println("*** set motor: 20 ***");
    }
    else if(ser_cmd == 'f')
    {
      Motor_Thunder.Set_L_Target(40);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(40);
      Serial.println("*** set motor: 40 ***");
    }
    else if(ser_cmd == 'q')
    {
      Motor_Thunder.Set_L_Target(-3);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(-3);
      Serial.println("*** set motor: -3 ***");
    }
    else if(ser_cmd == 'w')
    {
      Motor_Thunder.Set_L_Target(-8);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(-8);
      Serial.println("*** set motor: -8 ***");
    }
    else if(ser_cmd == 'e')
    {
      Motor_Thunder.Set_L_Target(-18);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(-18);
      Serial.println("*** set motor: -18 ***");
    }
    else if(ser_cmd == 'r')
    {
      Motor_Thunder.Set_L_Target(-28);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(-28);
      Serial.println("*** set motor: -28 ***");
    }
    else if(ser_cmd == 't')
    {
      Motor_Thunder.Set_L_Target(0);  //50ms 最大40  //编码器计数值
      Motor_Thunder.Set_R_Target(0);
      Serial.println("*** set motor: 0 ***");
    }
  }

  // Thunder.En_Motor();
}
//////////////////////////////////////////////// 编码电机测试结束 /////////////////////////////////////////////////////////

////////////////////////////////////////////// 量产编码器板测试软件 /////////////////////////////////////////////////////////// 
void Test_Encoder()
{
  pinMode(17,OUTPUT);
  pinMode(26,OUTPUT);
  
  if(Motor_Thunder.Get_L_Speed() > 0)
  {
    Serial.printf("红灯\n");
    digitalWrite(17, 1);
    digitalWrite(26, 0);
  }
  else  if(Motor_Thunder.Get_L_Speed() < 0)
  {
    Serial.printf("绿灯\n");
    digitalWrite(17, 0);
    digitalWrite(26, 1);
  }
  else
  {
    Serial.printf("关灯\n");
//    digitalWrite(25, 0);
//    digitalWrite(26, 0);
    digitalWrite(17, 1);
    digitalWrite(26, 1);
    delay(100);
  }

  // Thunder.En_Motor();
}
//////////////////////////////////////////////// 量产编码器板测试软件 /////////////////////////////////////////////////////////

////////////////////////////////////////////// 量产电机模块测试软件 /////////////////////////////////////////////////////////// 
void Test_Motor()
{
  static int motor_speed = 10;
  
  Motor_Thunder.Motor_Move(1, motor_speed, 1);
  Motor_Thunder.Motor_Move(2, motor_speed, 1);
  Serial.printf("%d\n", motor_speed);
  if(motor_speed >= 255){
    motor_speed = 10;
  }else motor_speed++;
  delay(200);
}
//////////////////////////////////////////////// 量产电机模块测试软件 /////////////////////////////////////////////////////////

////////////////////////////////////////////// 电池电压测试 ///////////////////////////////////////////////////////////
void Test_Battery()
{
  float Battery_Voltage = 0;
  
  Battery_Voltage = Thunder.Get_Battery_Data(); //读取一次电池电压
  Serial.printf("SSSSSSSSSS Battery_Voltage : %.1f [V]SSSSSSSSSS\n",Battery_Voltage);
  delay(1000);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 超声波测试 ///////////////////////////////////////////////////////////
void Test_US()
{
  //超声波数据
//  unsigned short US_Data[2] = {0,0};
  float US_Data_cm = 0;
  
//  Sensor_Ultrasonic.Get_US_Data(US_Data);  //更新US数据
//  Serial.printf("SSSSSSSSSS\nUS_Data[0] : %x\nUS_Data[1] : %x\nSSSSSSSSSS\n",US_Data[0],US_Data[1]);
//  Serial.printf("SSSSSSSSSS\nUS_Data[0]+US_Data[1] : %d\nSSSSSSSSSS\n",US_Data[0]+US_Data[1]);

  US_Data_cm = Sensor_Ultrasonic.Get_US_cm();
  if(US_Data_cm < 1.0){
    Serial.printf("### US_Data_cm : %.1f [cm]###############################\n",US_Data_cm);
  }else{
    Serial.printf("*** US_Data_cm : %.1f [cm]***\n",US_Data_cm);
  }
  

  delay(50);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 巡线传感器测试 ///////////////////////////////////////////////////////////
uint8_t IR_Data[2];
void Test_Line_Sensor()
{
  Thunder.Get_IR_Data(IR_Data);  //更新IR数据 //0-->白; 1-->黑
  
  Serial.printf("* IR_Data[0] : %d ___ IR_Data[1] : %d *\n",IR_Data[0],IR_Data[1]);
/* 
  Display_Screen.Play_LED_HT16F35B_Show(0);  //单色点阵图案 全亮

  if(IR_Data[0] == 1)
  {
    LED_Color.Set_LED_Data(1,5,5,5);   //第几个灯(1开始)，R,G,B
    delay(100);
    LED_Color.LED_Updata();  //按照现有数据刷新
    delay(100);
  }
  else 
  {
    LED_Color.Set_LED_Data(1,0,0,0);   //第几个灯(1开始)，R,G,B
    delay(100);
    LED_Color.LED_Updata();  //按照现有数据刷新
    delay(100);
  }

  if(IR_Data[1] == 1)
  {
    LED_Color.Set_LED_Data(7,5,5,5);   //第几个灯(1开始)，R,G,B
    delay(100);
    LED_Color.LED_Updata();  //按照现有数据刷新
    delay(100);
  }
  else 
  {
    LED_Color.Set_LED_Data(7,0,0,0);   //第几个灯(1开始)，R,G,B
    delay(100);
    LED_Color.LED_Updata();  //按照现有数据刷新
    delay(100);
  }
  delay(500); */
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 颜色识别测试 ///////////////////////////////////////////////////////////
void Test_Colour_Sensor()
{
  //颜色识别数据
  unsigned short RGBC[4] = {0};
  float HSV[3] = {0};
//  uint8_t Colour_Num = 0;

  if (Sensor_Color.Get_RGBC_Data(RGBC) != 0)
  {
    RGBC[0] = 0;
    RGBC[1] = 0;
    RGBC[2] = 0;
    RGBC[3] = 0;
    Serial.printf("### RGBC 获取失败 ########################\n");
  }
  else
  {
    // Colour_Num = Sensor_Color.Colour_Recognition(RGBC,HSV);  //识别颜色
    Sensor_Color.RGBtoHSV(RGBC,HSV);   // 计算HSV

    //calc RGB max & RGB Min
    uint16_t RGBmax, RGBmin;
    RGBmax = RGBC[0];
    RGBmin = RGBC[0];
    for(uint8_t i=1; i<3; i++){
      if(RGBC[i] > RGBmax)  RGBmax=RGBC[i];
      if(RGBC[i] < RGBmin)  RGBmin=RGBC[i];
    }
    #if 1
      #if 0
        Serial.printf("\n");
        Serial.printf("***** R : %4d   G : %4d B : %4d ***\n",RGBC[0], RGBC[1], RGBC[2]);  //(RED)(GREEN)(BLUE)(CLEAR)
        Serial.printf("*** Max : %4d \n*** Min : %4d \n", RGBmax, RGBmin);  //RGB max & RGB Min
        Serial.printf("***** H : %4d   C : %4d ***\n", (uint8_t)HSV[0], RGBC[3]);  //H、 C
      #else
        switch( Sensor_Color.Colour_Recognition(RGBC) ){
          case BLACK_CARD:
            Serial.println("*** black card");
          break;
          case WHITE_CARD:
            Serial.println("*** white card");
          break;
          case RED_CARD:
            Serial.println("*** red card");
          break;
          case BROWN_CARD:
            Serial.println("*** brown card");
          break;
          case YELLOW_CARD:
            Serial.println("*** yellow card");
          break;
          case GREEN_CARD:
            Serial.println("*** green card");
          break;
          case BLUE_CARD:
            Serial.println("*** blue card");
          break;
          case NO_CARD:
            Serial.println("*** no card");
          break;
          default:
            Serial.println("### bad return!");
          break;
        }
      #endif
    #else // 以下打印用于 serialPlot 打印曲线
      Serial.printf("%d %d %d ",RGBC[0], RGBC[1], RGBC[2]);  //(RED)(GREEN)(BLUE)(CLEAR)
      Serial.printf("%f %f ", HSV[1], Sensor_Color.env_backlight_c);  //RGB max & RGB Min
      Serial.printf("%d %d\n", (uint8_t)HSV[0], RGBC[3]);  //H、 C
    #endif
    // Serial.printf("SSSSSSSSSS H  %f ___ (uint8_t)H : %d SSSSSSSSSS\n",HSV[0],(uint8_t)HSV[0]);  //H
    // Serial.printf("SSSSSSSSSS H  %f ___ S : %f ___ V : %f ___ 颜色 : %d SSSSSSSSSS\n",HSV[0],HSV[1],HSV[2],Colour_Num);  //HSV
    // Serial.printf("SSSSSSSSSS R : %x ___ G : %x ___ B : %x ___ Colour_Num : %x SSSSSSSSSS\n",RGBC[0],RGBC[1],RGBC[2],Colour_Num);  //(RED)(GREEN)(BLUE)(CLEAR)
  }
  delay(200);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 声音测试 ///////////////////////////////////////////////////////////
void Test_Speaker()
{
  Speaker_Thunder.Set_Sound_Volume(15);

  for(int i=0; i<174; i++)
  {
    Speaker_Thunder.Play_Song(i);
    Serial.print("Song is : ");
    Serial.println(i); 
    delay(1000);
  }

    int busy_flag = Speaker_Thunder.WT588_Busy_Check(); // 0  播放中  1 停止
    if(busy_flag == 0)
    {
      Serial.printf("音乐播放中...\n");
    }
    else if(busy_flag == 1)
    {
      Serial.printf("无音乐播放\n");
    }
}

//////////////////////// I2C 彩色+单色点阵 坏灯测试 /////////////////////////
//uint8_t LED_Counter = 1;
//I2C彩灯
uint8_t LED_Data1[18] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };
uint8_t LED_Data2[18] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };

void Test_Display_Module()
{
//  LED_Color.LED_OFF();   //全关，立即刷新
//  delay(1000);

  LED_Color.Set_LEDs_Data(1, LED_Data1, sizeof(LED_Data1));   //写入多个寄存器数据
  delay(50);
  LED_Color.Set_LEDs_Data(7, LED_Data2, sizeof(LED_Data2));   //写入多个寄存器数据
  delay(50);
  LED_Color.LED_Updata();  //按照现有数据刷新
  delay(400);
  // 单色灯 
  Display_Screen.Play_LED_HT16F35B_Show(255);  //单色点阵图案 全亮
  delay(1000);

  // LED_Color.LED_OFF();   //全关，立即刷新
  delay(200);  
}
/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// 内置动画表情 ////////////////////////////
void Test_LED_Show()
{
  if(Serial.available())
  {
    ser_cmd = Serial.read();
    
    if(ser_cmd == 'a')
    {
      Thunder.Set_LED_Show_No(3);
    }
    else if(ser_cmd == 's')
    {
      Thunder.Set_LED_Show_No(4);
    }
    else if(ser_cmd == 'd')
    {
      Thunder.Set_LED_Show_No(5);
    }
    else if(ser_cmd == 'f')
    {
      Thunder.Set_LED_Show_No(8);
    }
    else if(ser_cmd == 'g')
    {
      Thunder.Set_LED_Show_No(9);
    }
    else if(ser_cmd == 'h')
    {
      Thunder.Set_LED_Show_No(10);
    }
    else if(ser_cmd == 'z')
    {
      Thunder.Set_LED_Show_No(12);
    }
    else if(ser_cmd == 'x')
    {
      Thunder.Set_LED_Show_No(13);
    }
    else if(ser_cmd == 'c')
    {
      Thunder.Set_LED_Show_No(14);
    }
    else if(ser_cmd == 'v')
    {
      Thunder.Set_LED_Show_No(15);
    }
  }
  _loop();
}

////////////////////////////////////////////// 巡线 ///////////////////////////////////////////////////////////
void Test_Line_Tracing()
{
    Thunder.Enable_En_Motor();  // En_Motor_Flag = 1;
    Rx_Data[1] = 1;
    Thunder.Line_Tracing();
}

////////////////////////////////////////////// 戴老师展示用车 ///////////////////////////////////////////////////////////
//uint8_t LED_Counter = 1;
////////////////////////////////////////////// 推土机 ///////////////////////////////////////////////////////////
void Demo_1()
{
      ///////////////// 前进 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(2);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(1);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(2);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(3);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(4);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(5);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(6);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(7);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(8);  //单色点阵图案
      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(3);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(9);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(10);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(11);  //单色点阵图案
      delay(200);

      ///////////////// 左 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(2);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(1);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(2);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(3);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(4);  //单色点阵图案
      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(5);  //单色点阵图案
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(6);  //单色点阵图案
//      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(3);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(9);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(10);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(11);  //单色点阵图案
      delay(200);

      ///////////////// 右 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(2);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(1);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(2);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(3);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(4);  //单色点阵图案
      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(5);  //单色点阵图案
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(6);  //单色点阵图案
//      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(3);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(9);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(10);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(11);  //单色点阵图案
      delay(200);
      
      ///////////////// 后退 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(4);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(17);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(18);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(19);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(20);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(21);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(22);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(23);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(24);  //单色点阵图案
      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(5);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(25);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(26);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(27);  //单色点阵图案
      delay(200);
      
      ///////////////// 右 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(2);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(1);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(2);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(3);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(4);  //单色点阵图案
      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(5);  //单色点阵图案
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(6);  //单色点阵图案
//      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(3);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(9);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(10);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(11);  //单色点阵图案
      delay(200);

      ///////////////// 左 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(2);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(1);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(2);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(3);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(4);  //单色点阵图案
      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(5);  //单色点阵图案
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(6);  //单色点阵图案
//      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

//      Colour_LED.Play_LED_XT1511_2427_Show(3);     //彩色点阵图案
      
      Display_Screen.Play_LED_HT16F35B_Show(9);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(10);  //单色点阵图案
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(11);  //单色点阵图案
      delay(200);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 机械手 ///////////////////////////////////////////////////////////
//LED_Counter = 1;
void Demo_2()
{
//      Colour_LED.Play_LED_XT1511_2427_Show(2);     //彩色点阵图案
      ///////////////// 前进 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 舵机 /////////////////
//      Thunder.Servo_Turn(1, 70);  //臂 A
      Thunder.Servo_Turn(2, 170);  //爪 B
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 左 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 舵机 /////////////////
      Thunder.Servo_Turn(1, 70);  //臂 A
      Thunder.Servo_Turn(2, 50);  //爪 B

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 右 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 舵机 /////////////////
      Thunder.Servo_Turn(1, 140);  //臂 A
//      Thunder.Servo_Turn(2, 130);  //爪 B


      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      
      ///////////////// 后退 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 停 /////////////////
      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 舵机 /////////////////
      Thunder.Servo_Turn(1, 100);  //臂 A
      Thunder.Servo_Turn(2, 130);  //爪 B

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

//      LED_Counter++;
//      if(LED_Counter > 100)
//      {
//        LED_Counter = 1;
//      }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 负载测试 ///////////////////////////////////////////////////////////
//LED_Counter = 1;
void Demo_3()
{
      ///////////////// 前进 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

//      ///////////////// 停 /////////////////
//      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//
//      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
//      LED_Counter++;
//      if(LED_Counter > 100)  LED_Counter = 1;
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
//      LED_Counter++;
//      if(LED_Counter > 100)  LED_Counter = 1;
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
//      LED_Counter++;
//      if(LED_Counter > 100)  LED_Counter = 1;
//      delay(200);

      ///////////////// 舵机 /////////////////
      Thunder.Servo_Turn(1, 120);  //臂 A
      Thunder.Servo_Turn(2, 120);  //爪 B
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

      ///////////////// 舵机 /////////////////
      Thunder.Servo_Turn(1, 65);  //臂 A
      Thunder.Servo_Turn(2, 120);  //爪 B

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      
      ///////////////// 后退 /////////////////
      Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

//      ///////////////// 停 /////////////////
//      Motor_Thunder.Motor_Move(1, 0, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//      Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//
//      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
//      LED_Counter++;
//      if(LED_Counter > 100)  LED_Counter = 1;
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
//      LED_Counter++;
//      if(LED_Counter > 100)  LED_Counter = 1;
//      delay(200);
//      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
//      LED_Counter++;
//      if(LED_Counter > 100)  LED_Counter = 1;
//      delay(200);

      ///////////////// 舵机 /////////////////
      Thunder.Servo_Turn(1, 90);  //臂 A
      Thunder.Servo_Turn(2, 180);  //爪 B

      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);
      Display_Screen.Play_LED_HT16F35B_Show(LED_Counter);  //单色点阵图案
      LED_Counter++;
      if(LED_Counter > 100)  LED_Counter = 1;
      delay(200);

//      LED_Counter++;
//      if(LED_Counter > 100)
//      {
//        LED_Counter = 1;
//      }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 超声波构型 ///////////////////////////////////////////////////////////
void Demo_US()
{
    //超声波数据
    float US_Data_cm = 0;
    US_Data_cm = Sensor_Ultrasonic.Get_US_cm();
    Serial.printf("SSSSSSSSSS US_Data_cm : %.1f [cm]SSSSSSSSSS\n",US_Data_cm);

    if(US_Data_cm != 0)
    {
      if(US_Data_cm < 20)
      {
        //暂停
        Motor_Thunder.Motor_Move(1, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        delay(50);
        //后退
//        Motor_Thunder.Motor_Move(1, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//        Motor_Thunder.Motor_Move(2, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

        Motor_Thunder.Motor_Move(1, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        Motor_Thunder.Motor_Move(2, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        delay(1000);
        //左转
//        Motor_Thunder.Motor_Move(1, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//        Motor_Thunder.Motor_Move(2, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

        Motor_Thunder.Motor_Move(1, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        Motor_Thunder.Motor_Move(2, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        delay(500);
      }
      else  
      { //前进
//        Motor_Thunder.Motor_Move(1, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
//        Motor_Thunder.Motor_Move(2, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        
        Motor_Thunder.Motor_Move(1, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        Motor_Thunder.Motor_Move(2, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
      }
    }

    delay(50);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/********************************Auto test process************************************/
#if 1

void Auto_Test_Motor()
{
  static uint8_t test_status = 0;

  if(test_status == 1)
  {
    Motor_Thunder.Set_L_Target(8);  //50ms 最大40  //编码器计数值
    Motor_Thunder.Set_R_Target(8);
    //Serial.println("*** set motor: 10 ***");
  }
  else if(test_status == 2)
  {
    Motor_Thunder.Set_L_Target(20);  //50ms 最大40  //编码器计数值
    Motor_Thunder.Set_R_Target(20);
    //Serial.println("*** set motor: 5 ***");
  }
  else if(test_status == 3)
  {
    Motor_Thunder.Set_L_Target(0);  //50ms 最大40  //编码器计数值
    Motor_Thunder.Set_R_Target(0);
    //Serial.println("*** set motor: 0 ***");
  }
  else if(test_status == 4)
  {
    Motor_Thunder.Set_L_Target(-20);  //50ms 最大40  //编码器计数值
    Motor_Thunder.Set_R_Target(-20);
    //Serial.println("*** set motor: 20 ***");
  }
  else if(test_status == 5)
  {
    Motor_Thunder.Set_L_Target(-8);  //50ms 最大40  //编码器计数值
    Motor_Thunder.Set_R_Target(-8);
    //Serial.println("*** set motor: -8 ***");
  }else{
    test_status = 0;
  }
  test_status++;

  //Thunder.En_Motor();
}

void Auto_Test_Led()
{
  static uint8_t test_status = 3;

  Thunder.Set_LED_Show_No(test_status);

  test_status++;
  if(test_status > 15){
    test_status = 3;
  }
  
}

void Auto_Test_US()
{
  //超声波数据
  float US_Data_cm = 0;

  US_Data_cm = Sensor_Ultrasonic.Get_US_cm();
  if(US_Data_cm < 1.0){
    #ifdef PRINT_DEBUG_ERROR
    Serial.printf("### US_Data_cm : %.1f [cm]###############################\n", US_Data_cm);
    #endif
  }else{
    #ifdef PRINT_DEBUG_INFO
    Serial.printf("* US_Data_cm : %.1f [cm]*\n",US_Data_cm);
    #endif
  }

}

void Auto_Test_Speaker()
{
  static uint8_t test_status = 169;

  int busy_flag = Speaker_Thunder.WT588_Busy_Check(); // 0  播放中  1 停止
  if(busy_flag == 0)
  {
    return; // 正在有音乐播放中，不进行新的播放
  }
  else if(busy_flag == 1)
  {
  }

  Speaker_Thunder.Set_Sound_Volume(15);
  Speaker_Thunder.Play_Song(test_status);

  // test_status++;
  // if(test_status > 174){
  //   test_status = 100;
  // }

}

/**
 * 
 */
void Auto_Test_Color()
{
  //颜色识别数据
  unsigned short RGBC[4] = {0};
  float HSV[3] = {0};
//  uint8_t Colour_Num = 0;

  if (Sensor_Color.Get_RGBC_Data(RGBC) != 0)
  {
    RGBC[0] = 0;
    RGBC[1] = 0;
    RGBC[2] = 0;
    RGBC[3] = 0;
    #ifdef PRINT_DEBUG_ERROR
    Serial.printf("### RGBC no data ###############################\n");
    #endif
  }
  else
  {
    Sensor_Color.RGBtoHSV(RGBC,HSV);   // 计算HSV

    #ifdef PRINT_DEBUG_INFO
    Serial.printf("* R: %d ~G: %d ~B: %d ~C: %d ~H: %f *\n",RGBC[0],RGBC[1],RGBC[2],RGBC[3],HSV[0]);  //(RED)(GREEN)(BLUE)(CLEAR)
    #endif

//  Serial.printf("*** H  %f ___ (uint8_t)H : %d ***\n",HSV[0],(uint8_t)HSV[0]);  //H
//    Serial.printf("SSSSSSSSSS H  %f ___ S : %f ___ V : %f ___ 颜色 : %d SSSSSSSSSS\n",HSV[0],HSV[1],HSV[2],Colour_Num);  //HSV
//    Serial.printf("SSSSSSSSSS R : %x ___ G : %x ___ B : %x ___ Colour_Num : %x SSSSSSSSSS\n",RGBC[0],RGBC[1],RGBC[2],Colour_Num);  //(RED)(GREEN)(BLUE)(CLEAR)
  }
}

void Auto_Test_LED_Color()
{
  static byte ledDataIndex = 0;

  byte LEDs_DataResult[36];
  byte colorData[36] = {0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8,
  0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8,0xC8,0x00,0xC8};
  
  for(byte i=0; i<sizeof(colorData); i++){
    LEDs_DataResult[i] = (byte)( (((float)colorData[i])/2) * (cos(ledDataIndex*2*PI/200-PI)+1) );
  }

  LED_Color.Set_LEDs_Data( 0x01, LEDs_DataResult, sizeof(LEDs_DataResult)/2 );
  LED_Color.Set_LEDs_Data( 0x07, LEDs_DataResult+18, sizeof(LEDs_DataResult)/2 );
  // delay(10);
  LED_Color.LED_Updata();

  if(ledDataIndex++ > 200) ledDataIndex = 0;
}

void Auto_Test_Touch()
{
  static byte ledIndex = 0;
  byte statusValue, errorCode;

  switch(ledIndex){
    case 0:
      Sensor_Touch.Set_LED_RGBvalue(10, 0, 0);
      // Serial.println("red LED");
      break;
    case 1:
      Sensor_Touch.Set_LED_RGBvalue(0, 10, 0);
      // Serial.println("green LED");
      break;
    case 2:
      Sensor_Touch.Set_LED_RGBvalue(0, 0, 10);
      // Serial.println("blue LED");
      break;
    case 5:
      Sensor_Touch.Reset_Mode();
    default:
      break;
  }

  // errorCode = Sensor_Touch.Get_Status(&statusValue);
  // if(errorCode != 0){
  //   Serial.println("### Touch read Error #################################");
  // }else Serial.printf("* Touch Status: %d \n", statusValue);

  if(Sensor_Touch.Check_Touch_Event(TOUCH_EVENT_RELEASE) == true){
    Serial.println("Touch Release");
  }
  if(Sensor_Touch.Check_Touch_Event(TOUCH_EVENT_PRESS) == true){
    Serial.println("Touch Press");
  }
  if(Sensor_Touch.Check_Touch_Event(TOUCH_EVENT_TOUCH) == true){
    Serial.println("Touch Happened");
  }
  Serial.println("");

  if(++ledIndex > 10) ledIndex = 0;
  // else if(ledIndex > 5) Sensor_Touch.Reset_Mode();
}

void Auto_Test_Light()
{
  static byte lightIndex = 0;
  float lightValue;
  byte errorCode;
  
  // if(lightIndex < 10){
  //     lightIndex++;
  //     Sensor_Light.Set_Led_Brightness(lightIndex*10);
  // }
  // else{
  //     lightIndex = 0;
  //     Sensor_Light.Set_Led_Brightness(0);
  // }
  // delay(30);

  Sensor_Light.Set_Led_Brightness(0);
  delay(500);
  Serial.println("=======50ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(50);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(50);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======40ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(40);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(40);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======30ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(30);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(30);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======20ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(20);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(20);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======15ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(15);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(15);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======10ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(10);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(10);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======5ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(5);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(5);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======4ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(4);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(4);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======3ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(3);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(3);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======2ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(2);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(2);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  delay(500);
  Serial.println("=======1ms========");
  Sensor_Light.Set_Led_Brightness(100);
  delay(1);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);
  Sensor_Light.Set_Led_Brightness(0);
  delay(1);
  lightValue = Sensor_Light.Get_Light_Value(0);
  Serial.printf("* light Value: %.2f \n", lightValue);

}

void Auto_Get_Motor_Speed_Value()
{
  int16_t leftSpeed, rightSpeed;

  leftSpeed = Motor_Thunder.Get_L_Speed();
  rightSpeed = Motor_Thunder.Get_R_Speed();

  Serial.printf("%d\t%d\n", leftSpeed, rightSpeed);
}

/* 
 * 测试电机编码器的数值与旋转角度的关系
 * 
 * @parameters: 
 * @return: 
 */
void Auto_Test_Encoder_RotateValue()
{
  int32_t Pre_encoderValue;
  int32_t Cur_encoderValue;
  int8_t flag = 1;

  // 关闭电机PID控制
  Thunder.Disable_En_Motor();
  while(1){
    Pre_encoderValue = Motor_Thunder.Get_L_RotateValue();
    Motor_Thunder.Set_L_Motor_Power(40 * flag);
    for(;;){
      Cur_encoderValue = Motor_Thunder.Get_L_RotateValue();
      if(abs(Cur_encoderValue - Pre_encoderValue) > 216){
        Motor_Thunder.Set_L_Motor_Power(0);
        break;
      }
    }
    delay(1000);
    Cur_encoderValue = Motor_Thunder.Get_L_RotateValue();
    Serial.printf("rotate: %d\n", abs(Cur_encoderValue - Pre_encoderValue));

    Pre_encoderValue = Motor_Thunder.Get_R_RotateValue();
    Motor_Thunder.Set_R_Motor_Power(40 * flag);
    for(;;){
      Cur_encoderValue = Motor_Thunder.Get_R_RotateValue();
      if(abs(Cur_encoderValue - Pre_encoderValue) > 216){
        Motor_Thunder.Set_R_Motor_Power(0);
        break;
      }
    }
    delay(1000);
    Cur_encoderValue = Motor_Thunder.Get_R_RotateValue();
    Serial.printf("rotate: %d\n\n", abs(Cur_encoderValue - Pre_encoderValue));

    if(flag == 1){
      flag = -1;
    }else{
      flag = 1;
    }
  }

}

void Auto_Test_Flame_Sensor()
{
  Serial.printf("Angle: %3d", Sensor_Flame.Get_Flame_Angle());
  Serial.printf(" Intens: %3d\n", Sensor_Flame.Get_Flame_Intensity());

  delay(100);
}

void Auto_Test_Fan_Motor()
{
  uint8_t speed = Motor_Fan.Get_Fan_Speed();

  speed += 20;
  if(speed > 100){
    speed = 0;
  }
  // speed = 100;
  Serial.println(speed);
  Motor_Fan.Set_Fan_Speed(speed);

  delay(1000);
}

void Wait_Command(byte byte_counter)
{
  int command_param_int[2];

  // uint32_t beginWaitTime;
  // beginWaitTime = millis();
  while( Serial.available() < byte_counter){
    // current_time = millis();
    // if(current_time > beginWaitTime + 5000){
    //   break;
    // }
  }
  for(byte i=0; i<2; i++){
    command_param_int[i] = Serial.parseInt();
  }

  // parse command
  // Sensor_Color.write(0x41, (byte *)(&command_param_int[0]), 1);
  // Sensor_Color.write(0x42, (byte *)(&command_param_int[1]), 1);
}

void Serial_Ble_Print_Speed()
{
  char speed_info[17];
  memset(speed_info, ' ', sizeof(speed_info));
  // sprintf(speed_info, "INFO: %3d ,%3d \n", Motor_Thunder.Get_L_Speed(), Motor_Thunder.Get_R_Speed());
  // sprintf(speed_info, "%7d,%7d\n", Motor_Thunder.Get_L_RotateValue(), Motor_Thunder.Get_R_RotateValue());
  int walk_diff;
  walk_diff = Motor_Thunder.Get_L_RotateValue()-Motor_Thunder.Get_R_RotateValue();
  assert( walk_diff == 0 );
  sprintf(speed_info, "DiffL-R:%7d\n", walk_diff);
  
  BLE_ThunderGo.Tx_BLE((uint8_t *)speed_info, sizeof(speed_info)-1);
}

#endif

#endif