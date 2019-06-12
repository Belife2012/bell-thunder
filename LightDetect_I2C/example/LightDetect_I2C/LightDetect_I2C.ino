#include <Thunder_lib.h>

void setup() {
  // initial thunder-car all hareware resource
  Thunder.Setup_All();

  // 舵机位置初始化
  Thunder.Servo_Turn(1, 90);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
  Thunder.Servo_Turn(2, 90);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)

  // 开启一个新的 loop, 新的loop 使用 setup_1 和 loop_1
  Task_Mesg.Create_New_Loop();

  Task_Mesg.Set_Flush_Task(FLUSH_COMMUNICATIONS); // 把 查询蓝牙/串口数据指令 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_MATRIX_LED); // 把 LED点阵滚动显示字符串刷新工作 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_COLOR_LED); // 把 彩灯刷新工作 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_MOTOR_PID_CTRL); // 把 电机闭环控制 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_CHARACTER_ROLL); // 把 滚动显示的刷新工作 交给后台守护线程进行

  // 写入彩灯的显示数据
  byte colorData[36] = {182,180,245,132,129,239,90,86,235,44,39,228,29,24,205,22,19,155,
  182,180,245,132,129,239,90,86,235,44,39,228,29,24,205,22,19,155};
  I2C_LED.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
}

byte lightIndex = 0;
float lightValue;
byte errorCode;
byte ledIndex = 0;
byte statusValue;
void loop() {
  ///////////////////////////////////////////////
  switch(lightIndex){
    case 0:
      Light_Sensor.Set_Operate_Mode(0);
      break;
    case 1:
      Light_Sensor.Set_Operate_Mode(1);
      break;
    default:
      lightIndex = 0;
      break;
  }

  lightValue = Light_Sensor.Get_Light_Value(0);
  Serial.printf("*** light Value: %.2f \n", lightValue);

  lightIndex++;
  if(lightIndex > 1) lightIndex = 0;

  ///////////////////////////////////////////////
  switch(ledIndex){
    case 0:
      Touch_Sensor.Set_LED_RGBvalue(164, 26, 88);
      break;
    case 1:
      Touch_Sensor.Set_LED_RGBvalue(55, 134, 95);
      break;
    case 2:
      Touch_Sensor.Set_LED_RGBvalue(23, 15, 187);
      break;
    default:
      ledIndex = 0;
      break;
  }

  errorCode = Touch_Sensor.Get_Status(&statusValue);
  if(errorCode != 0){
    Serial.println("### Touch read Error #################################");
  }else Serial.printf("*** Touch Status: %d \n", statusValue);

  if(++ledIndex > 2) ledIndex = 0;

  ///////////////////////////////////////////////
  delay(200);
}

/***********************************new loop**************************************/
void setup_1()
{
  // 设置LED显示字符串，
  // 如果调用了 Task_Mesg.Set_Flush_Task(FLUSH_COMMUNICATIONS);效果需要在连接上蓝牙后才有
  char alphabet_C[] = "abcdefghijklmnopqrstuvwxyz0123456789";
  Dot_Matrix_LED.Play_LED_String(alphabet_C);
}
void loop_1()
{
  // 每6秒 切换一种彩灯的动态显示效果
  // 如果调用了 Task_Mesg.Set_Flush_Task(FLUSH_COMMUNICATIONS);效果需要在连接上蓝牙后才有
  for(byte colorDisplayIndex = 0; colorDisplayIndex < 4; colorDisplayIndex++){
    I2C_LED.Set_LED_Dynamic(colorDisplayIndex);
    delay(6000);
  }
}
