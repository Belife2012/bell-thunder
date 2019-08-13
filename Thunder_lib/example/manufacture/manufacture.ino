#include <bell_thunder.h>

void setup()
{
  // initial thunder-car all hareware resource
  Bell_Thunder.Setup_All();

  // 舵机位置初始化
  Motor_Servo.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
  Motor_Servo.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
}
void loop()
{
  Programs_System();
}

#ifdef COMPETITION_FW_001

void Program_AutoCtrl()
{}

#endif
void Program_1()
{}
void Program_2()
{}
void Program_3()
{}
void Program_4()
{}

void Program_ThunderGo()
{
  Bell_Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
  Bell_Thunder.Set_Need_Communication(true);
}
