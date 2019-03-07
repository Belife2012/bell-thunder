#include <Thunder_lib.h>

void setup()
{
  // initial thunder-car all hareware resource
  Thunder.Setup_All();

  // 舵机位置初始化
  Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
  Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
}
void loop()
{
  Programs_System();
}

#ifdef COMPETITION_FW_001

void Program_AutoCtrl()
{
  // 创建赛前一分钟的程序
  Create_New_Loop_AutoCtrl(PROGRAM_USER_1, setup_AutoCtrl_1, loop_AutoCtrl_1);
}
void setup_AutoCtrl_1()
{
}
void loop_AutoCtrl_1()
{
  /* 滚动显示字符串 */
  char alphabet_B[] = "AutoCtrl";
  Dot_Matrix_LED.Play_LED_String(alphabet_B);
  delay(10000);
}

#endif
void Program_1()
{
  Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
  Thunder.Set_Need_Communication(true);
}
void Program_2()
{}
void Program_3()
{}
void Program_4()
{}

void Program_ThunderGo()
{
  Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
  Thunder.Set_Need_Communication(true);
}
