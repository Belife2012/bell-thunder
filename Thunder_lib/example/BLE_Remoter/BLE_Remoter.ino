#include <bell_thunder.h>

void setup()
{
  // initial thunder-car all hareware resource
  Bell_Thunder.Setup_All();
  Bell_Thunder.Set_Ble_Type(BLE_TYPE_CLIENT);

  // 舵机位置初始化
  Bell_Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
  Bell_Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
}
void loop()
{
  Programs_System();
}

/* 
 * 在以下程序 Program_1 Program_2 Program_3 Program_4 Program_ThunderGo 里面开启任务线程
 * 使用函数 System_Task.Create_New_Loop 开启任务线程，每个程序中最多可以开启五个任务线程
 * System_Task.Create_New_Loop需要指定三个参数: program_sequence(程序号), 
 *                                          func_Program_Setup program_setup(setup函数入口), 
 *                                          func_Program_Loop program_loop(loop函数入口)
 * 
 * @parameters: 
 * @return: 
 */
void Program_1()
{
  // 创建用户程序1的线程，最多五个线程
  System_Task.Create_New_Loop(PROGRAM_USER_1, setup_1_1, loop_1_1);
}
void Program_2()
{
  // 创建用户程序2的线程，最多五个线程
  System_Task.Create_New_Loop(PROGRAM_USER_2, setup_2_1, loop_2_1);
}
void Program_3()
{
  // 创建用户程序3的线程，最多五个线程
  System_Task.Create_New_Loop(PROGRAM_USER_3, setup_3_1, loop_3_1);
}
void Program_4()
{
  // 创建用户程序4的线程，最多五个线程
  System_Task.Create_New_Loop(PROGRAM_USER_4, setup_4_1, loop_4_1);
}
void Program_ThunderGo()
{
  Bell_Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
  Bell_Thunder.Set_Need_Communication(true);
}
/*******setup函数 loop函数********/
void setup_1_1()
{
}
void loop_1_1()
{
  /* 使用蓝牙手柄 */
  if(BLE_Remoter.Check_Key_Action(KEY_Y, KEY_PRESSING)){
    Speaker_Thunder.Play_Song(1);
  }
  if(BLE_Remoter.Check_Key_Action(KEY_Y, KEY_RELEASING)){
    Speaker_Thunder.Play_Song(6);
  }
  if(BLE_Remoter.Check_Key_Action(KEY_X, KEY_PRESSING)){
    Speaker_Thunder.Play_Song(95);
  }
  if(BLE_Remoter.Check_Key_Action(KEY_X, KEY_RELEASING)){
    Speaker_Thunder.Play_Song(96);
  }
  if(BLE_Remoter.Check_Key_Action(KEY_A, KEY_PRESSING)){
    Speaker_Thunder.Play_Song(98);
  }
  if(BLE_Remoter.Check_Key_Action(KEY_A, KEY_RELEASING)){
    Speaker_Thunder.Play_Song(99);
  }
  if(BLE_Remoter.Check_Key_Action(KEY_B, KEY_PRESSING)){
    Speaker_Thunder.Play_Song(101);
  }
  if(BLE_Remoter.Check_Key_Action(KEY_B, KEY_RELEASING)){
    Speaker_Thunder.Play_Song(102);
  }

  if( BLE_Remoter.Check_Key(KEY_UP) ){
    Motor_Thunder.Set_L_Target(20);
    Motor_Thunder.Set_R_Target(20);
  }else if( BLE_Remoter.Check_Key(KEY_DOWN) ){
    Motor_Thunder.Set_L_Target(-20);
    Motor_Thunder.Set_R_Target(-20);
  }else if( BLE_Remoter.Check_Key(KEY_LEFT) ){
    Motor_Thunder.Set_L_Target(-20);
    Motor_Thunder.Set_R_Target(20);
  }else if( BLE_Remoter.Check_Key(KEY_RIGHT) ){
    Motor_Thunder.Set_L_Target(20);
    Motor_Thunder.Set_R_Target(-20);
  }else{
    Motor_Thunder.Set_L_Target(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y)-BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X));
    Motor_Thunder.Set_R_Target(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y)+BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X));
  }

  Bell_Thunder.Servo_Turn(2, 90+2*BLE_Remoter.Get_Control_Value(KEY_R2_ANALOG)/10);

  /* 使用功能按键 */
  if(Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_ONE)){
    Speaker_Thunder.Play_Song(112);
  }else if(Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_TWO)){
    Speaker_Thunder.Play_Song(113);
  }else if(Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_THREE)){
    Speaker_Thunder.Play_Song(114);
  }else if(Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_FOUR)){
    Speaker_Thunder.Play_Song(115);
  }
}
void setup_2_1()
{

}
void loop_2_1()
{
  /* 滚动显示字符串 */
  char alphabet_B[] = "2bellbell";
  Display_Screen.Play_LED_String(alphabet_B);
  delay(10000);
}
void setup_3_1()
{

}
void loop_3_1()
{
  /* 滚动显示字符串 */
  char alphabet_C[] = "abcdefghijklmnopqrstuvwxyz0123456789";
  Display_Screen.Play_LED_String(alphabet_C);
  delay(40000);
}
void setup_4_1()
{
  byte colorData[36] = {182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155,
                        182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155};
  LED_Color.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
  LED_Color.Set_LED_Dynamic(COLOR_MODE_ROLL);
}
void loop_4_1()
{
  /* 滚动显示字符串 */
  // char alphabet_D[] = "";
  // Display_Screen.Play_LED_String(alphabet_D);
  // delay(1000);
}
