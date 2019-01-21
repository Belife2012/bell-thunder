#include <Thunder_lib.h>
#include <Task_Mesg.h>

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

/* 
 * 在以下程序 Program_1 Program_2 Program_3 Program_4 Program_ThunderGo 里面开启任务线程
 * 使用函数 Task_Mesg.Create_New_Loop 开启任务线程，每个程序中最多可以开启五个任务线程
 * Task_Mesg.Create_New_Loop需要指定三个参数: program_sequence(程序号), 
 *                                          func_Program_Setup program_setup(setup函数入口), 
 *                                          func_Program_Loop program_loop(loop函数入口)
 * 
 * @parameters: 
 * @return: 
 */
void Program_1()
{
  // 创建用户程序1的线程，最多五个线程
  Task_Mesg.Create_New_Loop(PROGRAM_USER_1, setup_1_1, loop_1_1);
}
void Program_2()
{
  // 创建用户程序2的线程，最多五个线程
  Task_Mesg.Create_New_Loop(PROGRAM_USER_2, setup_2_1, loop_2_1);
}
void Program_3()
{
  // 创建用户程序3的线程，最多五个线程
  Task_Mesg.Create_New_Loop(PROGRAM_USER_3, setup_3_1, loop_3_1);
}
void Program_4()
{
  // 创建用户程序4的线程，最多五个线程
  Task_Mesg.Create_New_Loop(PROGRAM_USER_4, setup_4_1, loop_4_1);
}
void Program_ThunderGo()
{
  Thunder.Set_Need_Communication(true);
}
/*******setup函数 loop函数********/
void setup_1_1()
{

}
void loop_1_1()
{
  /* 滚动显示字符串 */
  char alphabet_A[] = "1bell";
  Dot_Matrix_LED.Play_LED_String(alphabet_A);
  delay(10000);
}
void setup_2_1()
{

}
void loop_2_1()
{
  /* 滚动显示字符串 */
  char alphabet_B[] = "2bellbell";
  Dot_Matrix_LED.Play_LED_String(alphabet_B);
  delay(10000);
}
void setup_3_1()
{

}
void loop_3_1()
{
  /* 滚动显示字符串 */
  char alphabet_C[] = "abcdefghijklmnopqrstuvwxyz0123456789";
  Dot_Matrix_LED.Play_LED_String(alphabet_C);
  delay(40000);
}
void setup_4_1()
{
  byte colorData[36] = {182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155,
                        182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155};
  I2C_LED.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
  I2C_LED.Set_LED_Dynamic(COLOR_MODE_ROLL);
}
void loop_4_1()
{
  /* 滚动显示字符串 */
  // char alphabet_D[] = "";
  // Dot_Matrix_LED.Play_LED_String(alphabet_D);
  // delay(1000);
}
