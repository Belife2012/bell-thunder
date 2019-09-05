#include <bell_thunder.h>

/*************************************************************
 * @brief: thunder系统相关配置
 *************************************************************/
void setup()
{
    Bell_Thunder.Setup_All();
    Bell_Thunder.Set_Ble_Type(BLE_TYPE_CLIENT);
    Motor_Servo.Servo_Turn(1, 90);
    Motor_Servo.Servo_Turn(2, 90);
}
void loop()
{
    Programs_System();
    vTaskDelay(pdMS_TO_TICKS(10));
}

// 系统参数thunder_system_parameter 为赛事时，启动Program_AutoCtrl，此环节蓝牙遥控器不能遥控
void Program_AutoCtrl() {}
// ThunderGo依赖于此程序，开机后切换为ThunderGo模式时，执行该程序
void Program_ThunderGo()
{
    Bell_Thunder.Set_Ble_Type(BLE_TYPE_SERVER);
    Bell_Thunder.Set_Need_Communication(true);
}

/*************************************************************
 * @brief: thunder用户程序的启动代码，只在用户程序启动时执行一次
 * 一般在此过程创建用户线程，用户程序最多4个，分别为Program_1、
 * Program_2、Program_3、Program_4;
 * 
 * 在以下程序 Program_1 Program_2 Program_3 Program_4 里面开启任务线程
 * 使用函数 System_Task.Create_New_Loop 开启任务线程，每个程序中最多可以开启32个任务线程
 * System_Task.Create_New_Loop需要指定三个参数: 
 *                                          program_sequence(程序号), 
 *                                          func_Program_Setup program_setup(setup函数入口), 
 *                                          func_Program_Loop program_loop(loop函数入口)
 *
 *************************************************************/
void Program_1()
{
    System_Task.Create_New_Loop(PROGRAM_USER_1, setup_1_1, loop_1_1);
}
void Program_2()
{
    System_Task.Create_New_Loop(PROGRAM_USER_2, setup_2_1, loop_2_1);
}
void Program_3()
{
    System_Task.Create_New_Loop(PROGRAM_USER_3, setup_3_1, loop_3_1);
}
void Program_4()
{
    System_Task.Create_New_Loop(PROGRAM_USER_4, setup_4_1, loop_4_1);
}


/*************************************************************
 * @brief: thunder用户线程
 *************************************************************/
// 用户自主的程序，下面示例编写 4 个程序

// 1 号程序
void setup_1_1()
{
    Display_Screen.Play_LED_String(1);
}
void loop_1_1()
{
    Display_Screen.Move_Picture_To(0, 1);
    delay(100);
    Display_Screen.Move_Picture_To(0, -1);
    delay(100);
    Display_Screen.Move_Picture_To(0, 1);
    delay(100);
    Display_Screen.Move_Picture_To(0, -1);
    delay(1000);
}

// 2 号程序
void setup_2_1()
{
    Display_Screen.Play_LED_String(2);
}
void loop_2_1()
{
    Display_Screen.Move_Picture_To(1, 0);
    delay(100);
    Display_Screen.Move_Picture_To(-1, 0);
    delay(100);
    Display_Screen.Move_Picture_To(1, 0);
    delay(100);
    Display_Screen.Move_Picture_To(-1, 0);
    delay(1000);
}

// 3 号程序
void setup_3_1()
{
    Display_Screen.Play_LED_String(3);
}
void loop_3_1()
{
    Display_Screen.Move_Picture_To(0, 1);
    delay(100);
    Display_Screen.Move_Picture_To(0, -1);
    delay(100);
    Display_Screen.Move_Picture_To(1, 0);
    delay(100);
    Display_Screen.Move_Picture_To(-1, 0);
    delay(1000);
}

// 4 号程序
void setup_4_1()
{
    Display_Screen.Play_LED_String(4);
}
void loop_4_1()
{
    Display_Screen.Move_Picture_To(1, 1);
    delay(100);
    Display_Screen.Move_Picture_To(-1, -1);
    delay(100);
    Display_Screen.Move_Picture_To(1, 1);
    delay(100);
    Display_Screen.Move_Picture_To(-1, -1);
    delay(1000);
}
