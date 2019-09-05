#include <bell_thunder.h>

/*************************************************************
 * @brief: thunder系统相关配置
 *************************************************************/
void setup()
{
    // initial System
    thunder_system_parameter = 1; // 开启赛事模式

    // initial thunder-car all hareware resource
    Bell_Thunder.Setup_All();
    Bell_Thunder.Set_Ble_Type(BLE_TYPE_CLIENT);

    // 舵机位置初始化
    Motor_Servo.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
    Motor_Servo.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
}
void loop()
{
    Programs_System();
    vTaskDelay(pdMS_TO_TICKS(10));
}

#ifdef COMPETITION_FW_001
void Program_AutoCtrl()
{
    // 使用Create_New_Loop_AutoCtrl创建赛前一分钟的任务
    Create_New_Loop_AutoCtrl(PROGRAM_USER_1, setup_AutoCtrl_1, loop_AutoCtrl_1);
}
#endif
void Program_ThunderGo()
{
    Bell_Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
    Bell_Thunder.Set_Need_Communication(true);
}

/*************************************************************
 * @brief: thunder用户程序的启动代码，赛事程序只能启动 Program_1 程序
 *************************************************************/
void Program_1()
{
    System_Task.Create_New_Loop(PROGRAM_USER_1, setup_1_1, loop_1_1);
}
void Program_2() {}
void Program_3() {}
void Program_4() {}

/*************************************************************
 * @brief: thunder用户线程
 *************************************************************/

// 编写赛前一分钟的任务
#ifdef COMPETITION_FW_001
void setup_AutoCtrl_1()
{
}
void loop_AutoCtrl_1()
{
    /* 滚动显示字符串 */
    char alphabet_B[] = "AutoCtrl";
    Display_Screen.Play_LED_String(alphabet_B);
    delay(10000);
}
#endif

// 用户自主的程序，下面示例编写 4 个程序

// 1 号程序
void setup_1_1()
{
    Display_Screen.Play_LED_String("RUN");
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
