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
    System_Task.Create_New_Loop(PROGRAM_USER_1, setup_1_2, loop_1_2);
}
void Program_2() {}
void Program_3() {}
void Program_4() {}

/*************************************************************
 * @brief: thunder用户线程
 *************************************************************/
// Task 1
void setup_1_1()
{
    // 写入彩灯的显示数据
    LED_COLOR::t_color_led_buff colorData = {{182, 180, 245}, {132, 129, 239}, {90, 86, 235}, {44, 39, 228}, {29, 24, 205}, {22, 19, 155}, 
                                    {182, 180, 245}, {132, 129, 239}, {90, 86, 235}, {44, 39, 228}, {29, 24, 205}, {22, 19, 155}};
    LED_Color.Set_LEDs_Data(colorData);
    LED_Color.Set_LED_Dynamic(LED_COLOR::COLOR_MODE_BREATH);
}
void loop_1_1()
{
    Display_Screen.Play_Animation(5);

    delay(6000);
}

// Task 2
void setup_1_2()
{
    Sensor_Light.Set_Operate_Mode(100, 1);
    Sensor_Light.Set_Extremum(0, 25, 1); // 设置最大值
}

#define MOTOR_MAX   40
void loop_1_2()
{
    float light_value;

    light_value = Sensor_Light.Get_Light_Value(1);
    Serial.printf("light: %f\n", light_value);

    Motor_Thunder.Set_Motor_Power(1, MOTOR_MAX * light_value / 100);
    Motor_Thunder.Set_Motor_Power(2, MOTOR_MAX - (MOTOR_MAX * light_value / 100));

    delay(10);
}
