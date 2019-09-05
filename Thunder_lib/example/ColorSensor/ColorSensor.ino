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
 * Program_2、Program_3、Program_4
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
void setup_1_1()
{
    Serial.println("Sensor of Color...");
}
void loop_1_1()
{
    int value;

    value = Sensor_Color.Get_Color_Result(1);
    switch(value)
    {
        case CARD_NO: Speaker_Thunder.Play_Song(SOUND_MUSIC_C0);break;
        case CARD_RED: Speaker_Thunder.Play_Song(SOUND_MUSIC_C1);break;
        case CARD_BROWN: Speaker_Thunder.Play_Song(SOUND_MUSIC_C2);break;
        case CARD_YELLOW: Speaker_Thunder.Play_Song(SOUND_MUSIC_C3);break;
        case CARD_GREEN: Speaker_Thunder.Play_Song(SOUND_MUSIC_C4);break;
        case CARD_BLUE: Speaker_Thunder.Play_Song(SOUND_MUSIC_C5);break;
        case CARD_WHITE: Speaker_Thunder.Play_Song(SOUND_MUSIC_C6);break;
        case CARD_BLACK: Speaker_Thunder.Play_Song(SOUND_MUSIC_C7);break;
        default: break;
    }

    delay(1000);
}
