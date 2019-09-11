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
// Task 1
void setup_1_1()
{
}
void loop_1_1()
{
    /* 使用蓝牙手柄 */
    if (BLE_Remoter.Check_Key_Action(KEY_Y, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_C5);
    }
    if (BLE_Remoter.Check_Key_Action(KEY_X, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_F5);
    }
    if (BLE_Remoter.Check_Key_Action(KEY_A, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
    }
    if (BLE_Remoter.Check_Key_Action(KEY_B, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_D5);
    }
    if (BLE_Remoter.Check_Key_Action(KEY_UP, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_G5);
    }
    if (BLE_Remoter.Check_Key_Action(KEY_RIGHT, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_A5);
    }
    if (BLE_Remoter.Check_Key_Action(KEY_DOWN, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_B5);
    }
    if (BLE_Remoter.Check_Key_Action(KEY_LEFT, KEY_PRESSING))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_C6);
    }

    if (BLE_Remoter.Check_Key_Action(KEY_Y, KEY_RELEASING) ||
        BLE_Remoter.Check_Key_Action(KEY_X, KEY_RELEASING) ||
        BLE_Remoter.Check_Key_Action(KEY_A, KEY_RELEASING) ||
        BLE_Remoter.Check_Key_Action(KEY_B, KEY_RELEASING) ||
        BLE_Remoter.Check_Key_Action(KEY_UP, KEY_RELEASING) ||
        BLE_Remoter.Check_Key_Action(KEY_DOWN, KEY_RELEASING) ||
        BLE_Remoter.Check_Key_Action(KEY_RIGHT, KEY_RELEASING) ||
        BLE_Remoter.Check_Key_Action(KEY_LEFT, KEY_RELEASING))
    {
        Speaker_Thunder.Set_Sound_Volume(0);
    }

    // Serial.printf("LX:%d LY:%d\n", BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X), BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y));
    Motor_Thunder.Set_Target(1, (BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y) + BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X)) * 0.4);
    Motor_Thunder.Set_Target(2, (BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y) - BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X)) * 0.4);

    /* 使用主控上的 功能按键 */
    if (Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_ONE))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MACHINE_SPORTCAR);
    }
    else if (Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_TWO))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MACHINE_SHIP);
    }
    else if (Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_THREE))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MACHINE_PLANE);
    }
    else if (Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_FOUR))
    {
        Speaker_Thunder.Set_Sound_Volume(100);
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MACHINE_MOTORBIKE);
    }
}
