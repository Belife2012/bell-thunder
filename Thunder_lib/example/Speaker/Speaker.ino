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
    Speaker_Thunder.Play_Song(SOUND_MUSIC_C4); delay(800);
    Speaker_Thunder.Play_Song(SOUND_MUSIC_D4); delay(800);
    Speaker_Thunder.Play_Song(SOUND_MUSIC_E4); delay(800);
    Speaker_Thunder.Play_Song(SOUND_MUSIC_F4); delay(800);
    Speaker_Thunder.Play_Song(SOUND_MUSIC_G4); delay(800);
    Speaker_Thunder.Play_Song(SOUND_MUSIC_A4); delay(800);
    Speaker_Thunder.Play_Song(SOUND_MUSIC_B4); delay(800);
    delay(2000);
}
void loop_1_1()
{
    // 
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(4, 300, 100);
    Play_OdeToJoy(5, 300, 100);
    delay(300);

    Play_OdeToJoy(5, 300, 100);
    Play_OdeToJoy(4, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    delay(300);

    Play_OdeToJoy(1, 300, 100);
    Play_OdeToJoy(1, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    delay(300);

    Play_OdeToJoy(3, 450, 100);
    Play_OdeToJoy(2, 150, 100);
    Play_OdeToJoy(2, 600, 100);
    delay(300);

    delay(300);
    //
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(4, 300, 100);
    Play_OdeToJoy(5, 300, 100);
    delay(300);

    Play_OdeToJoy(5, 300, 100);
    Play_OdeToJoy(4, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    delay(300);

    Play_OdeToJoy(1, 300, 100);
    Play_OdeToJoy(1, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    delay(300);

    Play_OdeToJoy(2, 450, 100);
    Play_OdeToJoy(1, 150, 100);
    Play_OdeToJoy(1, 600, 100);
    delay(300);

    delay(300);
    // 
    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(1, 300, 100);
    delay(300);

    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(3, 150, 100);
    Play_OdeToJoy(4, 150, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(1, 300, 100);
    delay(300);

    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(3, 150, 100);
    Play_OdeToJoy(4, 150, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    delay(300);

    Play_OdeToJoy(1, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(-5, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    delay(300);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(4, 300, 100);
    Play_OdeToJoy(5, 300, 100);
    delay(300);

    Play_OdeToJoy(5, 300, 100);
    Play_OdeToJoy(4, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    Play_OdeToJoy(4, 150, 100);
    Play_OdeToJoy(2, 150, 100);
    delay(300);

    Play_OdeToJoy(1, 300, 100);
    Play_OdeToJoy(1, 300, 100);
    Play_OdeToJoy(2, 300, 100);
    Play_OdeToJoy(3, 300, 100);
    delay(300);

    Play_OdeToJoy(2, 450, 100);
    Play_OdeToJoy(1, 150, 100);
    Play_OdeToJoy(1, 600, 100);
    delay(300);
    

    delay(2000);
}

void Play_OdeToJoy(int chart, int sustain, int volume)
{
    int sound;

    switch(chart) { //SOUND_MUSIC_ + CDEFGAB 为七音阶，1234567为高低音 
        case 1: sound = SOUND_MUSIC_C4; break;
        case 2: sound = SOUND_MUSIC_D4; break;
        case 3: sound = SOUND_MUSIC_E4; break;
        case 4: sound = SOUND_MUSIC_F4; break;
        case 5: sound = SOUND_MUSIC_G4; break;
        case 6: sound = SOUND_MUSIC_A4; break;
        case 7: sound = SOUND_MUSIC_B4; break;
        case -1: sound = SOUND_MUSIC_C3; break;
        case -2: sound = SOUND_MUSIC_D3; break;
        case -3: sound = SOUND_MUSIC_E3; break;
        case -4: sound = SOUND_MUSIC_F3; break;
        case -5: sound = SOUND_MUSIC_G3; break;
        case -6: sound = SOUND_MUSIC_A3; break;
        case -7: sound = SOUND_MUSIC_B3; break;
        default: break;
    }
    
    Speaker_Thunder.Set_Sound_Volume(volume);
    Speaker_Thunder.Play_Song(sound);
    delay(sustain);
}
