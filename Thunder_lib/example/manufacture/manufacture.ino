#include <bell_thunder.h>


/*************************************************************
 * @brief: 
 * 出厂内置程序，设备启动按一下或者两下，都是进入到ThunderGO模式，
 * 可以使用ThunderGo APP进行遥控，开机立刻可以玩耍。
 *************************************************************/

void setup()
{
	// initial thunder-car all hareware resource
	Bell_Thunder.Setup_All();
	Bell_Thunder.Reset_Process_Status();

	// 舵机位置初始化
	Motor_Servo.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	Motor_Servo.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
}
void loop()
{
	Programs_System();
	vTaskDelay(pdMS_TO_TICKS(10)); // 如果不增加vTaskDelay，此循环出现不工作的情况；
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
}
#endif

void Program_1()
{
	Bell_Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
	Bell_Thunder.Set_Need_Communication(true);
}
void Program_2()
{
}
void Program_3()
{
}
void Program_4()
{
}

void Program_ThunderGo()
{
	Bell_Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
	Bell_Thunder.Set_Need_Communication(true);
}
