#include <bell_thunder.h>

// #define DEBUG_VERSION
#define DEMO_VERSION
// #define THUNDER_GO_VERSION
// #define OLDLIB_THUNDER_GO_VERSION
// #define BURT_TEST_DEVICE_VERSION
// #define CALL_TEST_FUNCTION
// #define PRODUCE_FUNCTION

/********************************************Main****************************************************/
#ifdef DEBUG_VERSION

// #define NORMAL_MODE_TEST
#define TASK_MODE_TEST
// #define SINGLE_TEST

// #define PROCESS_CONTROL

void setup()
{
	Thunder.Setup_All();
	Thunder.Set_Ble_Type(BLE_TYPE_CLIENT);

	// 舵机位置初始化
	Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)

#ifdef TASK_MODE_TEST

	System_Task.Create_New_Loop(PROGRAM_THUNDER_GO, setup_1, loop_1);

	// 测试彩色灯条
	// byte colorData[36] = {182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155,
	//                       182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155};
	// LED_Color.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
	// LED_Color.Set_LED_Dynamic(COLOR_MODE_ROLL);

	// 九轴传感器初始化、校准
	// Attitude_Sensor.Init_Sensor();
	// // Attitude_Sensor.Calibrate_Sensor();
	// Attitude_Sensor.Open_Sensor();
	// Attitude_Sensor.SetShakeThreshold(100); // 设置震动灵敏度
	
	// Sensor_Sound.SetDetectRange(60, 0, 1);
	// Sensor_Infrared.SetSysMode(SYS_MODE_DISTANCE, 1);
#endif
}

/* 
 * main.cpp里面定义的loopTask, 执行了 setup 和 loop
 * xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
 *   loop是级别最低的task, uxPriority 为1
 */
void loop()
{
#ifdef TASK_MODE_TEST
	Display_Screen.Play_LED_String((uint32_t)Sensor_Ultrasonic.Get_US_cm(1));

	// 空气温湿度 
	// Serial.printf("\nHT: %f  ", Sensor_HumTemp.GetHumidity(1) );
	// Serial.printf("%f\n", Sensor_HumTemp.GetTemperature(1) );
	// 有毒气体传感器
	// Sensor_Gas.SetDetectRange(100, 0, 1);
	// Serial.printf("Toxicgas: %d\n", Sensor_Gas.GetToxicgasRatio(1));
	// 液体温度
	// Serial.printf("T: %f\n", Sensor_Temp.GetTemperature(1));
	// 土壤湿度
	// Sensor_Soil.SetDetectRange(80, 0, 1);
	// Serial.printf("SoilHum: %d\n", Sensor_Soil.GetHumidity(1));
	// 火焰传感器
	// Auto_Test_Flame_Sensor();
	// 风扇电动机
	// Auto_Test_Fan_Motor();
	// 声音传感器
	// Serial.printf("%d\n", Sensor_Sound.GetSoundDB(1));
	// 人体移动传感器
	// Serial.printf("Human %d\n", Sensor_Human.GetStatus(1));
	// 红外接收器
	// Serial.printf("Remote:%d ", Sensor_Infrared.GetRemoteInfo(1));
	// Serial.printf("Beacon0:%d ", Sensor_Infrared.GetBeaconDist(1));
	// Serial.printf("Beacon1:%d ", Sensor_Infrared.GetBeaconDire(1));
	// Serial.printf("Distance:%d\n", Sensor_Infrared.GetDistance(1));
	delay(100);


	// 测试读取九轴传感器数据
	// if(Attitude_Sensor.CheckShakeStatus()){
	// 	Serial.println("check shake!!!");
	// }
	// Attitude_Sensor.ShowAttitude();
	// delay(200);

	// Auto_Test_Flame_Sensor();
	// Auto_Test_Motor();
	// Auto_Test_US();
	// test_Us_QuickRead();
	// Auto_Test_LED_Color();
	// Auto_Test_Led();
	// Auto_Test_AllSensors();
	// Auto_Test_Touch();
	// delay(500);

	// Test_Line_Tracing();
	// Test_Colour_Sensor();

	// Auto_Get_Motor_Speed_Value();

	// char alphabet_B[] = "dengguihuan";
	// Display_Screen.Play_LED_String(alphabet_B);
	// delay(3000);

	// Serial_Ble_Print_Speed();
	// delay(500);
	
	// 多机通信
	// static int re_value;
	// static int recv_count = 0;
	// int recv_int_value;
	// recv_int_value = Thunder.RecvNameVarInt("MyIntValue");
	// if(re_value != recv_int_value){
	// 	recv_count++;
	// 	re_value = recv_int_value;
	// 	//Display_Screen.Play_LED_String(recv_count);
	// 	Serial.printf("recv_count: %d\n", recv_count);
	// }

	// Serial.printf("MyIntValue: %d\n", recv_int_value);
	// Serial.printf("YourIntValue: %d\n", Thunder.RecvNameVarInt("YourIntValue"));
	// delay(1);
#endif

#ifdef SINGLE_TEST
	// Test_Colour_Sensor();
	// Test_Servo();
	// Test_Open_Motor();
	// Test_Close_Motor();
	// Test_LED_Show();
	// Test_US();
	// Test_Speaker();
#endif

#ifdef NORMAL_MODE_TEST
	Thunder.Check_Communication();
#endif
}

#ifdef TASK_MODE_TEST
/*******new loop********/
void setup_1()
{
	Serial.println("System Start running...");

	// delay(500);
	// Thunder.Open_Multi_Message();
	// Thunder.InitNameVarInt("MyIntValue", 100000);
	// Thunder.InitNameVarInt("YourIntValue", 200000);

	delay(100);
}
int counterNum = 0;
int dirFlag = 0;
void loop_1()
{
	/* 使用多路同型号传感器 */
	// Thunder.Select_Sensor_Channel(1);
	// Serial.println("Port 1: ");
	// Auto_Test_US();
	// Thunder.Select_Sensor_Channel(2);
	// Serial.println("Port 2: ");
	// Auto_Test_US();

	/* 传感器测试 */
	// Test_Display_Module();
	// Auto_Test_Touch();
	// Auto_Test_Light();
	// Auto_Test_US();
	// Auto_Test_Color();
	// Test_Line_Sensor();
	// Auto_Test_Speaker();
	// Auto_Test_AllSensors();
	// delay(500);

	/* 显示电机旋转记录 */
	// Serial.printf("left rotate: %d\n", Motor_Thunder.Get_L_RotateValue());
	// Serial.printf("Right rotate: %d\n\n", Motor_Thunder.Get_R_RotateValue());

	/* 滚动显示字符串 */
	// char alphabet_A[] = "2a";
	// Display_Screen.Play_LED_String(alphabet_A);
	// delay(3000);
	// char alphabet_B[] = "qwe";
	// Display_Screen.Play_LED_String(alphabet_B);
	// delay(3000);
	// char alphabet_C[] = "abcdefghijklmnopqrstuvwxyz0123456789";
	// Display_Screen.Play_LED_String(alphabet_C);
	// delay(40000);
	// char alphabet_D[] = "";
	// Display_Screen.Play_LED_String(alphabet_D);
	// delay(1000);

	/* 测试闭环控制电机 */
	// delay(1000);
	// Motor_Thunder.motor_running_data.motor_select = 2;
	// Motor_Thunder.motor_running_data.running_mode = 2;
	// Motor_Thunder.motor_running_data.mode_data = 900;
	// Motor_Thunder.motor_running_data.right_motor_speed = 30;
	// Motor_Thunder.motor_running_data.left_motor_speed = 30;
	// Motor_Thunder.Control_Motor_Running(Motor_Thunder.motor_running_data);
	// Motor_Thunder.Motor_Brake(2);
	// delay(1000);
	// Motor_Thunder.motor_running_data.motor_select = 2;
	// Motor_Thunder.motor_running_data.running_mode = 2;
	// Motor_Thunder.motor_running_data.mode_data = 900;
	// Motor_Thunder.motor_running_data.right_motor_speed = -30;
	// Motor_Thunder.motor_running_data.left_motor_speed = -30;
	// Motor_Thunder.Control_Motor_Running(Motor_Thunder.motor_running_data);
	// Motor_Thunder.Motor_Brake(2);

	/* 测试开环电机 */
	// Test_Open_Motor();
	// Test_Motor();

	/* 测试多机通信 */
	#if 0
	// delay(5000);
	// Thunder.Close_Multi_Message();
	// Serial.println("Close_Multi_Message");
	// delay(5000);
	// Thunder.Open_Multi_Message();
	// Serial.println("Open_Multi_Message");
	#else 
	// static int value = 0;
	// value++;
	// Thunder.SendNameVarInt(0, "MyIntValue", value);
	// Thunder.SendNameVarInt(0, "YourIntValue", value*10);
	// // Thunder.SendNameVarInt(2, "MyIntValue", value);
	// // Thunder.SendNameVarInt(2, "YourIntValue", value*10);
	// if(value == 10000) while(1);
	#endif 
}

void Program_AutoCtrl()
{
}
void Program_1()
{}
void Program_2()
{}
void Program_3()
{}
void Program_4()
{}
#endif

#endif // #ifdef DEBUG_VERSION

/********************************************Main****************************************************/
#ifdef DEMO_VERSION

void setup()
{
	// initial System
	// thunder_system_parameter = 1;

	// initial thunder-car all hareware resource
	Thunder.Setup_All();
	Thunder.Set_Ble_Type(BLE_TYPE_CLIENT);

	// 舵机位置初始化
	Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)	
}
void loop()
{
	Programs_System();
	vTaskDelay(pdMS_TO_TICKS(10)); // 如果不增加vTaskDelay，此循环出现不工作的情况；
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
#ifdef COMPETITION_FW_001
void Program_AutoCtrl()
{
	// 创建赛前一分钟的程序
	Create_New_Loop_AutoCtrl(PROGRAM_USER_1, setup_AutoCtrl_1, loop_AutoCtrl_1);
}
#endif
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
	Thunder.Set_Ble_Type(BLE_TYPE_SERVER); // ThunderGo 模式，进入BLE Server
	Thunder.Set_Need_Communication(true);
}
/*******setup函数 loop函数********/
#ifdef COMPETITION_FW_001

void setup_AutoCtrl_1()
{

	Serial.printf("timer1: %d\n",Thunder.Get_Virtual_Timer(1));
	Thunder.Reset_Virtual_Timer(1);
	Serial.printf("light1: %.3f\n",Sensor_Light.Get_Light_Value(0));

	delay(3000);
	Sensor_Light.Set_Dark_Value(50);
	
	Serial.printf("timer1: %d\n",Thunder.Get_Virtual_Timer(1));
	Serial.printf("timer4: %d\n\n",Thunder.Get_Virtual_Timer(4));
	Serial.printf("light2: %.3f\n",Sensor_Light.Get_Light_Value(0));
}
void loop_AutoCtrl_1()
{
	/* 滚动显示字符串 */
	char alphabet_B[] = "AutoCtrl";
	Display_Screen.Play_LED_String(alphabet_B);
	delay(10000);
}

#endif
void setup_1_1()
{
	byte colorData[36] = {182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155,
	                      182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155};
	LED_Color.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
	LED_Color.Set_LED_Dynamic(COLOR_MODE_BLINK);
	Serial.printf("timer1: %d\n",Thunder.Get_Virtual_Timer(1));
}
void loop_1_1()
{
	/* 滚动显示字符串 */
	char alphabet_B[] = "1bell";
	Display_Screen.Play_LED_String(alphabet_B);
	delay(3000);
	LED_Color.Set_LED_Dynamic(COLOR_MODE_STATIC);
	delay(3000);
	LED_Color.Set_LED_Dynamic(COLOR_MODE_ROLL);
	delay(3000);
	LED_Color.Set_LED_Dynamic(COLOR_MODE_BREATH);
	delay(3000);
	LED_Color.Set_LED_Dynamic(COLOR_MODE_BLINK);
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
	Display_Screen.Play_LED_String((uint32_t)Sensor_Ultrasonic.Get_US_cm(1));
	delay(100);
}

#endif // #ifdef DEMO_VERSION

/********************************************Main****************************************************/
#ifdef THUNDER_GO_VERSION

void setup()
{
  // initial thunder-car all hareware resource
  Thunder.Setup_All();
  Thunder.Reset_Process_Status();

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
  Display_Screen.Play_LED_String(alphabet_B);
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

#endif // #ifdef THUNDER_GO_VERSION

/********************************************Main****************************************************/
#ifdef OLDLIB_THUNDER_GO_VERSION
void setup()
{
	// initial thunder-car all hareware resource
	Thunder.Setup_All();

	// 舵机位置初始化
	Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)

	System_Task.Create_New_Loop();
}
/* 
 * main.cpp里面定义的loopTask, 执行了 setup 和 loop
 * xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
 *   loop是级别最低的task, uxPriority 为1
 */
void loop()
{
}
/*******new loop********/

#endif // #ifdef OLDLIB_THUNDER_GO_VERSION

/********************************************Main****************************************************/
#ifdef BURT_TEST_DEVICE_VERSION

#define TASK_MODE_TEST
// #define SINGLE_TEST

void setup()
{
	// put your setup code here, to run once:
	Thunder.Setup_All();

	// 舵机位置初始化
	Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)

#ifdef TASK_MODE_TEST
	System_Task.Create_New_Loop();

	byte colorData[36] = {182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155,
												182, 180, 245, 132, 129, 239, 90, 86, 235, 44, 39, 228, 29, 24, 205, 22, 19, 155};
	LED_Color.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
#endif
}

/* 
 * main.cpp里面定义的loopTask, 执行了 setup 和 loop
 * xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
 *   loop是级别最低的task, uxPriority 为1
 */
void loop()
{

#ifdef TASK_MODE_TEST
	// Auto_Test_Motor();
	// Auto_Test_US();
	// test_Us_write();

	Auto_Test_LED_Color();
	// Auto_Test_Led();
	// Auto_Test_Touch();
	// Auto_Test_Light();

	delay(30);
#endif

#ifdef SINGLE_TEST
	// Test_Colour_Sensor();
	// Test_Servo();
	// Test_Open_Motor();
	// Test_Close_Motor();
	// Test_LED_Show();
	// Test_US();
	Test_Speaker();
#endif
}

#ifdef TASK_MODE_TEST

/*******new loop********/
void setup_1()
{
}
byte counterNum = 0;
void loop_1()
{
	// Thunder.Select_Sensor_Channel(1);
	// Serial.println("Port 1: ");
	// Auto_Test_US();
	// Thunder.Select_Sensor_Channel(2);
	// Serial.println("Port 2: ");
	// Auto_Test_US();

	// if(counterNum == 2){

	//   Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	//   Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	//   Thunder.Servo_Turn(1, 180);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	//   Thunder.Servo_Turn(2, 180);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	// }else if(counterNum == 3){

	//   Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	//   Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	//   Thunder.Servo_Turn(1, 0);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	//   Thunder.Servo_Turn(2, 0);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	//   counterNum = 0;
	// }
	// counterNum++;

	Auto_Test_US();
	Auto_Test_Color();
	Test_Line_Sensor();
	Auto_Test_Touch();
	Auto_Test_Light();

	Auto_Test_Speaker();
	Serial.println();

	// Serial.printf("left rotate: %d\n", Motor_Thunder.Get_L_RotateValue());
	// Serial.printf("Right rotate: %d\n\n", Motor_Thunder.Get_R_RotateValue());

	// for(byte colorDisplayIndex=1; colorDisplayIndex<4; colorDisplayIndex++){
	//   LED_Color.Set_LED_Dynamic(colorDisplayIndex);
	//   delay(6000);
	// }

	delay(1000);
}

#endif

#endif // #ifdef BURT_TEST_DEVICE_VERSION

/********************************************Main****************************************************/
#ifdef CALL_TEST_FUNCTION

void setup()
{
	// initial thunder-car all hareware resource
	Thunder.Setup_All();

	// 舵机位置初始化
	Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)

	// 开启一个新的 loop, 新的loop 使用 setup_1 和 loop_1
	System_Task.Create_New_Loop();
}
void loop()
{
	Wait_Command(7);
	// Auto_Test_Encoder_RotateValue();
}

/*******new loop********/
void setup_1()
{
}

byte lightIndex = 0;
void loop_1()
{
	Test_Colour_Sensor();

}

#endif // #ifdef CALL_TEST_FUNCTION

#ifdef PRODUCE_FUNCTION

void setup()
{
	// initial thunder-car all hareware resource

	pinMode(2, OUTPUT);
	pinMode(0, OUTPUT);
	pinMode(12, OUTPUT);

	digitalWrite(2, 1);
	digitalWrite(0, 1);
	digitalWrite(12, 1);

	Thunder.Setup_All();

	// 舵机位置初始化
	Thunder.Servo_Turn(1, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
	Thunder.Servo_Turn(2, 90); //参数1--> 舵机编号；参数2 --> 角度[%](0~180)

	// 开启一个新的 loop, 新的loop 使用 setup_1 和 loop_1
	// System_Task.Create_New_Loop();
}
void loop()
{
	// Produce_Color_Sensor();
	// Produce_Touch_Sensor();
	Produce_Light_Sensor();
}


#endif // #ifdef PRODUCE_FUNCTION
