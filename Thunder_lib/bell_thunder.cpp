#include <bell_thunder.h>
#include "esp_adc_cal.h"
#include "function_macro.h"
#include "disk_manager.h"
#include "common.h"

BELL_THUNDER Bell_Thunder;
int BELL_THUNDER::program_change_to = PROGRAM_RUNNING;

//ADC 校准使用
esp_adc_cal_characteristics_t adc_chars;
esp_adc_cal_value_t cal_value_type;

// IIC选通芯片TCA9548的地址是（IIC address：0x70）

MOTOR_THUNDER Motor_Thunder;
// 风扇电动机(IIC address：0x03)
MOTOR_FAN Motor_Fan(FAN_IIC_ADDR);
// 炮台模组 （IIC address：0x24）
BELL_BARBETTE Barbette_Thunder;
// 彩色LED（IIC address：0x11）
LED_COLOR LED_Color(CLOOUR_LED_DEVICE);
// 单色色LED（IIC address：0x69）
DISPLAY_SCREEN Display_Screen;
// 音频
SPEAKER_THUNDER Speaker_Thunder(AUDIO, BUSY); // 配置引脚16 --> AUDIO 和 4 --> BUSY
MOTOR_SERVO Motor_Servo;

// 姿态传感器（IIC address: 0x68 0x0C）
// 超声波（IIC address：0x01）
SENSOR_US Sensor_Ultrasonic(US_IIC_ADDR);
// 颜色识别（IIC address：0x38）
SENSOR_COLOR Sensor_Color(COLOR_IIC_ADDR); //I2C从机地址
// 触碰传感器（IIC address：0x10）
SENSOR_TOUCH Sensor_Touch(TOUCH_IIC_ADDR);
// 光电传感器（IIC address：0x52）
SENSOR_LIGHT Sensor_Light(LIGHT_IIC_ADDR);
SENSOR_COLORLIGHT Sensor_ColorLight(COLORLIGHT_IIC_ADDR);
// 火焰传感器（IIC address：0x02）
SENSOR_FLAME Sensor_Flame(FLAME_IIC_ADDR);
// 空气温湿度 （IIC address：0x05）
SENSOR_HT Sensor_HumTemp(HT_IIC_ADDR);
// 有毒气体传感器 （IIC address：0x06）
SENSOR_GAS Sensor_Gas(GAS_IIC_ADDR);
// 温度探头 （IIC address：0x07）
SENSOR_TEMP Sensor_Temp(TEMP_IIC_ADDR);
// 土壤湿度传感器（IIC address：0x08）
SENSOR_SOIL Sensor_Soil(SOIL_IIC_ADDR);
// 声音传感器（IIC address：0x09）
SENSOR_SOUND Sensor_Sound(SOUND_IIC_ADDR);
// 人体移动传感器（IIC address：0x0A）
SENSOR_HUMAN Sensor_Human(HUMAN_IIC_ADDR);
// 红外接收模块（IIC address：0x04）
SENSOR_INFRARED Sensor_Infrared(INFRARED_IIC_ADDR);

BLE_THUNDERGO BLE_ThunderGo;
BLE_CLIENT BLE_Client;
SENSOR_REMOTER BLE_Remoter;
SENSOR_ATTITUDE Sensor_Attitude(ATTITUDE_IIC_ADDR);

// 蓝牙
uint8_t Rx_Data[21] = {0};
uint8_t Tx_Data[16] = {0};
uint8_t uart_position = 0;
uint8_t uart_recv_finish = 0;
uint8_t add_sum;
bool deviceConnected = false;
bool ble_command_busy = false;

// 固件版本号 4 bytes, 第一个是区分测试版本和 正式版本的前缀
// 测试固件命名规则 Tx.x.x
// 正式发布固件命名 Vx.x.x
// 版本号第一位数字，发布版本具有重要功能修改
// 版本号第二位数字，当有功能修改和增减时，相应地递增
// 版本号第三位数字，每次为某个版本修复BUG时，相应地递增
const uint8_t Version_FW[4] = {'V', 1, 2, 1};

uint32_t thunder_system_parameter = 0;
// 所有模块初始化
void BELL_THUNDER::Setup_All(void)
{
	delay(300);
#ifdef SERIAL_PRINT_HIGHSPEED
	Serial.begin(SERIAL_PRINT_HIGHSPEED);
#else
	Serial.begin(115200);
#endif
	while (!Serial)
		;

	Serial.printf("\n\n======\n");
	Serial.printf("\nFirmware VERSION : %c%d.%d.%d \n",
				  Version_FW[0], Version_FW[1], Version_FW[2], Version_FW[3]);

	Serial.printf("\nInitialize All modules of -Bell_Thunder Go- \n\n");

	SENSOR_IIC::IIC_Init(); // 会对 IIC扩展IC 和 UART扩展模块IC 进行复位

	// 多机通信模块初始化测试
	Open_Multi_Message();
	delay(10);
	Close_Multi_Message();


	Disk_Manager.Disk_Manager_Initial();

	Motor_Servo.Setup_Servo();			 // 舵机初始化配置
	Setup_IR();				 // 巡线IR传感器初始化配置
	Setup_Battery();		 // 电池电压检测初始化配置
	Setup_Led_Indication();  // 初始化指示灯LED
	Setup_Function_Button(); // 初始化程序控制按键：BUTTON_START

	Sensor_Color.Setup(1); // 配置颜色传感器
	Sensor_Color.Setup(2); // 配置颜色传感器
	Sensor_Color.Setup(3); // 配置颜色传感器
	Sensor_Color.Setup(4); // 配置颜色传感器
	Sensor_Color.Setup(5); // 配置颜色传感器
	Sensor_Color.Setup(6); // 配置颜色传感器

	Display_Screen.Setup(); // 初始化单色LED驱动IC配置

	Motor_Thunder.Setup_Motor();	 // 配置电机
	Motor_Thunder.Setup_Motor_PID(); // 配置左右两个电机编码器
	Stop_All();						 //

	LED_Color.LED_OFF(); // 彩灯全关，立即刷新

	// 九轴传感器初始化
	// Attitude_Sensor.Init_Sensor();
	// Attitude_Sensor.Open_Sensor();

	BLE_THUNDERGO::CreateQueueBLE();
	BLE_ThunderGo.Setup_BLE();

	System_Task.Set_Flush_Task(FLUSH_MATRIX_LED);	  // 把 LED点阵显示动画效果刷新工作 交给后台守护线程进行
	System_Task.Set_Flush_Task(FLUSH_COLOR_LED);	   // 把 彩灯刷新工作 交给后台守护线程进行
	System_Task.Set_Flush_Task(FLUSH_MOTOR_PID_CTRL);  // 把 电机闭环控制 交给后台守护线程进行
	System_Task.Set_Flush_Task(FLUSH_CHARACTER_ROLL);  // 把 滚动显示的刷新工作 交给后台守护线程进行
	System_Task.Set_Flush_Task(FLUSH_BATTERY_MEASURE); // 每300ms检测一次电池电压，以保证每次测量都有之前的数据作为滤波数据
	System_Task.Set_Flush_Task(FLUSH_COMMUNICATIONS);  // 开启UART指令、BLE指令 通信控制功能
	System_Task.Create_Deamon_Threads();			   // 创建并开始 守护线程

	Serial.printf("\n*** Initial Completes, spend time:%d ***\n\n", (int)millis());
	SENSOR_IIC::Select_Sensor_AllChannel();

#ifndef DISABLE_LAUNCH_DISPLAY
	// 开机动画/声效
	Start_Show();
#endif

	Serial.printf("Battery Vlotage: %fV\n", ((float)Get_Battery_Data() / 1000));

	if (thunder_system_parameter == 1)
	{
		System_Task.Toggle_Competition_Status(1);
	}
}

void BELL_THUNDER::Set_Ble_Type(int new_type)
{
	if ((ble_type != BLE_TYPE_SERVER) && (new_type == BLE_TYPE_SERVER))
	{
		Serial.println("BLE type: server");
		BLE_THUNDERGO::SetBleConnectType(BLE_SERVER_CONNECTED);
		if (ble_type == BLE_TYPE_CLIENT)
		{
			BLE_Client.Disconnect_Ble_Server();
			BLE_Client.Stop_Scan();
		}

		BLE_ThunderGo.Start_Advertisement(); // 配置 BLE Server

		ble_type = BLE_TYPE_SERVER;
	}
	else if ((ble_type != BLE_TYPE_CLIENT) && (new_type == BLE_TYPE_CLIENT))
	{
		Serial.println("BLE type: client");
		BLE_THUNDERGO::SetBleConnectType(BLE_CLIENT_DISCONNECT); // 允许启动 client scan
		if (ble_type == BLE_TYPE_SERVER)
		{
			BLE_ThunderGo.Delete_Ble_Server_Service(); // 其实函数里没有设置Server断开BLE连接后
		}

		BLE_Client.Setup_Ble_Client(); // 配置 BLE Client

		ble_type = BLE_TYPE_CLIENT;
		Serial.println("Starting BLE Client application...");
	}
	else if ((ble_type != BLE_TYPE_NONE) && (new_type == BLE_TYPE_NONE))
	{
		Serial.println("BLE turn off");
		BLE_THUNDERGO::SetBleConnectType(BLE_NOT_OPEN);
		if (ble_type == BLE_TYPE_CLIENT)
		{
			BLE_Client.Disconnect_Ble_Server();
			BLE_Client.Stop_Scan();
		}
		if (ble_type == BLE_TYPE_SERVER)
		{
			BLE_ThunderGo.Delete_Ble_Server_Service(); // 其实函数里没有设置Server断开BLE连接后
		}

		ble_type = BLE_TYPE_NONE;
	}

	return;
}

void BELL_THUNDER::Reset_All_Components()
{
	Stop_All();
	delay(50);
	Set_Need_Communication(false);
	delay(50);

	Display_Screen.Setup();
	LED_Color.LED_OFF();
}

// 全部终止(电机)
void BELL_THUNDER::Stop_All(void)
{
	// Serial.printf("Stop Motor... \n");

	Disable_En_Motor(); // 关闭编码电机计算
	Motor_Thunder.Set_Motor_Output(1, 0);
	Motor_Thunder.Set_Motor_Output(2, 0);
}

// 电池电压检测初始化配置
void BELL_THUNDER::Setup_Battery()
{
	adc_power_on();
	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_11db);

	if (ESP_OK == esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF))
	{
		Serial.println("\nsupport Efuse.");
	}
	else
	{
		Serial.println("\nNo Efuse.");
	}

	pinMode(BATTERY_ADC_PIN, INPUT);
	cal_value_type = esp_adc_cal_characterize(
		ADC_UNIT_1,
		ADC_ATTEN_11db,
		ADC_WIDTH_12Bit,
		3300,
		&adc_chars);
	Serial.print("ADC Vref: ");
	if (cal_value_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
	{
		Serial.println("eFuse Vref");
	}
	else if (cal_value_type == ESP_ADC_CAL_VAL_EFUSE_TP)
	{
		Serial.println("Two Point");
	}
	else
	{
		Serial.println("Default");
	}
}

uint32_t BELL_THUNDER::Battery_Power_Filter(uint32_t new_data)
{
	uint32_t i, valid_num = 0;
	uint32_t sum_data = 0, max_data = 0, min_data = 15000;
	for (i = 0; i < BATTERY_FILTER_NUM - 1; i++)
	{
		battery_filter_data[i] = battery_filter_data[i + 1];
	}
	battery_filter_data[BATTERY_FILTER_NUM - 1] = new_data;

	for (i = 0; i < BATTERY_FILTER_NUM; i++)
	{
		if (3300 < battery_filter_data[i] && battery_filter_data[i] < 15000)
		{
			valid_num++;
			if (battery_filter_data[i] < min_data)
			{
				min_data = battery_filter_data[i];
			}
			if (max_data < battery_filter_data[i])
			{
				max_data = battery_filter_data[i];
			}
			sum_data += battery_filter_data[i];
		}
	}
	if (valid_num > 3)
	{
		sum_data -= min_data;
		sum_data -= max_data;
		valid_num -= 2;
	}
	//取平均值
	if (valid_num == 0)
	{
		return 0;
	}
	else
	{
		return sum_data / valid_num;
	}
}

uint32_t BELL_THUNDER::Get_Battery_Value()
{
	return Battery_Value;
}

// 获取电池电压
uint32_t BELL_THUNDER::Get_Battery_Data()
{
	uint32_t ADC_mV;
	uint32_t Battery_Voltage;

	ADC_mV = adc1_get_raw(ADC1_CHANNEL_7);
	// Serial.printf("\nADC : %d\n", ADC_mV);

	if (cal_value_type == ESP_ADC_CAL_VAL_DEFAULT_VREF)
	{
		// efuse 的校准信息不存在，所以要手动计算原始ADC值
		ADC_mV = ADC_mV * 3300 / 4095;
	}
	else
	{
		esp_adc_cal_get_voltage(ADC_CHANNEL_7, &adc_chars, &ADC_mV);
	}
	// Serial.printf("ADC Voltage: %dmV\n", ADC_mV);
	Battery_Voltage = ADC_mV * (ADC_R_1 + ADC_R_2) / ADC_R_1; // 分压电阻为：51k;100k

	// Filter
	Battery_Voltage = Battery_Power_Filter(Battery_Voltage);
	// Serial.printf("Bat Voltage: %dmV\n", Battery_Voltage);

	Indicate_Lowpower(Battery_Voltage);
	Battery_Value = Battery_Voltage;

	return Battery_Voltage;
}

/* 
 * 显示低电量 LED Matrix图案
 * 连续响三声 G5 声
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Indicate_Lowpower(uint32_t Battery_Voltage)
{
	if (Battery_Voltage < BATTERY_RESTART_VALUE)
	{
		Serial.printf("\nRestart With Power: %dmV\n", Battery_Voltage);
		delay(50);
		ESP.restart();
		delay(500);
		return;
	}

	if ( Battery_Voltage < BATTERY_LOW_VALUE )
	{
		if(power_alarm_timer + POWER_ALARM_PERIOD < millis() || Battery_Voltage < (Low_Power_Old - 100)){
			power_alarm_timer = millis();
			lowpower_flag = 1;

			Display_Screen.Play_Thunder_Picture(101);

			Speaker_Thunder.Play_Song(79);
			delay(300);
			Speaker_Thunder.Play_Song(79);
			delay(300);
			Speaker_Thunder.Play_Song(79);

			Serial.printf("\nLow Power: %dmV\n", Battery_Voltage);
			Low_Power_Old = Battery_Voltage;
		}
	}
}

/* 
 * 每次Polling_Check 都会更新一次，所以Timer时间精度与 Polling_Check周期相等
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Update_Function_Timer()
{
	// TIMER: wait_key_timer
	if (wait_key_timer != 0)
	{
		wait_key_timer -= POLLING_CHECK_PERIOD;
		// 如果时间到，则将系统状态 process_status 设置为PROCESS_STOP
		if (wait_key_timer == 0)
		{
			Set_Process_Status(PROCESS_STOP);
		}
	}

	// TIMER: led_indication_param.wait_led_timer
	if (led_indication_param.wait_led_timer > POLLING_CHECK_PERIOD)
	{
		led_indication_param.wait_led_timer -= POLLING_CHECK_PERIOD;
	}
	else
	{
		led_indication_param.wait_led_timer = 0;
	}
}

/* 
 * 使能LED指示灯的PWM输出：通过查询 10ms计数器来实现PWM
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Setup_Led_Indication()
{
	pinMode(LED_INDICATION, OUTPUT);
	digitalWrite(LED_INDICATION, LOW);

	if (thunder_system_parameter == 1)
	{
		Set_Program_User(PROCESS_USER_1);
	}
	else
	{
		// 检查Disk 里面是否存有正确的 program_user
		int store_program_mode;
		store_program_mode = Disk_Manager.Read_Program_Mode();
		if ((uint8_t)store_program_mode >= (uint8_t)PROCESS_THUNDER_GO)
		{
			Set_Program_User(PROCESS_USER_1);
			Disk_Manager.Write_Program_User(program_user);
		}
		else
		{
			Set_Program_User(store_program_mode);
		}
	}
}

/* 
 * 设置LED指示灯的显示参数：周期、闪烁时长、闪亮次数
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Set_Led_Indication_param()
{
	led_indication_counter = 0;

	// 防止 周期 < 闪烁总脉宽
	if ((led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount > led_indication_param.led_indication_period)
	{
		led_indication_param.led_indication_amount = led_indication_param.led_indication_period /
													 (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty);
	}

	// if(led_indication_on_duty > 0){
	//   digitalWrite(LED_INDICATION, HIGH);
	// }else{
	//   digitalWrite(LED_INDICATION, LOW);
	// }
}

/* 
 * 更新指示灯状态
 * 
 * @parameters: current_counter循环计数器，单位为ms，精度为10ms
 * @return: 
 */
void BELL_THUNDER::Update_Led_Indication_Status(uint32_t &current_counter)
{
	// wait_led_timer 存了 Led_Indication使能的倒计时时间，时间到之前LED处于熄灭状态
	if (led_indication_param.wait_led_timer != 0)
	{
		digitalWrite(LED_INDICATION, LOW);
		current_counter = 0;
	}
	else
	{
		if (current_counter >= led_indication_param.led_indication_period)
		{
			if (led_indication_param.led_indication_once_flag == 1)
			{
				led_indication_param.led_indication_once_flag = 0;
				Set_Process_Status(PROCESS_STOP);
			}

			if (led_indication_param.led_indication_on_duty != 0 && led_indication_param.wait_led_timer == 0)
			{
				digitalWrite(LED_INDICATION, HIGH);
			}
			else
			{
				digitalWrite(LED_INDICATION, LOW);
			}

			current_counter = 0;
		}
		else if (current_counter >= (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount)
		{
			digitalWrite(LED_INDICATION, LOW);
		}
		else
		{
			if (current_counter % (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) <= led_indication_param.led_indication_on_duty)
			{
				digitalWrite(LED_INDICATION, HIGH);
			}
			else
			{
				digitalWrite(LED_INDICATION, LOW);
			}
		}
	}
}

/* 
 * 初始化程序控制按键，通过此按键可以控制程序运行，切换程序
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Setup_Function_Button()
{
	pinMode(BUTTON_START, INPUT_PULLDOWN);
	button_press_time = 0;
	button_release_time = BUTTON_FILTER_PARAM;
	button_press_result_time = 0;
	button_release_result_time = BUTTON_FILTER_PARAM;
	button_status_record = 0;
	button_press_counter = 0;
	button_active = false;
}

/* 
 * 获取按键状态
 *  button_press_time：按下时长
 *  button_release_time：释放时长
 *  button_release_result_time：确认按下前 的释放时长
 *  button_press_result_time：确认释放前 的按下时长
 * 
 * @parameters: 
 * @return: 确认button按下返回 1，否则返回 0
 */
uint8_t BELL_THUNDER::Get_Function_Button_Status()
{
	// digitalRead(BUTTON_START)获取按键状态
	if (digitalRead(BUTTON_START) == 1)
	{
		// 10ms检测一次，防止button_press_time、button_release_time溢出
		button_press_time += (button_press_time > 10000) ? 0 : 10;
		if (button_press_time > BUTTON_FILTER_PARAM)
		{
			button_release_result_time = button_release_time;
			button_release_time = 0;
			return 1; // 按下有效
		}
		else
		{
			return 0;
		}
	}
	else
	{
		button_release_time += (button_release_time > 10000) ? 0 : 10;
		if (button_release_time > BUTTON_FILTER_PARAM)
		{
			button_press_result_time = button_press_time;
			button_press_time = 0;
			return 0; // 释放有效
		}
		else
		{
			return 1;
		}
	}
}

/* 
 * 更新按键控制值
 * button_active：标志是否有按键事件在进行时，
 * button_press_counter：一次按键事件中发生按下次数
 * button_status_record：记录着上次检测的按键状态
 * 
 * 注：当连续click正在发生时，跟着一个长按，发生的按键事件是长按
 * 
 * @parameters: 
 * @return: 
 */
int BELL_THUNDER::Check_Function_Button_Value()
{
	int button_value = KEY_NONE;

	if (Get_Function_Button_Status() == 1)
	{
		// 每次从释放到按下，按下次数增加1
		if (button_status_record == 0)
		{
			button_press_counter++;
		}

		// 当按下时长超过 BUTTON_LONG_PRESS_TIME ,发出一个按键事件
		if (button_press_time == BUTTON_LONG_PRESS_TIME)
		{
			button_value = KEY_LONG_NO_RELEASE; // 获得按键事件1
		}

		if (wait_key_timer != 0)
		{
			wait_key_timer = 3000;
		}
		button_active = true; // 按键动作正在进行时
		button_status_record = 1;
	}
	else
	{
		if (button_active)
		{
			// 保证先进入 KEY_LONG_NO_RELEASE 状态再进入 KEY_LONG_RELEASE
			if (button_press_result_time > BUTTON_LONG_PRESS_TIME)
			{
				button_value = KEY_LONG_RELEASE; // 获得按键事件2

				button_press_counter = 0;
				button_active = false; // 完成一次按键动作
			}
			else
			{
				if (button_release_time >= BUTTON_CONTINUE_CLICK_TIME)
				{
					if (button_press_counter == 1)
					{
						button_value = KEY_CLICK_ONE; // 获得按键事件3
					}
					else if (button_press_counter == 2)
					{
						button_value = KEY_CLICK_TWO; // 获得按键事件4
					}
					else if (button_press_counter == 3)
					{
						button_value = KEY_CLICK_THREE; // 获得按键事件5
					}
					else if (button_press_counter == 4)
					{
						button_value = KEY_CLICK_FOUR; // 获得按键事件6
					}

					button_press_counter = 0;
					button_active = false; // 完成一次按键动作
				}
			}
		}

		button_status_record = 0;
	}

	if (button_value != KEY_NONE)
	{
		function_button_event = button_value;
		Update_Process_Status(button_value);
	}
	return button_value;
}

bool BELL_THUNDER::Check_Function_Button_Event(int key_event)
{
	bool ret = false;

	if (function_button_event == key_event)
	{
		ret = true;
		function_button_event = KEY_NONE;
	}

	return ret;
}

/* 
 * 设置系统状态，并更新 指示灯为相应的状态
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Set_Process_Status(int new_status)
{
	led_indication_param.wait_led_timer = 700;
	led_indication_param.led_indication_once_flag = 0;
	process_status = new_status;

	switch (process_status)
	{
	case PROCESS_STOP:
		led_indication_param.led_indication_period = 2000;
		led_indication_param.led_indication_on_duty = 1900;
		led_indication_param.led_indication_off_duty = 100;
		led_indication_param.led_indication_amount = 1;
		break;
	case PROCESS_INDICATE_STOP:
		System_Task.Clear_All_Loops();
		Set_Process_Status(PROCESS_STOP);
		return; // 做好STOP 的工作，直接退出函数，以免函数后续有变动
		break;
	case PROCESS_THUNDER_GO:
		// if(thunder_system_parameter == 1){
		//   led_indication_param.led_indication_period = 3000;
		//   led_indication_param.led_indication_on_duty = 100;
		//   led_indication_param.led_indication_off_duty = 400;
		//   led_indication_param.led_indication_amount = 2;
		// }
		// else
		{
			led_indication_param.led_indication_on_duty = 1000;
			led_indication_param.led_indication_off_duty = 300;
			led_indication_param.led_indication_amount = 1;
			led_indication_param.led_indication_period = (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount + 2000;
		}
		break;
	case PROCESS_USER_1:
		led_indication_param.led_indication_on_duty = 70;
		led_indication_param.led_indication_off_duty = 300;
		led_indication_param.led_indication_amount = 1;
		led_indication_param.led_indication_period = (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount + 2000;
		break;
	case PROCESS_USER_2:
		led_indication_param.led_indication_on_duty = 70;
		led_indication_param.led_indication_off_duty = 300;
		led_indication_param.led_indication_amount = 2;
		led_indication_param.led_indication_period = (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount + 2000;
		break;
	case PROCESS_USER_3:
		led_indication_param.led_indication_on_duty = 70;
		led_indication_param.led_indication_off_duty = 300;
		led_indication_param.led_indication_amount = 3;
		led_indication_param.led_indication_period = (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount + 2000;
		break;
	case PROCESS_USER_4:
		led_indication_param.led_indication_on_duty = 70;
		led_indication_param.led_indication_off_duty = 300;
		led_indication_param.led_indication_amount = 4;
		led_indication_param.led_indication_period = (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount + 2000;
		break;
	case PROCESS_INDICATE_SWITCH:
		System_Task.Clear_All_Loops();
		led_indication_param.wait_led_timer = 0;
		led_indication_param.led_indication_period = 100;
		led_indication_param.led_indication_on_duty = 50;
		led_indication_param.led_indication_off_duty = 50;
		led_indication_param.led_indication_amount = 1;
		break;
	case PROCESS_WAIT_SWITCH:
		led_indication_param.wait_led_timer = 0;
		led_indication_param.led_indication_period = 100;
		led_indication_param.led_indication_on_duty = 50;
		led_indication_param.led_indication_off_duty = 50;
		led_indication_param.led_indication_amount = 1;
		break;
	}

	Set_Led_Indication_param();
}

/* 
 * 根据按键事件更新程序运行状态
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Update_Process_Status(int button_event)
{
	if (thunder_system_parameter == 1)
	{
		if ((button_event != KEY_CLICK_ONE) && (button_event != KEY_CLICK_TWO))
		{
			return;
		}
	}

	if (button_event != KEY_NONE)
	{
		// 发生按键事件后，清零等待按键事件的倒计时定时器
		wait_key_timer = 0;
	}

	switch (process_status)
	{
	case PROCESS_STOP:
	{ //
		if (button_event == KEY_LONG_NO_RELEASE)
		{
			function_button_event = KEY_NONE;
			Set_Process_Status(PROCESS_INDICATE_SWITCH);
			system_program_mode = 1;
		}
		else if (button_event == KEY_CLICK_ONE)
		{
			function_button_event = KEY_NONE;
			Set_Program_Run_Index(program_user);
			system_program_mode = 1;
		}
		else if (button_event == KEY_CLICK_TWO && system_program_mode == 0) // 只有未确定系统程序时才能选APP程序
		{
			function_button_event = KEY_NONE;
			Set_Program_Run_Index(PROCESS_THUNDER_GO);
			system_program_mode = 2;
		}

		break;
	}
	case PROCESS_THUNDER_GO: // APP程序不能接受程序切换
	{
		// if (button_event == KEY_LONG_NO_RELEASE)
		// {
		//   function_button_event = KEY_NONE;
		//   Set_Process_Status(PROCESS_INDICATE_SWITCH);
		// }

		break;
	}
	case PROCESS_USER_1:
	{
		if (button_event == KEY_LONG_NO_RELEASE)
		{
			function_button_event = KEY_NONE;
			Set_Process_Status(PROCESS_INDICATE_STOP);
		}

		break;
	}
	case PROCESS_USER_2:
	{
		if (button_event == KEY_LONG_NO_RELEASE)
		{
			function_button_event = KEY_NONE;
			Set_Process_Status(PROCESS_INDICATE_STOP);
		}

		break;
	}
	case PROCESS_USER_3:
	{
		if (button_event == KEY_LONG_NO_RELEASE)
		{
			function_button_event = KEY_NONE;
			Set_Process_Status(PROCESS_INDICATE_STOP);
		}

		break;
	}
	case PROCESS_USER_4:
	{
		if (button_event == KEY_LONG_NO_RELEASE)
		{
			function_button_event = KEY_NONE;
			Set_Process_Status(PROCESS_INDICATE_STOP);
		}

		break;
	}
	case PROCESS_INDICATE_SWITCH:
	{
		if (button_event == KEY_LONG_RELEASE)
		{
			function_button_event = KEY_NONE;
			Set_Process_Status(PROCESS_WAIT_SWITCH);
		}

		break;
	}
	case PROCESS_WAIT_SWITCH:
	{
		if (button_event == KEY_LONG_NO_RELEASE)
		{
			function_button_event = KEY_NONE;
			Set_Process_Status(PROCESS_INDICATE_SWITCH);
		}
		else if (button_event == KEY_CLICK_ONE)
		{
			function_button_event = KEY_NONE;
			// Speaker_Thunder.Play_Song(3);
			Set_Program_User(PROCESS_USER_1);
			Disk_Manager.Write_Program_User(program_user);
		}
		else if (button_event == KEY_CLICK_TWO)
		{
			function_button_event = KEY_NONE;
			// Speaker_Thunder.Play_Song(4);
			Set_Program_User(PROCESS_USER_2);
			Disk_Manager.Write_Program_User(program_user);
		}
		else if (button_event == KEY_CLICK_THREE)
		{
			function_button_event = KEY_NONE;
			// Speaker_Thunder.Play_Song(5);
			Set_Program_User(PROCESS_USER_3);
			Disk_Manager.Write_Program_User(program_user);
		}
		else if (button_event == KEY_CLICK_FOUR)
		{
			function_button_event = KEY_NONE;
			// Speaker_Thunder.Play_Song(6);
			Set_Program_User(PROCESS_USER_4);
			Disk_Manager.Write_Program_User(program_user);
		}

		break;
	}
	case PROCESS_READY_RUN:
	{

		break;
	}
	default:
	{

		break;
	}
	}
}

// 复位程序号为 1号程序
void BELL_THUNDER::Reset_Process_Status()
{
	if (program_user != PROCESS_USER_1)
	{
		Set_Program_User(PROCESS_USER_1);
		Disk_Manager.Write_Program_User(program_user);
	}
}

void BELL_THUNDER::Toggle_Led_mode(uint32_t period, uint32_t on_duty, uint32_t off_duty, uint8_t amount)
{
	led_indication_param.wait_led_timer = 700;
	led_indication_param.led_indication_once_flag = 0;

	led_indication_param.led_indication_period = period;
	led_indication_param.led_indication_on_duty = on_duty;
	led_indication_param.led_indication_off_duty = off_duty;
	led_indication_param.led_indication_amount = amount;

	Set_Led_Indication_param();
}

void BELL_THUNDER::Set_Program_User(int new_program_user)
{
	program_user = new_program_user;

	led_indication_param.led_indication_amount = (uint8_t)new_program_user + 1;
	led_indication_param.wait_led_timer = 700;
	led_indication_param.led_indication_once_flag = 1;
	led_indication_param.led_indication_on_duty = 100;
	led_indication_param.led_indication_off_duty = 300;
	led_indication_param.led_indication_period = (led_indication_param.led_indication_on_duty + led_indication_param.led_indication_off_duty) * led_indication_param.led_indication_amount;
	Set_Led_Indication_param();

	// 可以不用等闪烁提示 完毕 就立刻进入“等待启动状态”
	process_status = PROCESS_STOP;
}

void BELL_THUNDER::Set_Program_Run_Index(int new_program)
{
	switch (new_program)
	{
	case PROCESS_USER_1:
	{
		Set_Process_Status(PROCESS_USER_1);
		program_change_to = PROGRAM_USER_1;
		break;
	}
	case PROCESS_USER_2:
	{
		Set_Process_Status(PROCESS_USER_2);
		program_change_to = PROGRAM_USER_2;
		break;
	}
	case PROCESS_USER_3:
	{
		Set_Process_Status(PROCESS_USER_3);
		program_change_to = PROGRAM_USER_3;
		break;
	}
	case PROCESS_USER_4:
	{
		Set_Process_Status(PROCESS_USER_4);
		program_change_to = PROGRAM_USER_4;
		break;
	}
	case PROCESS_THUNDER_GO:
	{ //
		Set_Process_Status(PROCESS_THUNDER_GO);
		program_change_to = PROGRAM_THUNDER_GO;
		break;
	}
	default:
	{

		break;
	}
	}
}

// 编码电机  闭环计算
void BELL_THUNDER::En_Motor(void)
{
	Motor_Thunder.Update_Encoder_Value();

	motor_semaphore.take(std::string("control"));
	if (En_Motor_Flag == 2)
	{
		Motor_Thunder.Drive_Car_Control();
	}
	else if (En_Motor_Flag == 1)
	{
		// if (xSemaphoreTake(Timer_PID_Flag, portMAX_DELAY) == pdTRUE) // 控制周期PID_dt[ms]
		{
			Motor_Thunder.PID_Speed();
		}
	}
	else if (En_Motor_Flag == 3)
	{
		Motor_Thunder.Position_Control();
	}
	motor_semaphore.give();
}

// 开启 Drive_Car_Control 功能
void BELL_THUNDER::Enable_Drive_Car(void)
{
	motor_semaphore.take(std::string("Drive_Car"));
	if (En_Motor_Flag != 2)
	{
		En_Motor_Flag = 2;
	}
	motor_semaphore.give();
}

// 开启 电机PID 功能
void BELL_THUNDER::Enable_En_Motor(void)
{
	motor_semaphore.take(std::string("En_Motor"));
	if (En_Motor_Flag != 1)
	{
		En_Motor_Flag = 1;
	}
	motor_semaphore.give();
}

// 开启 电机位置 控制
void BELL_THUNDER::Enable_Motor_Position(void)
{
	motor_semaphore.take(std::string("Motor_Position"));
	if (En_Motor_Flag != 3)
	{
		Motor_Thunder.Clear_Position_Control();
		En_Motor_Flag = 3;
	}
	motor_semaphore.give();
}

// 关闭编码电机计算
void BELL_THUNDER::Disable_En_Motor(void)
{
	motor_semaphore.take(std::string("Disable_Ctrl"));
	if (En_Motor_Flag != 0)
	{
		// 清除PID控制的变量
		// Motor_Thunder.Set_Target(1, 0);
		// Motor_Thunder.Set_Target(2, 0);

		En_Motor_Flag = 0;
		Motor_Thunder.PID_Reset();
		Motor_Thunder.All_PID_Init();
		delay(1); //等待电机PID控制稳定
	}
	motor_semaphore.give();
}

// 巡线IR传感器初始化配置
void BELL_THUNDER::Setup_IR()
{
	pinMode(IR_1, INPUT);
	pinMode(IR_2, INPUT);
}

// 获取巡线IR数据
void BELL_THUNDER::Get_IR_Data(uint8_t data[])
{
	SENSOR_IIC::Set_Port4_IIC(false);
	data[0] = digitalRead(IR_1);
	data[1] = digitalRead(IR_2);

#ifdef DEBUG_IR_SENSOR
	Serial.printf("*** Left: %d ___ Right: %d ***\n", data[0], data[1]);
#endif
}

void BELL_THUNDER::Wait_For_Motor_Slow()
{
	// Motor_Thunder.Set_Motor_Power(1, 0);
	// Motor_Thunder.Set_Motor_Power(2, 0);
	Motor_Thunder.Motor_Brake(MOTOR_INDEX_LEFT);
	Motor_Thunder.Motor_Brake(MOTOR_INDEX_RIGHT);

	do
	{
		delay(1);
	} while (Motor_Thunder.Get_Speed_simple(1) > 15 || Motor_Thunder.Get_Speed_simple(2) > 15);
}
#if 1
/* 
 * 前驱电机的巡线
 * 1、控制没有使用PID控制，直接列举方式将传感器状态值作为参考量，
 *    电机运动功率分三个级别 Line_H_Speed Line_M_Speed Line_L_Speed（无PID控制速度）
 *    对应着不同的状态值：
 *    0(两点黑线) 
 *    1(偏右时间超过 50ms，需要大幅度偏左运动) 
 *    2 (偏左时间超过 50ms，需要大幅度偏右运动) 
 *    3(偏右时间小于 50ms，需要小幅度偏左运动) 
 *    4 (偏左时间小于 50ms，需要小幅度偏右运动) 
 *    5(两个白点持续长时间)
 *  a、快速出线，打转
 * 2、电机安装在前面，传感器安装在前面
 * 3、传感器安装高度升为 1.5cm，与电机间隔两个安装孔位置
 * 
 * @parameters: 
 * @return: 
 */
#define WAIT_DIRECTION_COMFIRM_TIME 100 //ms
#define MAYBE_STRAIGHT_DIRECTION 300	//ms
#define LITTLE_L_R_POWER_DIFF 5
#define SPIN_L_R_DIFF_ROTATEVALUE 700 //编码器数值的差量300为打转90度
#define LINE_TRACING_STOP_FLAG (line_tracing_running == false || Rx_Data[0] == 0x61 || deviceConnected == false)

void BELL_THUNDER::Line_Tracing(void)
{
	int last_L_R_diffrotate;
	int current_L_R_diffrotate;
	int rotate_back_quantity;
	int line_out_flag = 0;
	int LED_counter = 99;

	uint32_t L_last_time = 0; // 上次偏左的时间戳
	uint32_t R_last_time = 0; // 上次偏右的时间戳

	Line_last_time = millis();
	Line_last_led_time = millis();
	Line_last_sound_time = millis();

	Speaker_Thunder.Play_Song(131);
	while (1)
	{
		delay(5);
		// 接收到巡线停止指令， 则退出巡线循环
		if (LINE_TRACING_STOP_FLAG)
		{
			break;
		}
		Get_IR_Data(IR_Data); //更新巡线传感器数据 //0-->白; 1-->黑
		// Serial.printf("*** Left: %d ___ Right: %d ***\n", IR_Data[0], IR_Data[1]);
		// Serial.printf("*** line_state: %d ***\n", line_state);

		current_time = millis();

		if ((IR_Data[0] == 0) & (IR_Data[1] == 0)) //Serial.printf("SSSSSSSSSS 没线 SSSSSSSSSS\n");
		{
			// Speaker_Thunder.Play_Song(7); //test用---------
			if (((current_time - 5000) > Line_last_time) & (current_time > 19000)) //超时未找到线停止
			{
				Motor_Thunder.Set_Motor_Power(1, Line_L_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_B_Speed);
				line_state = 5;
			}
			else if (line_state == 0) // 忽然从全黑变为全零过程中出线
			{
				// 左右扭头查找黑线
				while (1)
				{
					Motor_Thunder.Set_Motor_Power(1, Line_B_Speed);
					Motor_Thunder.Set_Motor_Power(2, Line_L_Speed);
					Line_last_time = millis();
					current_time = millis();
					while (current_time - 2000 < Line_last_time)
					{
						Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
						if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
						{
							break;
						}
						current_time = millis();
						delay(5);
					}
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}
					Motor_Thunder.Set_Motor_Power(1, Line_L_Speed);
					Motor_Thunder.Set_Motor_Power(2, Line_B_Speed);
					Line_last_time = millis();
					current_time = millis();
					while (current_time - 2000 < Line_last_time)
					{
						Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
						if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
						{
							break;
						}
						current_time = millis();
						delay(5);
					}
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}

					if (LINE_TRACING_STOP_FLAG)
					{
						break;
					}
					delay(5);
				}
			}
			else if (line_state == 3) //短时间偏右的过程中出线，打转(可能是直角转弯)
			{
				// 左右打转90度查找黑线
				rotate_back_quantity = 0;
				while (1)
				{
					Motor_Thunder.Set_Motor_Power(1, Line_B_Speed);
					Motor_Thunder.Set_Motor_Power(2, Line_L_Speed);
					current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
					last_L_R_diffrotate = current_L_R_diffrotate;
					while (current_L_R_diffrotate + SPIN_L_R_DIFF_ROTATEVALUE + rotate_back_quantity > last_L_R_diffrotate)
					{
						Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
						if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
						{
							break;
						}
						current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
						delay(5);
					}
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}

					Motor_Thunder.Set_Motor_Power(1, Line_L_Speed);
					Motor_Thunder.Set_Motor_Power(2, Line_B_Speed);
					current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
					last_L_R_diffrotate = current_L_R_diffrotate;
					while (current_L_R_diffrotate - SPIN_L_R_DIFF_ROTATEVALUE * 2 < last_L_R_diffrotate)
					{
						Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
						if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
						{
							break;
						}
						current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
						delay(5);
					}
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}

					rotate_back_quantity = SPIN_L_R_DIFF_ROTATEVALUE; // 回转要增加旋转量

					if (LINE_TRACING_STOP_FLAG)
					{
						break;
					}
					delay(5);
				}
				Wait_For_Motor_Slow();
			}
			else if (line_state == 4) //短时间偏左的过程中出线，打转(可能是直角转弯)
			{
				// 左右打转90度查找黑线
				rotate_back_quantity = 0;
				while (1)
				{
					Motor_Thunder.Set_Motor_Power(1, Line_L_Speed);
					Motor_Thunder.Set_Motor_Power(2, Line_B_Speed);
					current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
					last_L_R_diffrotate = current_L_R_diffrotate;
					while (current_L_R_diffrotate - SPIN_L_R_DIFF_ROTATEVALUE - rotate_back_quantity < last_L_R_diffrotate)
					{
						Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
						if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
						{
							break;
						}
						current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
						delay(5);
					}
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}

					Motor_Thunder.Set_Motor_Power(1, Line_B_Speed);
					Motor_Thunder.Set_Motor_Power(2, Line_L_Speed);
					current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
					last_L_R_diffrotate = current_L_R_diffrotate;
					while (current_L_R_diffrotate + SPIN_L_R_DIFF_ROTATEVALUE * 2 > last_L_R_diffrotate)
					{
						Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
						if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
						{
							break;
						}
						current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
						delay(5);
					}
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}

					rotate_back_quantity = SPIN_L_R_DIFF_ROTATEVALUE; // 回转要增加旋转量

					if (LINE_TRACING_STOP_FLAG)
					{
						break;
					}
					delay(5);
				}
				Wait_For_Motor_Slow();
			}
			else if (line_state == 1) //偏右的过程中出线，快速漂移打转(弯道转弯)
			{
				Motor_Thunder.Set_Motor_Power(1, Line_B_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_M_Speed);
				while (1)
				{
					Line_last_time = millis(); // 不改变line_state，刷新时间
					Get_IR_Data(IR_Data);	  //更新IR数据 //0-->白; 1-->黑
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}
					delay(5);
				}
				Wait_For_Motor_Slow();
			}
			else if (line_state == 2) //偏左的过程中出线，快速漂移打转(弯道转弯)
			{
				Motor_Thunder.Set_Motor_Power(1, Line_M_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_B_Speed);
				while (1)
				{
					Line_last_time = millis(); // 不改变line_state，刷新时间
					Get_IR_Data(IR_Data);	  //更新IR数据 //0-->白; 1-->黑
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}
					delay(5);
				}
				Wait_For_Motor_Slow();
			}
			else
			{
				Motor_Thunder.Set_Motor_Power(1, Line_B_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_L_Speed);
			}
		}
		else if (IR_Data[0] == 0) //Serial.printf("SSSSSSSSSS 右转 SSSSSSSSSS\n");
		{
			if ((line_state == 1) | (line_state == 3)) //从左转过来的需要更新时间
			{
				// Motor_Thunder.Set_Motor_Power(1, 0);
				Motor_Thunder.Motor_Brake(MOTOR_INDEX_LEFT);
				Motor_Thunder.Set_Motor_Power(2, Line_L_Speed);
				Line_last_time = millis(); // 不改变运动状态
				line_out_flag = 1;
				continue; // 保持前运动状态继续运动
			}
			line_out_flag = 0;

			if (R_last_time + MAYBE_STRAIGHT_DIRECTION < current_time)
			{ // 如果上次偏右时间已经超过200ms，那这次偏左需要偏向运动
				Motor_Thunder.Set_Motor_Power(1, Line_M_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_L_Speed);
			}
			else
			{
				Motor_Thunder.Set_Motor_Power(1, Line_M_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_M_Speed - LITTLE_L_R_POWER_DIFF);
			}

			if (line_state == 2)
			{
				Line_last_time = millis(); //一直为偏左出线，所以一直更新状态时间
			}
			else if ((current_time - WAIT_DIRECTION_COMFIRM_TIME) > Line_last_time && line_state == 4)
			{
				line_state = 2; // 偏左时间已经超过 50ms
				L_last_time = current_time;
			}
			else
			{
				line_state = 4; // 偏左时间小于 50ms，急转偏右运动一小段时间
				Motor_Thunder.Set_Motor_Power(1, Line_H_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_L_Speed);
			}
		}
		else if (IR_Data[1] == 0) //Serial.printf("SSSSSSSSSS 左转 SSSSSSSSSS\n");
		{
			if ((line_state == 2) | (line_state == 4)) //从右转过来的需要更新时间
			{
				Motor_Thunder.Set_Motor_Power(1, Line_L_Speed);
				// Motor_Thunder.Set_Motor_Power(2, 0);
				Motor_Thunder.Motor_Brake(MOTOR_INDEX_RIGHT);
				Line_last_time = millis(); // 不改变运动状态
				line_out_flag = 1;
				continue; // 保持前运动状态继续运动
			}
			line_out_flag = 0;

			if (L_last_time + MAYBE_STRAIGHT_DIRECTION < current_time)
			{ // 如果上次偏右时间已经超过200ms，那这次偏左需要偏向运动
				Motor_Thunder.Set_Motor_Power(1, Line_L_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_M_Speed);
			}
			else
			{
				Motor_Thunder.Set_Motor_Power(1, Line_M_Speed - LITTLE_L_R_POWER_DIFF);
				Motor_Thunder.Set_Motor_Power(2, Line_M_Speed);
			}

			if (line_state == 1)
			{
				Line_last_time = millis(); //一直为偏右出线，所以一直更新状态时间, 不改变运动状态
			}
			else if ((current_time - WAIT_DIRECTION_COMFIRM_TIME) > Line_last_time && line_state == 3)
			{
				line_state = 1; // 偏右时间已经超过 50ms
				R_last_time = current_time;
			}
			else
			{
				Motor_Thunder.Set_Motor_Power(1, Line_L_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_H_Speed);
				line_state = 3; // 偏右时间小于 50ms，急转偏右运动一小段时间
			}
		}
		else //Serial.printf("SSSSSSSSSS 线上 SSSSSSSSSS\n");
		{
			if (line_out_flag == 0)
			{
				Motor_Thunder.Set_Motor_Power(1, Line_M_Speed);
				Motor_Thunder.Set_Motor_Power(2, Line_M_Speed);
				line_state = 0;
			}
			Line_last_time = millis();
		}

		////////////////////////////////// (start)巡线时LED画面、播放的声音 //////////////////////////////////
		if ((current_time - 50) > Line_last_led_time)
		{
			if ((LED_counter > 98) & (line_state != 5))
			{
				LED_counter = 77;
			}
			else if ((LED_counter > 100) & (line_state == 5))
			{
				LED_counter = 99;
				Line_last_sound_time = millis();
			}

			Display_Screen.Play_Thunder_Picture(LED_counter); //单色点阵图案
			LED_counter++;

			Line_last_led_time = millis();
		}

		if (((current_time - 19000) > Line_last_sound_time) & (current_time > 19000))
		{
			// Speaker_Thunder.Play_Song(139);
			Line_last_sound_time = millis();
		}
		////////////////////////////////// (end)巡线时LED画面、播放的声音 //////////////////////////////////
	}
	Stop_All();
	line_tracing_running = false;
	Speaker_Thunder.Play_Song(130);
}

/* 
 * 左右电机缓慢同步前行
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Motor_Slow_Go()
{
	Motor_Thunder.Set_Car_Speed_Direction(0, 0);

	do
	{
		delay(1);
	} while (Motor_Thunder.Get_Speed_simple(1) > 15 || Motor_Thunder.Get_Speed_simple(2) > 15);
}
/* 
 * 车子左右晃动0.5s, IR传感器发现有黑线则返回
 * 
 * @parameters: 
 *    dir: 1为先左后右，-1为先右后左
 * @return: 0表示未发现两点黑线返回，1表示已发现两点黑线返回
 */
int BELL_THUNDER::Car_Shake_Left_Right(int dir)
{
	if (dir == 1)
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, -50);
	}
	else
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, 50);
	}
	Line_last_time = millis();
	current_time = millis();
	while (current_time - 500 < Line_last_time)
	{
		Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
		if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
		{
			break;
		}
		current_time = millis();
		delay(5);
	}
	if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
	{
		return 1;
	}

	if (dir == 1)
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, 50);
	}
	else
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, -50);
	}
	Line_last_time = millis();
	current_time = millis();
	while (current_time - 500 < Line_last_time)
	{
		Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
		if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
		{
			break;
		}
		current_time = millis();
		delay(5);
	}
	if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
	{
		return 1;
	}

	return 0;
}

/* 
 * 车子左右旋转90度, IR传感器发现有黑线则返回
 * 
 * @parameters: 
 *    dir: 1为先左后右，-1为先右后左
 * @return: 0表示未发现两点黑线返回，1表示已发现两点黑线返回
 */
int BELL_THUNDER::Car_Rotate_90_Left_Right(int dir)
{
	int last_L_R_diffrotate;
	int current_L_R_diffrotate;

	if (dir == 1)
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, -50);
	}
	else
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, 50);
	}
	current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
	last_L_R_diffrotate = current_L_R_diffrotate;
	while (dir == 1 ? (current_L_R_diffrotate + SPIN_L_R_DIFF_ROTATEVALUE > last_L_R_diffrotate) : (current_L_R_diffrotate - SPIN_L_R_DIFF_ROTATEVALUE < last_L_R_diffrotate))
	{
		Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
		if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
		{
			break;
		}
		current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
		delay(5);
	}
	if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
	{
		return 1;
	}

	if (dir == 1)
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, 50);
	}
	else
	{
		Motor_Thunder.Set_Car_Speed_Direction(15, -50);
	}
	current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
	last_L_R_diffrotate = current_L_R_diffrotate;
	while (dir == 1 ? (current_L_R_diffrotate - SPIN_L_R_DIFF_ROTATEVALUE < last_L_R_diffrotate) : (current_L_R_diffrotate + SPIN_L_R_DIFF_ROTATEVALUE > last_L_R_diffrotate))
	{
		Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
		if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
		{
			break;
		}
		current_L_R_diffrotate = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
		delay(5);
	}
	if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
	{
		return 1;
	}

	return 0;
}

void BELL_THUNDER::Line_Tracing_Speed_Ctrl(void)
{
	uint32_t L_last_time = 0; // 上次偏左的时间戳
	uint32_t R_last_time = 0; // 上次偏右的时间戳
	int line_out_flag = 0;
	int LED_counter = 99;

	Bell_Thunder.Enable_Drive_Car();

	Line_last_time = millis();
	Line_last_led_time = millis();
	Line_last_sound_time = millis();

	Speaker_Thunder.Play_Song(131);
	while (1)
	{
		delay(5);
		// 接收到巡线停止指令， 则退出巡线循环
		if (LINE_TRACING_STOP_FLAG)
		{
			break;
		}
		Get_IR_Data(IR_Data); //更新巡线传感器数据 //0-->白; 1-->黑
		// Serial.printf("*** Left: %d ___ Right: %d ***\n", IR_Data[0], IR_Data[1]);
		// Serial.printf("*** line_state: %d ***\n", line_state);

		current_time = millis();

		if ((IR_Data[0] == 0) & (IR_Data[1] == 0)) //Serial.printf("* 没线 \n");
		{
			// Speaker_Thunder.Play_Song(7); //test用---------
			if (((current_time - 5000) > Line_last_time) & (current_time > 19000)) //超时未找到线停止
			{
				Motor_Thunder.Set_Car_Speed_Direction(15, 100);
				line_state = 5;
			}
			else if (line_state == 0) // 忽然从全黑变为全零过程中出线
			{
				// 左右扭头查找黑线
				while (1)
				{
					if (1 == Car_Shake_Left_Right(1))
					{
						break;
					}
					if (1 == Car_Shake_Left_Right(-1))
					{
						break;
					}

					if (LINE_TRACING_STOP_FLAG)
					{
						break;
					}
					delay(5);
				}
			}
			else if (line_state == 3) //短时间偏右的过程中出线，打转(可能是直角转弯)
			{
				// 左右打转90度查找两点黑线
				while (1)
				{
					if (1 == Car_Rotate_90_Left_Right(1))
					{
						break;
					}
					if (1 == Car_Rotate_90_Left_Right(-1))
					{
						break;
					}

					if (LINE_TRACING_STOP_FLAG)
					{
						break;
					}
					delay(5);
				}
				// Motor_Slow_Go();
			}
			else if (line_state == 4) //短时间偏左的过程中出线，打转(可能是直角转弯)
			{
				// 右左打转90度查找两点黑线
				while (1)
				{
					if (1 == Car_Rotate_90_Left_Right(-1))
					{
						break;
					}
					if (1 == Car_Rotate_90_Left_Right(1))
					{
						break;
					}

					if (LINE_TRACING_STOP_FLAG)
					{
						break;
					}
					delay(5);
				}
				// Motor_Slow_Go();
			}
			else if (line_state == 1) //偏右的过程中出线，快速漂移打转(弯道转弯)
			{
				Motor_Thunder.Set_Car_Speed_Direction(20, -40);
				while (1)
				{
					Line_last_time = millis(); // 不改变line_state，刷新时间
					Get_IR_Data(IR_Data);	  //更新IR数据 //0-->白; 1-->黑
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}
					delay(5);
				}
				// Motor_Slow_Go();
			}
			else if (line_state == 2) //偏左的过程中出线，快速漂移打转(弯道转弯)
			{
				Motor_Thunder.Set_Car_Speed_Direction(20, 40);
				while (1)
				{
					Line_last_time = millis(); // 不改变line_state，刷新时间
					Get_IR_Data(IR_Data);	  //更新IR数据 //0-->白; 1-->黑
					if ((IR_Data[0] != 0) || (IR_Data[1] != 0))
					{
						break;
					}
					delay(5);
				}
				// Motor_Slow_Go();
			}
			else
			{
				Motor_Thunder.Set_Car_Speed_Direction(15, -100);
			}
		}
		else if (IR_Data[0] == 0) //Serial.printf("* 偏左，右转 \n");
		{
			if ((line_state == 1) | (line_state == 3)) //从左转过来的需要更新时间
			{
				Motor_Thunder.Set_Car_Speed_Direction(15, -50);
				Line_last_time = millis(); // 不改变运动状态
				line_out_flag = 1;
				continue; // 这种情况是越界的情况，保持前运动状态继续运动，直到状态回归
			}
			line_out_flag = 0;
			// 如果上次偏右时间不够200ms，那可能这里是处于直线状态的，转弯幅度要小
			if (current_time < R_last_time + MAYBE_STRAIGHT_DIRECTION)
			{
				Motor_Thunder.Set_Car_Speed_Direction(20, 10);
			}
			else
			{
				Motor_Thunder.Set_Car_Speed_Direction(20, 20);
			}

			if (line_state == 2)
			{
				//一直为偏左出线，所以一直更新状态时间, 不改变运动状态
				Line_last_time = millis();
			}
			else if ((current_time - WAIT_DIRECTION_COMFIRM_TIME) > Line_last_time && line_state == 4)
			{
				line_state = 2; // 偏左时间已经超过 50ms
				L_last_time = current_time;
			}
			else
			{
				line_state = 4; // 偏左时间小于 50ms，急转偏右运动一小段时间
				Motor_Thunder.Set_Car_Speed_Direction(25, 30);
			}
		}
		else if (IR_Data[1] == 0) //Serial.printf("* 偏右，左转 \n");
		{
			if ((line_state == 2) | (line_state == 4)) //从右转过来的需要更新时间
			{
				Motor_Thunder.Set_Car_Speed_Direction(15, 50);
				Line_last_time = millis(); // 不改变运动状态
				line_out_flag = 2;
				continue; // 保持前运动状态继续运动
			}
			line_out_flag = 0;
			// 如果上次偏右时间不够200ms，那可能这里是处于直线状态的，转弯幅度要小
			if (current_time < L_last_time + MAYBE_STRAIGHT_DIRECTION)
			{
				Motor_Thunder.Set_Car_Speed_Direction(20, -10);
			}
			else
			{
				Motor_Thunder.Set_Car_Speed_Direction(20, -20);
			}

			if (line_state == 1)
			{
				//一直为偏右，所以一直更新状态时间, 不改变运动状态
				Line_last_time = millis();
			}
			else if ((current_time - WAIT_DIRECTION_COMFIRM_TIME) > Line_last_time && line_state == 3)
			{
				line_state = 1; // 偏右时间已经超过 50ms
				R_last_time = current_time;
			}
			else
			{
				Motor_Thunder.Set_Car_Speed_Direction(25, -30);
				line_state = 3; // 偏右时间小于 50ms，急转偏右运动一小段时间
			}
		}
		else //Serial.printf("* 线上 \n");
		{
			if (line_out_flag == 0)
			{
				Motor_Thunder.Set_Car_Speed_Direction(25, 0);
				line_state = 0;
			}
			Line_last_time = millis();
		}

		////////////////////////////////// (start)巡线时LED画面、播放的声音 //////////////////////////////////
		if ((current_time - 50) > Line_last_led_time)
		{
			if ((LED_counter > 98) & (line_state != 5))
			{
				LED_counter = 77;
			}
			else if ((LED_counter > 100) & (line_state == 5))
			{
				LED_counter = 99;
				Line_last_sound_time = millis();
			}

			Display_Screen.Play_Thunder_Picture(LED_counter); //单色点阵图案
			LED_counter++;

			Line_last_led_time = millis();
		}

		if (((current_time - 19000) > Line_last_sound_time) & (current_time > 19000))
		{
			// Speaker_Thunder.Play_Song(139);
			Line_last_sound_time = millis();
		}
		////////////////////////////////// (end)巡线时LED画面、播放的声音 //////////////////////////////////
	}
	line_tracing_running = false;
	Speaker_Thunder.Play_Song(130);
}
#else
/* 
 * 后驱电机的巡线
 * 1、控制使用了 方向PID、速度PID、分级减速的方式进行
 *    a/将传感器状态值输进状态机，以获取到位置状态；
 *    b/将位置状态转变为一个偏移量数值，以偏移量作为参考量
 *    c/以一个固定速度行驶，方向PID控制两轮间的差速，以偏移量的变化量进行分级减速（后面可以改一版：减速也可以进行PID控制）
 *      ，得到最后的速度值，然后速度PID以速度值作为参考量进行控制电机功率。
 * 2、车型：电机安装在后面，传感器安装在前面
 * 3、传感器安装高度升为 1.5cm
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Line_Tracing(void)
{
	history_data[0] = 0xFF;
	history_data[1] = 0xFF;

#ifdef DEBUG_LINE_TRACING
	// 等待串口输入新的PID参数
	uint32_t beginWaitTime;
	beginWaitTime = millis();
	Serial.print("Kp Ki Ki Kd Kp Ki Kd: ");
	//5.000 0.100 0.010 10.00 10.00 0.500 0.200
	while (Serial.available() < 41)
	{
		current_time = millis();
		if (current_time > beginWaitTime + 5000)
		{
			break;
		}
	}
	if (Serial.available() >= 41)
	{
		LINE_TRACE_P = Serial.parseFloat();
		LINE_TRACE_ACCE_I = Serial.parseFloat();
		LINE_TRACE_DECE_I = Serial.parseFloat();
		LINE_TRACE_SPEED_D = Serial.parseFloat();
		LINE_DEVIATION_P = Serial.parseFloat();
		LINE_DEVIATION_I = Serial.parseFloat();
		LINE_DEVIATION_D = Serial.parseFloat();
	}
	Serial.printf("\nK: %f %f\n", LINE_TRACE_P, LINE_DEVIATION_D);
#endif

	Line_last_time = millis();
	Line_last_led_time = millis();
	Line_last_sound_time = millis();
	int LED_counter = 99;

	Wait_Line_Location();
	line_state = LINE_STATE_START;

	// Speaker_Thunder.Play_Song(131);
	while (Rx_Data[1] == 1)
	{
		Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
		history_data[0] = history_data[0] << 1 | IR_Data[0];
		history_data[1] = history_data[1] << 1 | IR_Data[1];

		current_time = millis();
		if (current_time - Line_last_time > LINE_TRACE_PERIOD_MS)
		{
			// Serial.printf("*** Left: %d   Right: %d ***\n", IR_Data[0], IR_Data[1]);
			if ((IR_Data[0] == 0) & (IR_Data[1] == 0)) // 两点白
			{
				switch (line_state)
				{
				case LINE_STATE_LEFT: // 偏左状态
					line_state = LINE_STATE_LEFT_DEEP;
					break;
				case LINE_STATE_RIGHT: // 偏右状态
					line_state = LINE_STATE_RIGHT_DEEP;
					break;
				case LINE_STATE_LEFT_DEEP: // 太偏左状态
					line_state = LINE_STATE_LEFT_DEEP;
					break;
				case LINE_STATE_RIGHT_DEEP: // 太偏右状态
					line_state = LINE_STATE_RIGHT_DEEP;
					break;
				case LINE_STATE_LEFT_OVER: // 左向出界
					line_state = LINE_STATE_LEFT_DEEP;
					break;
				case LINE_STATE_RIGHT_OVER: // 右向出界
					line_state = LINE_STATE_RIGHT_DEEP;
					break;
				case LINE_STATE_START:	// 刚启动状态
				case LINE_STATE_STRAIGHT: // 直行状态
				case LINE_STATE_LOST:	 // 迷失状态，需要尝试左右旋转寻找出路
				default:
					line_state = LINE_STATE_LOST;
					break;
				}
			}
			else if (IR_Data[0] == 0) // 左点白
			{
				switch (line_state)
				{
				case LINE_STATE_START: // 刚启动状态
					line_state = LINE_STATE_LEFT;
					break;
				case LINE_STATE_STRAIGHT: // 直行状态
					line_state = LINE_STATE_LEFT;
					break;
				case LINE_STATE_LEFT: // 偏左状态
					line_state = LINE_STATE_LEFT;
					break;
				case LINE_STATE_LEFT_DEEP: // 太偏左状态
					line_state = LINE_STATE_LEFT;
					break;
				case LINE_STATE_RIGHT_OVER: // 右向出界
					line_state = LINE_STATE_RIGHT_OVER;
					break;
				case LINE_STATE_RIGHT_DEEP: // 太偏右状态
					line_state = LINE_STATE_RIGHT_OVER;
					break;
				case LINE_STATE_LOST:	  // 迷失状态，需要尝试左右旋转寻找出路
				case LINE_STATE_RIGHT:	 // 偏右状态
				case LINE_STATE_LEFT_OVER: // 左向出界
				default:
					line_state = LINE_STATE_LOST;
					break;
				}
			}
			else if (IR_Data[1] == 0) // 右点白
			{
				switch (line_state)
				{
				case LINE_STATE_START: // 刚启动状态
					line_state = LINE_STATE_RIGHT;
					break;
				case LINE_STATE_STRAIGHT: // 直行状态
					line_state = LINE_STATE_RIGHT;
					break;
				case LINE_STATE_RIGHT: // 偏右状态
					line_state = LINE_STATE_RIGHT;
					break;
				case LINE_STATE_LEFT_DEEP: // 太偏左状态
					line_state = LINE_STATE_LEFT_OVER;
					break;
				case LINE_STATE_RIGHT_DEEP: // 太偏右状态
					line_state = LINE_STATE_RIGHT;
					break;
				case LINE_STATE_LEFT_OVER: // 左向出界
					line_state = LINE_STATE_LEFT_OVER;
					break;
				case LINE_STATE_LOST:		// 迷失状态，需要尝试左右旋转寻找出路
				case LINE_STATE_LEFT:		// 偏左状态
				case LINE_STATE_RIGHT_OVER: // 右向出界
				default:
					line_state = LINE_STATE_LOST;
					break;
				}
			}
			else // 两点黑
			{
				switch (line_state)
				{
				case LINE_STATE_START: // 刚启动状态
					line_state = LINE_STATE_STRAIGHT;
					break;
				case LINE_STATE_STRAIGHT: // 直行状态
					line_state = LINE_STATE_STRAIGHT;
					break;
				case LINE_STATE_LEFT: // 偏左状态
					line_state = LINE_STATE_STRAIGHT;
					break;
				case LINE_STATE_RIGHT: // 偏右状态
					line_state = LINE_STATE_STRAIGHT;
					break;
				case LINE_STATE_LEFT_DEEP:  // 太偏左状态
				case LINE_STATE_RIGHT_DEEP: // 太偏右状态
				case LINE_STATE_LEFT_OVER:  // 左向出界
					line_state = LINE_STATE_LEFT_OVER;
					break;
				case LINE_STATE_RIGHT_OVER: // 右向出界
					line_state = LINE_STATE_RIGHT_OVER;
					break;
				case LINE_STATE_LOST: // 迷失状态，需要尝试左右旋转寻找出路
				default:
					line_state = LINE_STATE_LOST;
					break;
				}
			}

			// Serial.printf("*** line state: %d *** \n", line_state);
			Calculate_Motor_Power();
			Line_last_time = millis();
		}

		////////////////////////////////// 巡线时LED画面 //////////////////////////////////
		if ((current_time - 50) > Line_last_led_time)
		{
			if ((LED_counter > 98) & (line_state != LINE_STATE_LOST))
			{
				LED_counter = 77;
			}
			else if ((LED_counter > 100) & (line_state == LINE_STATE_LOST))
			{
				LED_counter = 99;
				Line_last_sound_time = millis();
			}

			Display_Screen.Play_Thunder_Picture(LED_counter); //单色点阵图案
			LED_counter++;

			Line_last_led_time = millis();
		}

		////////////////////////////////// 巡线时播放的声音 //////////////////////////////////
		/*    if (((current_time - 19000) > Line_last_sound_time) & (current_time > 19000))
	{
	  Speaker_Thunder.Play_Song(139);
	  Line_last_sound_time = millis();
	} */
	}
	Speaker_Thunder.Play_Song(130);
}
#endif

// 开机动画/声效
void BELL_THUNDER::Start_Show(void)
{
	Speaker_Thunder.Play_Song(79); //声音编号79

	Display_Screen.Play_Thunder_Picture(1); //单色点阵图案
	delay(200);
	Display_Screen.Play_Thunder_Picture(2); //单色点阵图案
	delay(200);
	Display_Screen.Play_Thunder_Picture(3); //单色点阵图案
	delay(200);
	Display_Screen.Play_Thunder_Picture(4); //单色点阵图案
	delay(200);
	Display_Screen.Play_Thunder_Picture(5); //单色点阵图案
	delay(200);
	Display_Screen.Play_Thunder_Picture(4); //单色点阵图案
	delay(200);
	Display_Screen.Play_Thunder_Picture(5); //单色点阵图案
	delay(200);
	Display_Screen.Play_Thunder_Picture(4); //单色点阵图案
	delay(200);
}

/* 
 * 设置need_communication 的接口函数
 * 
 * @parameters: 
 * @return: 
 */
void BELL_THUNDER::Set_Need_Communication(bool flag)
{
	need_communication = flag;
}

// 等待蓝牙连接动画 (有串口数据也跳出)
void BELL_THUNDER::Wait_Communication(void)
{
	uint32_t show_index = 6;
	while ((deviceConnected == false) && (Usart_Communication == 0) && (need_communication == true))
	{
		System_Task.Remove_Flush_Task(FLUSH_MATRIX_LED);
		System_Task.Remove_Flush_Task(FLUSH_CHARACTER_ROLL);
		if (lowpower_flag == 0)
		{
			Display_Screen.Play_Thunder_Picture(show_index); //单色点阵图案
			if (show_index == 13)
			{
				show_index = 6;
			}
			else
			{
				show_index++;
			}
			delay(200);
		}
		if (Serial.available())
		{
			Usart_Communication = 1;
		}
	}
	System_Task.Set_Flush_Task(FLUSH_MATRIX_LED);
	System_Task.Set_Flush_Task(FLUSH_CHARACTER_ROLL);
}

// 获取串口指令到 RX_Data
void BELL_THUNDER::Get_Serial_Command()
{
	while (Serial.available())
	{
		if (uart_recv_finish == 1)
		{
			break;
		}
		Rx_Data[uart_position] = Serial.read(); //Serial.parseInt();  //读取整数

#ifdef DEBUG_UART_COMMAND
		Serial.printf("* recv UART cmd: %x *\n", Rx_Data[0]);
#endif
		int i;
		switch (Rx_Data[0])
		{
		case UART_GENERAL_BLE_NAME: // 蓝牙命名指令数据
		{
			add_sum = 0;
			for (i = 1; i < BLE_NAME_SIZE; i++)
			{
				if (Serial.available())
				{
					Rx_Data[i] = Serial.read();
				}
				else
				{
					break;
				}
				BLE_Name_Data[i - 1] = Rx_Data[i];

				add_sum += Rx_Data[i - 1];
			}
			BLE_Name_Data[i - 2] = '\0';

			if (add_sum != Rx_Data[i - 1])
			{
				Serial.printf(" # %x add_sum error \n", Rx_Data[0]);
				Reset_Rx_Data();
			}
			else
			{
				uart_recv_finish = 1; // 接收完成，标志可以进行处理
			}
			break;
		}
		case UART_GENERAL_LEFT_RGBLED: //刷新左侧彩色灯
		{
			add_sum = Rx_Data[0];
			for (i = 1; i < 19; i++)
			{
				Bell_Thunder.LED_Color_BUFF1[i - 1] = Serial.read();
				add_sum += Bell_Thunder.LED_Color_BUFF1[i - 1];
				// Serial.printf(": %x \n",Bell_Thunder.LED_Color_BUFF1[i-1]);
			}

			if (add_sum != Serial.read())
			{
				Serial.printf(" # %x add_sum error \n", Rx_Data[0]);
				Reset_Rx_Data();
			}
			else
			{
				uart_recv_finish = 1; // 接收完成，标志可以进行处理
			}
			break;
		}
		case UART_GENERAL_RIGHT_RGBLED: //刷新右侧彩色灯
		{
			add_sum = Rx_Data[0];
			for (i = 1; i < 19; i++)
			{
				Bell_Thunder.LED_Color_BUFF2[i - 1] = Serial.read();
				add_sum += Bell_Thunder.LED_Color_BUFF2[i - 1];
				// Serial.printf(": %x \n",Bell_Thunder.LED_Color_BUFF2[i-1]);
			}

			if (add_sum != Serial.read())
			{
				Serial.printf(" # %x add_sum error \n", Rx_Data[0]);
				Reset_Rx_Data();
			}
			else
			{
				uart_recv_finish = 1; // 接收完成，标志可以进行处理
			}
			break;
		}
		case UART_GENERAL_DEBUG_PRE_LED: //单色点阵灯一次性刷新前半部分灯
		{
			add_sum = Rx_Data[0];
			for (i = 1; i < 15; i++)
			{
				Bell_Thunder.LED_BUFF_Dot[i] = Serial.read();
				add_sum += Bell_Thunder.LED_BUFF_Dot[i];
			}

			if (add_sum != Serial.read())
			{
				Serial.printf(" # %x add_sum error \n", Rx_Data[0]);
				Reset_Rx_Data();
			}
			else
			{
				uart_recv_finish = 1; // 接收完成，标志可以进行处理
			}
			break;
		}
		case UART_GENERAL_DEBUG_SUF_LED: //单色点阵灯一次性刷新后半部分灯
		{
			add_sum = Rx_Data[0];
			for (i = 1; i < 15; i++)
			{
				Bell_Thunder.LED_BUFF_Dot[i + 14] = Serial.read();
				add_sum += Bell_Thunder.LED_BUFF_Dot[i + 14];
			}

			if (add_sum != Serial.read())
			{
				Serial.printf(" # %x add_sum error \n", Rx_Data[0]);
				Reset_Rx_Data();
			}
			else
			{
				uart_recv_finish = 1; // 接收完成，标志可以进行处理
			}
			break;
		}
		case UART_CALL_SPECIAL_FUNCTION:
		{
			if (uart_position == 0)
			{
				add_sum = Rx_Data[0];
				while (Serial.available() == 0)
					; // 等待第二个字节：长度信息
				uart_position++;
				Rx_Data[COMMUNICATION_DATA_LOC_LENGTH] = Serial.read();
			}

			if (Rx_Data[COMMUNICATION_DATA_LOC_LENGTH] >= 3 && Rx_Data[COMMUNICATION_DATA_LOC_LENGTH] <= COMMUNICATION_DATA_LENGTH_MAX)
			{
				add_sum += Rx_Data[uart_position];
				uart_position++;
				for (; uart_position < Rx_Data[COMMUNICATION_DATA_LOC_LENGTH]; uart_position++)
				{
					if (Serial.available())
					{
						Rx_Data[uart_position] = Serial.read();
						if (uart_position < Rx_Data[COMMUNICATION_DATA_LOC_LENGTH] - 1)
						{
							add_sum += Rx_Data[uart_position];
						}
					}
					else
					{
						break;
					}
				}
				// 指令包接收完毕，进行校验
				if (uart_position == Rx_Data[COMMUNICATION_DATA_LOC_LENGTH])
				{
					if (add_sum != Rx_Data[Rx_Data[COMMUNICATION_DATA_LOC_LENGTH] - 1])
					{
						Serial.printf(" # %x add_sum error=%x\n", Rx_Data[0], add_sum);
						Reset_Rx_Data();
						uart_position = 0; // 校验失败，复位接收
					}
					else
					{
						Rx_Data[COMMUNICATION_DATA_LENGTH_MAX] = Rx_Data[COMMUNICATION_DATA_LOC_LENGTH];
						uart_recv_finish = 1; // 接收完成，标志可以进行处理
					}
				}
			}
			else
			{
				uart_position = 0; // 格式不对，复位接收
			}

			break;
		}
		case UART_BARBETTE_CTRL:
		case UART_BARBETTE_INFO:
		{
			uart_position++;
			while (Serial.available() > 0 && uart_position < (Rx_Data[2] + 5))
			{
				Rx_Data[uart_position] = (uint8_t)Serial.read();
				uart_position++;
				if (uart_position == (Rx_Data[2] + 5))
				{
					break;
				}
			}
			if (uart_position == (Rx_Data[2] + 5))
			{
				uint16_t check_sum = 0;
				uint16_t compute_crc = 0;

				uart_position = 0;
				check_sum = Rx_Data[Rx_Data[2] + 3];
				check_sum = (check_sum << 8) | Rx_Data[Rx_Data[2] + 4];
				compute_crc = crc16(Rx_Data, Rx_Data[2] + 3);
				if (compute_crc == check_sum)
				{
					uart_recv_finish = 1;
				}
				else
				{
					Serial.printf(" # %x crc16 error \n", Rx_Data[0]);
					memset(Rx_Data, 0, 20);
				}
			}

			break;
		}
		case UART_GENERAL_IR_SENSOR:
		case UART_GENERAL_US_SENSOR:
		case UART_GENERAL_COLOR_SENSOR:
		case UART_GENERAL_COLOR_CARD:
		case UART_GENERAL_BATTERY_SENSOR:
		case UART_GENERAL_VERSION_INFO:
		case UART_GENERAL_SEARCH_LINE:
		case UART_GENERAL_SERVO_CTRL:
		case UART_GENERAL_PLAY_VOICE:
		case UART_GENERAL_MOTOR_SINGLE:
		case UART_GENERAL_MOTOR_DOUBLE:
		case UART_GENERAL_MOTOR_SINGLE_PID:
		case UART_GENERAL_MOTOR_DOUBLE_PID:
		case UART_GENERAL_GET_MOTOR_SPEED:
		case UART_GENERAL_SINGLE_RGBLED:
		case UART_GENERAL_DEBUG_LED:
		case UART_GENERAL_PICTURE_LED:
		case UART_GENERAL_ANIMATION_LED:
		{
			// 老版协议的固定长度为 6 = 1+5
			if (Serial.available() == 5)
			{
				Rx_Data[1] = Serial.read();
				Rx_Data[2] = Serial.read();
				Rx_Data[3] = Serial.read();
				Rx_Data[4] = Serial.read();
				Rx_Data[5] = Serial.read();
			}
			else
			{
				// Serial.println("flush start");
				while (Serial.available() > 0)
				{
					Rx_Data[6] = Serial.read();
				}
				// Serial.println("flush end");
				Reset_Rx_Data();
				break;
			}

			if (Rx_Data[5] != (uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]))
			{
				Serial.printf(" # %x add_sum error \n", Rx_Data[0]);
				while (Serial.available() > 0)
				{
					Rx_Data[6] = Serial.read();
				}
				Reset_Rx_Data();
			}
			else
			{
				uart_recv_finish = 1; // 接收完成，标志可以进行处理
			}

			break;
		}
		default: // 其他包头，清除Serial 缓存
		{
			// Serial.println("flush start");
			while (Serial.available() > 0)
			{
				Rx_Data[6] = Serial.read();
			}
			// Serial.println("flush end");

			break;
		}
		}
	}
}

// 通信确认，蓝牙
void BELL_THUNDER::Check_BLE_Communication(void)
{
	// BLE已经过去到指令
	if (Rx_Data[0] != 0)
	{
		Check_Protocol();
		if (Tx_Data[0] != 0)
		{
			Tx_Data[5] = Tx_Data[0] + Tx_Data[1] + Tx_Data[2] + Tx_Data[3] + Tx_Data[4];
			BLE_ThunderGo.Tx_BLE(Tx_Data, 6); //通过蓝牙发送数据;参数1 --> 数据数组；参数2 -->字节数

			for (uint32_t i = 0; i < COMMUNICATION_FIXED_LENGTH_MAX; i++)
			{
				Serial.printf("%x ", Tx_Data[i]);
			}
			Serial.println();
			Tx_Data[0] = 0;
		}
		Reset_Rx_Data();
	}
}
// 通信确认，串口
void BELL_THUNDER::Check_UART_Communication(void)
{
	// 检查并等待蓝牙连接
	Wait_Communication();

	Get_Serial_Command();
	if (uart_recv_finish == 1 && !ble_command_busy)
	{
		Check_Protocol();
		uart_position = 0;
		uart_recv_finish = 0;
		if (Tx_Data[0] != 0)
		{
			Tx_Data[5] = Tx_Data[0] + Tx_Data[1] + Tx_Data[2] + Tx_Data[3] + Tx_Data[4];

			//串口返回数据
			Serial.flush();
			Serial.write(Tx_Data[0]);
			Serial.write(Tx_Data[1]);
			Serial.write(Tx_Data[2]);
			Serial.write(Tx_Data[3]);
			Serial.write(Tx_Data[4]);
			Serial.write(Tx_Data[5]);
			Serial.flush();

			Tx_Data[0] = 0;
		}
		Reset_Rx_Data();
	}
}

// 协议解析
void BELL_THUNDER::Check_Protocol(void)
{
	switch (Rx_Data[0])
	{
	case 0x00:
		// Serial.printf("SSSSSSSSSS 串口乱入 SSSSSSSSSS\n");
		break;

	case UART_GENERAL_IR_SENSOR: // IR数据读取
		Get_IR_Data(IR_Data);	//更新IR数据
		Tx_Data[0] = UART_GENERAL_IR_SENSOR;
		Tx_Data[1] = IR_Data[0];
		Tx_Data[2] = IR_Data[1];
		Tx_Data[3] = 0x00;
		Tx_Data[4] = 0x00;
		// Serial.printf("SSSSSSSSSS ___ IR_Data[0] : %d ___ IR_Data[1] : %d ___ SSSSSSSSSS\n",IR_Data[0],IR_Data[1]);
		break;

	case UART_GENERAL_US_SENSOR:			 // 获取US数据 16进制两个8位
		Sensor_Ultrasonic.Get_Data(US_Data); //更新US数据
		Tx_Data[0] = UART_GENERAL_US_SENSOR;
		Tx_Data[1] = US_Data[0];
		Tx_Data[2] = US_Data[1];
		Tx_Data[3] = 0x00;
		Tx_Data[4] = 0x00;
		// Serial.printf("SSSSSSSSSS ___ US_Data[0] : %x ___ US_Data[1] : %x ___ SSSSSSSSSS\n",US_Data[0],US_Data[1]);
		break;

	case UART_GENERAL_COLOR_SENSOR: // 获取颜色传感器数据 RGBC分4个8位传
		Sensor_Color.Get_RGBC_Data(RGBC);
		Sensor_Color.RGBtoHSV(RGBC, HSV); // 计算HSV

		Tx_Data[0] = UART_GENERAL_COLOR_SENSOR;
		Tx_Data[1] = RGBC[0] >> 4; //R >> 8
		Tx_Data[2] = RGBC[1] >> 4; //G >> 8
		Tx_Data[3] = RGBC[2] >> 4; //B >> 8
		// Tx_Data[4] = RGBC[3] >> 4;  //C >> 8
		Tx_Data[4] = (uint8_t)HSV[0]; //H
		break;

	case UART_GENERAL_COLOR_CARD: // 获取颜色传感器数据 RGBC分4个8位传
		Sensor_Color.Get_RGBC_Data(RGBC);

		Tx_Data[0] = UART_GENERAL_COLOR_CARD;
		Tx_Data[1] = Sensor_Color.Colour_Recognition(RGBC);
		break;

	case UART_GENERAL_BATTERY_SENSOR: //获取电池电压数据
		// Get_Battery_Data();
		Tx_Data[0] = UART_GENERAL_BATTERY_SENSOR;
		Tx_Data[1] = Battery_Value >> 8; //读取一次电池电压
		Tx_Data[2] = Battery_Value;
		Tx_Data[3] = 0;
		Tx_Data[4] = 0;
		break;

	case UART_GENERAL_VERSION_INFO: //获取固件版本号
		Tx_Data[0] = UART_GENERAL_VERSION_INFO;

		// 复制版本号的“整数”和“小数” 到 发送Buffer
		Tx_Data[1] = Version_FW[0];
		Tx_Data[2] = Version_FW[1];
		Tx_Data[3] = Version_FW[2];
		Tx_Data[4] = Version_FW[3];

		break;

	case UART_GENERAL_BLE_NAME:		  //蓝牙命名
		BLE_ThunderGo.Set_BLE_Name(); //从地址0开始写入命名的蓝牙
		break;

	case UART_GENERAL_SERVO_CTRL:			//控制舵机
		Motor_Servo.Servo_Turn(Rx_Data[1], Rx_Data[2]); //参数1：1-->机械臂，2-->机械爪；参数2：角度[%](0~100)
		break;

	case UART_GENERAL_PLAY_VOICE:			   //控制声音播放
		Speaker_Thunder.Set_Sound_Level(Rx_Data[2]);
		Speaker_Thunder.Play_Song(Rx_Data[1]); //播放第编号段音频
		break;

	case UART_GENERAL_MOTOR_SINGLE: //控制单个电机
		Disable_En_Motor();
		Motor_Thunder.Motor_Move(Rx_Data[1], Rx_Data[2], Rx_Data[3]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
		break;

	case UART_GENERAL_MOTOR_DOUBLE: //控制两个电机
		Disable_En_Motor();

		Motor_Thunder.Motor_Move(1, Rx_Data[1], Rx_Data[2]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
		Motor_Thunder.Motor_Move(2, Rx_Data[3], Rx_Data[4]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
		break;

	case UART_GENERAL_MOTOR_SINGLE_PID: //控制单个闭环电机
		if (Rx_Data[1] == 1)
		{
			if (Rx_Data[3] == 1)
			{
				Motor_Thunder.Set_Target(1, Rx_Data[2]);
			}
			else
			{
				Motor_Thunder.Set_Target(1, (-1) * Rx_Data[2]);
			}
		}
		else
		{
			if (Rx_Data[3] == 1)
			{
				Motor_Thunder.Set_Target(2, Rx_Data[2]);
			}
			else
			{
				Motor_Thunder.Set_Target(2, (-1) * Rx_Data[2]);
			}
		}
		break;

	case UART_GENERAL_MOTOR_DOUBLE_PID: //控制两个闭环电机
		if (Rx_Data[2] == 1)
		{
			Motor_Thunder.Set_Target(1, Rx_Data[1]);
		}
		else
		{
			Motor_Thunder.Set_Target(1, (-1) * Rx_Data[1]);
		}

		if (Rx_Data[4] == 1)
		{
			Motor_Thunder.Set_Target(2, Rx_Data[3]);
		}
		else
		{
			Motor_Thunder.Set_Target(2, (-1) * Rx_Data[3]);
		}
		break;

	case UART_GENERAL_GET_MOTOR_SPEED: //获取当前电机速度(编码器计数值)  需要在Enable_En_Motor()状态下
		L_Speed = Motor_Thunder.Get_Speed_simple(1);
		R_Speed = Motor_Thunder.Get_Speed_simple(2);

		Tx_Data[0] = UART_GENERAL_GET_MOTOR_SPEED;
		if (L_Speed >= 0)
		{
			Tx_Data[1] = L_Speed;
			Tx_Data[2] = 1;
		}
		else
		{
			Tx_Data[1] = (-1) * L_Speed;
			Tx_Data[2] = 2;
		}

		if (R_Speed >= 0)
		{
			Tx_Data[3] = R_Speed;
			Tx_Data[4] = 1;
		}
		else
		{
			Tx_Data[3] = (-1) * R_Speed;
			Tx_Data[4] = 2;
		}
		// Serial.printf("L_Speed:%d; R_Speed:%d; \n",Motor_Thunder.Get_Speed_simple(1),Motor_Thunder.Get_Speed_simple(2));
		break;

	case UART_GENERAL_SINGLE_RGBLED:											//控制单个彩色灯颜色
		LED_Color.Set_LED_Data(Rx_Data[1], Rx_Data[2], Rx_Data[3], Rx_Data[4]); //第几个灯(1开始)，R,G,B
		LED_Color.LED_Updata();													//按照现有数据刷新
		break;

	case UART_GENERAL_LEFT_RGBLED:											  //控制左侧彩色灯颜色
		LED_Color.Set_LEDs_Data(1, LED_Color_BUFF1, sizeof(LED_Color_BUFF1)); //写入多个寄存器数据
		LED_Color.LED_Updata();												  //按照现有数据刷新
		break;
	case UART_GENERAL_RIGHT_RGBLED:											  //控制右侧彩色灯颜色
		LED_Color.Set_LEDs_Data(7, LED_Color_BUFF2, sizeof(LED_Color_BUFF2)); //写入多个寄存器数据
		LED_Color.LED_Updata();												  //按照现有数据刷新
		break;

	case UART_GENERAL_DEBUG_LED: //控制单色点阵灯开关
		LED_BUFF_Dot[Rx_Data[1]] = Rx_Data[2];
		Display_Screen.Play_LEDs(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
		break;

	case UART_GENERAL_PICTURE_LED: //显示预设的单色点阵灯图案
		Display_Screen.Play_Thunder_Picture(Rx_Data[1]);
		break;

	case UART_GENERAL_DEBUG_PRE_LED: //单色点阵灯一次性刷新前半部分灯
		Display_Screen.Play_LEDs(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
		break;
	case UART_GENERAL_DEBUG_SUF_LED: //单色点阵灯一次性刷新后半部分灯
		Display_Screen.Play_LEDs(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
		break;

	case UART_GENERAL_ANIMATION_LED: //内置单色点阵图案动画显示命令
		Display_Screen.Play_Animation(Rx_Data[1]);
		break;

	case 0x61: //_______________________ Demo ___________________
		line_tracing_running = false;
		break;
	case UART_GENERAL_SEARCH_LINE: //_______________________ Demo ___________________
		if (Rx_Data[1] == 1)
		{
			// Serial.printf("* 巡线 *\n");
			// 使用开环控制电机，然后在巡线里面 以偏离黑线的时间长度作为参量 做速度闭环控制
			Stop_All();
			if (Rx_Data[2] == 0)
			{ // 轮胎版
				Line_H_Speed = 70;
				Line_M_Speed = 60;
				Line_L_Speed = 40;
				Line_B_Speed = -40;
			}
			else if (Rx_Data[2] == 1)
			{ // 履带版
				Line_H_Speed = 90;
				Line_M_Speed = 80;
				Line_L_Speed = 55;
				Line_B_Speed = -55;
			}
			line_tracing_running = true;
		}
		else if (Rx_Data[1] == 0)
		{
			line_tracing_running = false;
		}
		break;
	case UART_CALL_SPECIAL_FUNCTION:
		for (int i = 0; i < Rx_Data[COMMUNICATION_DATA_LENGTH_MAX]; i++)
		{
			Serial.printf("%x ", Rx_Data[i]);
		}
		Serial.println();
		break;

	case UART_BARBETTE_CTRL:
		if (Rx_Data[1] == 0x01)
		{
			int speed_x, speed_y;
			int power_left, power_right;
			int max_power = 0;

			speed_y = (int16_t)(((float)Rx_Data[3] - 126) / 3.0) * (float)(3 - (float)Rx_Data[7]);
			speed_x = (int16_t)(((float)Rx_Data[6] - 126) / 3.0) * (float)(3 - (float)Rx_Data[7]);
			power_left = speed_y + speed_x;
			power_right = speed_y - speed_x;

			max_power = abs(speed_y) + abs(speed_x);
			if (max_power < 126)
			{
				max_power = 126;
			}
			power_left = map(power_left, -max_power, max_power, -255, 255);
			power_right = map(power_right, -max_power, max_power, -255, 255);

			Motor_Thunder.Set_Motor_Output(1, power_left);
			Motor_Thunder.Set_Motor_Output(2, power_right);

			static bool last_status = 0;
			if (Rx_Data[8] != last_status)
			{
				Barbette_Thunder.Auto_Fire(Rx_Data[8]);

				last_status = Rx_Data[8];
			}
			if ((Rx_Data[5] > 129) || (Rx_Data[5] < 125))
			{
				Barbette_Thunder.Adjust_Pos(Rx_Data[5]);
			}
		}
		break;
	case UART_BARBETTE_INFO:
		uint8_t return_data[20];
		uint16_t get_bullet, compute_crc;
		uint32_t get_used_time;

		if (Rx_Data[1] == 0x01)
		{
			return_data[0] = 0xbd;
			return_data[1] = 0x01;
			return_data[2] = 0x02;
			get_bullet = Barbette_Thunder.Get_Bullet();
			return_data[3] = (get_bullet >> 8) & 0xff;
			return_data[4] = get_bullet & 0xff;
			compute_crc = crc16(return_data, 5);
			return_data[5] = (compute_crc >> 8) & 0xff;
			return_data[6] = compute_crc & 0xff;
			Serial.write(return_data, 7);
			memset(return_data, 0, 20);
		}
		else if (Rx_Data[1] == 0x02)
		{
			return_data[0] = 0xbd;
			return_data[1] = 0x02;
			return_data[2] = 0x04;
			get_used_time = Barbette_Thunder.Get_Used_Time();
			return_data[3] = (get_used_time >> 24) & 0xff;
			return_data[4] = (get_used_time >> 16) & 0xff;
			return_data[5] = (get_used_time >> 8) & 0xff;
			return_data[6] = get_used_time & 0xff;
			compute_crc = crc16(return_data, 7);
			return_data[7] = (compute_crc >> 8) & 0xff;
			return_data[8] = compute_crc & 0xff;
			Serial.write(return_data, 9);
			memset(return_data, 0, 20);
		}

		break;
	default:
		Serial.printf("# No cmd#\n");
		break;
	}
}

// 清空接收数据
void BELL_THUNDER::Reset_Rx_Data()
{
	memset(Rx_Data, 0, COMMUNICATION_FIXED_LENGTH_MAX);
	ble_command_busy = false;
}

/**
 * @brief: 获取虚拟计时器的时间，单位：毫秒ms
 * 
 * @param timer_index: 计时器序号 1~5
 * @return uint32_t : 返回计时时间，单位：毫秒ms
 */
uint32_t BELL_THUNDER::Get_Virtual_Timer(uint32_t timer_index)
{
	uint32_t ret_value;
	
	CHECK_RANGE(timer_index, 1, 5);

	ret_value = millis() - timer_value[timer_index - 1];
	return ret_value;
}

/**
 * @brief 将虚拟计时器清零，清零后的计时器数值变为0，然后立刻从 0 开始计时，+1/ms
 * 
 * @param timer_index 计时器序号 1~5
 */
void BELL_THUNDER::Reset_Virtual_Timer(uint32_t timer_index)
{
	CHECK_RANGE(timer_index, 1, 5);

	timer_value[timer_index - 1] = millis();
}

/**
 * @brief 由于通信收发需要消耗RAM和CPU线程运行时间，
 * 因此只有在需要用到通信时再打开多机通信，不需要后就关闭多机通信，
 * 调用 Open_Multi_Message() 打开多机通信，调用 Close_Multi_Message() 关闭多机通信，
 * 
 */
void BELL_THUNDER::Open_Multi_Message()
{
	if (Mult_Devices != NULL)
	{
		return;
	}

	Mult_Devices = new MULT_DEVICES();
	Mult_Devices->OpenCommunicate(&recv_int_message);
}

/**
 * @brief 由于通信收发需要消耗RAM和CPU线程运行时间，
 * 因此只有在需要用到通信时再打开多机通信，不需要后就关闭多机通信，
 * 调用 Close_Multi_Message() 关闭多机通信，
 * 
 */
void BELL_THUNDER::Close_Multi_Message()
{
	if (Mult_Devices == NULL)
	{
		return;
	}

	Mult_Devices->CloseCommunicate();
	delete Mult_Devices;
	Mult_Devices = NULL;
}

/**
 * @brief 为了方便使用多机通信，可以将数据传输进行 命名，
 * 支持设置最多 32个 命名变量，名字长度最大 16个字符，
 * 可以调用 InitNameVarInt() 设置 int 变量初始值，否则变量的初始值为 0
 * 
 * @param name 传入被传输的变量名称字符串，
 * @param init_value 传入 int 变量数值
 */
void BELL_THUNDER::InitNameVarInt(const char *name, int init_value)
{
	std::vector<struct_Int_Message>::iterator i;
	for (i = recv_int_message.begin(); i != recv_int_message.end(); i++)
	{
		if (i->message == std::string(name))
		{
			i->value = init_value;
		}
	}
	if (i == recv_int_message.end())
	{
		struct_Int_Message new_message = {std::string(name), init_value};
		recv_int_message.push_back(new_message);
	}
}

/**
 * @brief 为了方便使用多机通信，可以将数据传输进行 命名，
 * 支持设置最多 32个 命名变量，名字长度最大 16个字符，
 * 可以调用 SendNameVarInt() 向某个地址传输一个命名 int变量
 * 
 * @param addr 传入目的地址 0~4
 * @param name 传入被传输的变量名称字符串，
 * @param var_value 传入 int 变量数值
 * @return int 返回错误码，正常发送返回0
 */
int BELL_THUNDER::SendNameVarInt(unsigned char addr, const char *name, int var_value)
{
	if (Mult_Devices == NULL)
	{
		return -1;
	}
	return Mult_Devices->SendNameVarInt(addr, name, var_value);
}

/**
 * @brief 为了方便使用多机通信，可以将数据传输进行 命名，
 * 支持设置最多 32个 命名变量，名字长度最大 16个字符，
 * 可以调用 RecvNameVarInt() 更新命名 int变量，
 * 如果没有该命名数值的更新，则返回旧的数值
 * 
 * @param name 传入被传输的变量名称字符串，
 * @return int 返回最新数值
 */
int BELL_THUNDER::RecvNameVarInt(const char *name)
{
	std::vector<struct_Int_Message>::iterator i;
	for (i = recv_int_message.begin(); i != recv_int_message.end(); i++)
	{
		if (i->message == std::string(name))
		{
			return i->value;
		}
	}

	return 0;
}

int BELL_THUNDER::SendNameVarInt(unsigned char addr, int name, int var_value)
{
	const char *messageName;
	messageName = String(name).c_str();

	return SendNameVarInt(addr, messageName, var_value);
}
int BELL_THUNDER::RecvNameVarInt(int name)
{
	const char *messageName;
	messageName = String(name).c_str();

	return RecvNameVarInt(messageName);
}
void BELL_THUNDER::InitNameVarInt(int name, int init_value)
{
	const char *messageName;
	messageName = String(name).c_str();

	InitNameVarInt(messageName, init_value);
}
