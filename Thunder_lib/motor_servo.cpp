#include <Arduino.h>
#include "servo_thunder.h"


#define CHECK_SERVO_INDEX(index)      \
	do                                \
	{                                 \
		if (index != 1 && index != 2) \
			return;                   \
	} while (0)

// 舵机初始化配置
void MOTOR_SERVO::Setup_Servo(void)
{
	ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);
	ledcSetup(SERVO_CHANNEL_1, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);

	ledcAttachPin(SERVO_A, SERVO_CHANNEL_0);
	ledcAttachPin(SERVO_B, SERVO_CHANNEL_1);
}

// 舵机角度控制
// 参数1 --> 舵机编号；1-->A口；2-->B口
// 参数2 --> 角度[°]；范围为0-180
void MOTOR_SERVO::Servo_Turn(int servo, float angle)
{
	CHECK_SERVO_INDEX(servo);
	if (angle > 180)
	{
		angle = 180;
	}
	else if (angle < 0)
	{
		angle = 0;
	}

	if (servo == 1) //A口
	{
		ledcWrite(SERVO_CHANNEL_0, Servo_MIN + Servo_Range * angle / 180);
	}
	else if (servo == 2) //B口
	{
		ledcWrite(SERVO_CHANNEL_1, Servo_MIN + Servo_Range * angle / 180);
	}
}

void MOTOR_SERVO::Servo_Percent_Setting(int servo_index,
										 float max_value, float min_value, float zero_value, int direction)
{
	CHECK_SERVO_INDEX(servo_index);
	if (direction < 0)
	{
		servo_percent_zero[servo_index - 1] = zero_value;
		servo_percent_max[servo_index - 1] = min_value;
		servo_percent_min[servo_index - 1] = max_value;
	}
	else
	{
		servo_percent_zero[servo_index - 1] = zero_value;
		servo_percent_max[servo_index - 1] = max_value;
		servo_percent_min[servo_index - 1] = min_value;
	}
}

void MOTOR_SERVO::Servo_Turn_Percent(int servo, float percent)
{
	CHECK_SERVO_INDEX(servo);
	if (percent > 100)
	{
		percent = 100;
	}
	else if (percent < -100)
	{
		percent = -100;
	}

	// 映射舵机转动范围
	if (percent > 0)
	{
		percent = servo_percent_zero[servo - 1] + (servo_percent_max[servo - 1] - servo_percent_zero[servo - 1]) * percent / 100;
	}
	else
	{
		percent = servo_percent_zero[servo - 1] + (servo_percent_zero[servo - 1] - servo_percent_min[servo - 1]) * percent / 100;
	}

	if (servo == 1) //A口
	{
		ledcWrite(SERVO_CHANNEL_0, Servo_MIN + Servo_Range * (percent + 100) / 200);
	}
	else if (servo == 2) //B口
	{
		ledcWrite(SERVO_CHANNEL_1, Servo_MIN + Servo_Range * (percent + 100) / 200);
	}
}
