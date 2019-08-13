#include <Arduino.h>
#include "motor_servo.h"


#define CHECK_SERVO_INDEX(index) do{if(index > 2) { \
                                      index = 2; \
                                    } else if (index < 1) { \
                                      index = 1; \
                                    } \
                                  } while(0)

// 舵机初始化配置
void MOTOR_SERVO::Setup_Servo(void)
{
	ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);
	ledcSetup(SERVO_CHANNEL_1, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);

	ledcAttachPin(SERVO_A, SERVO_CHANNEL_0);
	ledcAttachPin(SERVO_B, SERVO_CHANNEL_1);
}

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

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 设置舵机旋转范围（初始值为 -100 ~ 100）
 * 
 * @param servo_index: 舵机接口编号（A:1  B:2）
 * @param max_value: 范围最大值为
 * @param min_value: 范围最小值为
 * @param zero_value: 范围零点
 * @param direction: < 0 时 最小值与最大值互换
 */
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

/**
 * @brief: 控制舵机旋转位置
 * 
 * @param servo: 舵机接口编号（A:1  B:2）
 * @param percent: 舵机旋转范围内的旋转百分比（正数为最大值方向的百分比，负数为最小值方向的百分比）
 */
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
