#include <Arduino.h>
#include "motor_servo.h"
#include "data_type.h"


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

/**
 * @brief: 暂未实现角度控制
 * 
 * @param servo:
 * @param angle:
 */
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

	servo_position[servo - 1] = Servo_MIN + Servo_Range * angle / 180;
	if (servo == A) //A口
	{
		ledcWrite(SERVO_CHANNEL_0, servo_position[0]);
	}
	else if (servo == B) //B口
	{
		ledcWrite(SERVO_CHANNEL_1, servo_position[1]);
	}
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 设置舵机旋转范围（初始值为 -100 ~ 100）
 * 
 * @param servo_index: 舵机接口编号（A:1  B:2）
 * @param max_value: 范围最大值，默认 100
 * @param min_value: 范围最小值，默认 -100
 * @param zero_value: 范围零点，默认 0
 * @param direction: 默认为1，当 < 0 时 最小值与最大值互换
 */
void MOTOR_SERVO::Servo_Percent_Setting(int servo_index,
										 float max_value, float min_value, float zero_value, int direction)
{
	CHECK_SERVO_INDEX(servo_index);
	servo_index = servo_index - 1;
	if (direction < 0)
	{
		servo_percent_zero[servo_index] = zero_value;
		servo_percent_max[servo_index] = min_value;
		servo_percent_min[servo_index] = max_value;
	}
	else
	{
		servo_percent_zero[servo_index] = zero_value;
		servo_percent_max[servo_index] = max_value;
		servo_percent_min[servo_index] = min_value;
	}
}

/**
 * @brief: 控制舵机旋转位置
 * 
 * @param servo: 舵机接口编号（A:1  B:2）
 * @param percent: 舵机旋转范围内的旋转百分比（正数为最大值方向的百分比，负数为最小值方向的百分比）
 * @param speed: 控制速度时会阻塞直到转到固定位置，参数数值范围 1 ~ 100 
 */
void MOTOR_SERVO::Servo_Turn_Percent(int servo, float percent, float speed)
{
	float last_position;
	int servo_index;

	CHECK_SERVO_INDEX(servo);
	CHECK_RANGE(speed, 1, 100);
	servo_index = servo-1;
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
		percent = servo_percent_zero[servo_index] + (servo_percent_max[servo_index] - servo_percent_zero[servo_index]) * percent / 100;
	}
	else
	{
		percent = servo_percent_zero[servo_index] + (servo_percent_zero[servo_index] - servo_percent_min[servo_index]) * percent / 100;
	}

	last_position = servo_position[servo_index];
	servo_position[servo_index] = Servo_MIN + Servo_Range * (percent + 100) / 200;
	if(speed <= 99){
		if(last_position < servo_position[servo_index]) {
			while(last_position < servo_position[servo_index]) {
				if (servo == A) //A口
				{
					ledcWrite(SERVO_CHANNEL_0, last_position);
				}
				else if (servo == B) //B口
				{
					ledcWrite(SERVO_CHANNEL_1, last_position);
				}
				last_position += speed * 0.1;
				delay(10);
			}
		} else if(last_position > servo_position[servo_index]) {
			while(last_position > servo_position[servo_index]) {
				if (servo == A) //A口
				{
					ledcWrite(SERVO_CHANNEL_0, last_position);
				}
				else if (servo == B) //B口
				{
					ledcWrite(SERVO_CHANNEL_1, last_position);
				}
				last_position -= speed * 0.1;
				delay(10);
			}
		}
	}
	if (servo == A) //A口
	{
		ledcWrite(SERVO_CHANNEL_0, servo_position[servo_index]);
	}
	else if (servo == B) //B口
	{
		ledcWrite(SERVO_CHANNEL_1, servo_position[servo_index]);
	}
}
