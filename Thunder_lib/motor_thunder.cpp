#include <motor_thunder.h>
#include <bell_thunder.h>

#include "driver/periph_ctrl.h"
#include "driver/pcnt.h"

// #define PRINT_DEBUG_INFO

/***********************************************************/

void Encoder_Counter_Clear(void);

struct s_encode_amount
{
	int8_t _times; // 需要叠加多少次
	int8_t times; // 已经叠加了多少次 
	int16_t _amount; // 进行叠加编码器数据的总和
	int16_t amount; // 上次叠加编码器数据的总和
};

struct  s_encode_amount encode_amount[2] = {{0,0,0,0},{0,0,0,0}};
// 编码器计数器数值
int16_t Encoder_Counter[2];

void Update_Rotate_Value();
void Get_Encoder_Value();

// PID时间中断
volatile SemaphoreHandle_t Timer_PID_Flag;

//
volatile int32_t rotate_RawValue[2] = {0, 0};

volatile uint32_t PID_Timer_Enable = 0;

void MOTOR_THUNDER::Update_Encoder_Value()
{
	Get_Encoder_Value();
	Update_Rotate_Value();
}

// 配置PID定时器
void MOTOR_THUNDER::Setup_PID_Timer()
{
}

// 移除PID定时器(会影响其它用到此定时器的功能)
void MOTOR_THUNDER::Disable_PID_Timer(void)
{
	PID_Timer_Enable = 0;
}
// 重启PID定时器
void MOTOR_THUNDER::Enable_PID_Timer(void)
{
	PID_Timer_Enable = 1;
}

#if (MOTOR_DRIVER_IC == MOTOR_DRIVER_IC_TB6612)

// 配置电机
void MOTOR_THUNDER::Setup_Motor()
{
	ledcSetup(MOTOR_CHANNEL_2, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
	ledcSetup(MOTOR_CHANNEL_3, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);

	ledcAttachPin(MOTOR_L_PWM, MOTOR_CHANNEL_2);
	ledcAttachPin(MOTOR_R_PWM, MOTOR_CHANNEL_3);

	pinMode(MOTOR_L_IN1, OUTPUT);
	pinMode(MOTOR_L_IN2, OUTPUT);
	pinMode(MOTOR_R_IN1, OUTPUT);
	pinMode(MOTOR_R_IN2, OUTPUT);
}

// 配置电机速度
// 参数1-->通道
// 参数2-->功率
void MOTOR_THUNDER::set_speed(uint8_t motor_channel, uint32_t speed)
{
	if (speed > valueMax)
	{
		speed = valueMax;
	}
	uint32_t duty = (speed * MOTOR_MAX_DUTY) / valueMax;

	ledcWrite(motor_channel, duty); //0-MOTOR_MAX_DUTY
}

// 开环电机控制函数
// 参数1-->电机编号；1或者2
// 参数2-->速度；范围为0-255
// 参数3-->方向, 0为反向，1为正向
void MOTOR_THUNDER::Motor_Move(int motor, int speed, int direction)
{
	if (direction == 1)
	{
		inPin1 = HIGH;
		inPin2 = LOW;
	}
	else
	{
		inPin1 = LOW;
		inPin2 = HIGH;
	}

	if (motor == L)
	{
		if (speed > LIMIT_SPEED)
		{
			digitalWrite(MOTOR_L_IN1, inPin1);
			digitalWrite(MOTOR_L_IN2, inPin2);
			set_speed(MOTOR_CHANNEL_2, speed);
		}
		else
		{
			digitalWrite(MOTOR_L_IN1, LOW);
			digitalWrite(MOTOR_L_IN2, LOW);
			ledcWrite(MOTOR_CHANNEL_2, 0);
		}
	}
	else if (motor == R)
	{
		if (speed > LIMIT_SPEED)
		{
			digitalWrite(MOTOR_R_IN1, inPin2);
			digitalWrite(MOTOR_R_IN2, inPin1);
			set_speed(MOTOR_CHANNEL_3, speed);
		}
		else
		{
			digitalWrite(MOTOR_R_IN1, LOW);
			digitalWrite(MOTOR_R_IN2, LOW);
			ledcWrite(MOTOR_CHANNEL_3, 0);
		}
	}
}

void MOTOR_THUNDER::Motor_Brake(int motor)
{
	Bell_Thunder.Disable_En_Motor();
	if (motor == L)
	{
		digitalWrite(MOTOR_L_IN1, HIGH);
		digitalWrite(MOTOR_L_IN2, HIGH);
		ledcWrite(MOTOR_CHANNEL_2, 0);
	}
	else if (motor == R)
	{
		digitalWrite(MOTOR_R_IN1, HIGH);
		digitalWrite(MOTOR_R_IN2, HIGH);
		ledcWrite(MOTOR_CHANNEL_3, 0);
	}
}
// 开环电机滑行函数
// 参数1-->电机编号；1或者2
void MOTOR_THUNDER::Motor_Free(int motor)
{
	Bell_Thunder.Disable_En_Motor();
	if (motor == L)
	{
		digitalWrite(MOTOR_L_IN1, LOW);
		digitalWrite(MOTOR_L_IN2, LOW);
		ledcWrite(MOTOR_CHANNEL_2, 0);
	}
	else if (motor == R)
	{
		digitalWrite(MOTOR_R_IN1, LOW);
		digitalWrite(MOTOR_R_IN2, LOW);
		ledcWrite(MOTOR_CHANNEL_3, 0);
	}
}

#elif (MOTOR_DRIVER_IC == MOTOR_DRIVER_IC_PT5126)

// 配置电机
void MOTOR_THUNDER::Setup_Motor()
{
	pinMode(PWM_L_A, INPUT_PULLUP);
	// if( digitalRead(PWM_L_A) == HIGH ){
	//   Serial.println("Motor IC is PT5126");
	// }else{
	//   Serial.println("Motor IC is not PT5126");
	// }

	ledcSetup(MOTOR_CHANNEL_2, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
	ledcSetup(MOTOR_CHANNEL_3, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
	ledcSetup(MOTOR_CHANNEL_4, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);
	ledcSetup(MOTOR_CHANNEL_5, MOTOR_BASE_FREQ, MOTOR_TIMER_13_BIT);

	ledcAttachPin(PWM_L_A, MOTOR_CHANNEL_2);
	ledcAttachPin(PWM_L_B, MOTOR_CHANNEL_3);
	ledcAttachPin(PWM_R_A, MOTOR_CHANNEL_4);
	ledcAttachPin(PWM_R_B, MOTOR_CHANNEL_5);

	pinMode(PWM_L_A, OUTPUT);
	pinMode(PWM_L_B, OUTPUT);
	pinMode(PWM_R_A, OUTPUT);
	pinMode(PWM_R_B, OUTPUT);

	motor_drive_mux = xSemaphoreCreateMutex();
}

// 配置电机速度
// 参数1-->通道
// 参数2-->功率
void MOTOR_THUNDER::set_speed(uint8_t motor_channel, uint32_t speed)
{
	uint8_t channel = 0;
	uint16_t speed_pulse = 0;
	if (speed > MOTOR_INPUT_MAX)
	{
		speed = MOTOR_INPUT_MAX;
	}
	switch (motor_channel)
	{
	case PWM_L_A:
		channel = MOTOR_CHANNEL_2;
		break;
	case PWM_L_B:
		channel = MOTOR_CHANNEL_3;
		break;
	case PWM_R_A:
		channel = MOTOR_CHANNEL_4;
		break;
	case PWM_R_B:
		channel = MOTOR_CHANNEL_5;
		break;
	default:
		break;
	}
	speed_pulse = ((speed * MOTOR_MAX_DUTY) / MOTOR_INPUT_MAX);
	ledcWrite(channel, speed_pulse);
}

// 开环电机控制函数
// 参数1-->电机编号；1或者2
// 参数2-->速度；范围为0-255
// 参数3-->方向, 0为反向，1为正向
void MOTOR_THUNDER::Motor_Move(int motor, int speed, int direction)
{
	if (speed > 255)
	{
		speed = 255;
	}
	else if (speed < 0)
	{
		speed = 0;
	}

	if (motor == L)
	{
		if (speed > LIMIT_SPEED)
		{
			if (direction == 1)
			{
				set_speed(PWM_L_A, MOTOR_INPUT_MAX);
				set_speed(PWM_L_B, MOTOR_INPUT_MAX - speed);
			}
			else if (direction == 2)
			{
				set_speed(PWM_L_A, MOTOR_INPUT_MAX - speed);
				set_speed(PWM_L_B, MOTOR_INPUT_MAX);
			}
		}
		else
		{
			set_speed(PWM_L_A, 0);
			set_speed(PWM_L_B, 0);
		}
	}
	else if (motor == R)
	{
		if (speed > LIMIT_SPEED)
		{
			if (direction == 1)
			{
				set_speed(PWM_R_A, MOTOR_INPUT_MAX - speed);
				set_speed(PWM_R_B, MOTOR_INPUT_MAX);
			}
			else if (direction == 2)
			{
				set_speed(PWM_R_A, MOTOR_INPUT_MAX);
				set_speed(PWM_R_B, MOTOR_INPUT_MAX - speed);
			}
		}
		else
		{
			set_speed(PWM_R_A, 0);
			set_speed(PWM_R_B, 0);
		}
	}
}

/**
 * @brief: 电机闭环刹车，如果电机高速行驶中突然闭环刹车，会引起电机抖动
 * 
 * @param motor:
 */
void MOTOR_THUNDER::Motor_Brake(int motor)
{
#if 0
  Bell_Thunder.Disable_En_Motor();

  if(motor == L)
  {
    set_speed(PWM_L_A,MOTOR_INPUT_MAX);
    set_speed(PWM_L_B,MOTOR_INPUT_MAX);
  }
  else if(motor == R)
  {
    set_speed(PWM_R_A,MOTOR_INPUT_MAX);
    set_speed(PWM_R_B,MOTOR_INPUT_MAX);
  }
#else
	int rotate_value;
	if (motor == L)
	{
		rotate_value = Get_RotateValue(1);
		Set_Motor_Position(1, rotate_value);
	}
	else if (motor == R)
	{
		rotate_value = Get_RotateValue(2);
		Set_Motor_Position(2, rotate_value);
	}

#endif
}

/**
 * @brief: 电机惯性停止
 * 
 * @param motor:
 */
void MOTOR_THUNDER::Motor_Free(int motor)
{
	Bell_Thunder.Disable_En_Motor();

	if (motor == L)
	{
		set_speed(PWM_L_A, 0);
		set_speed(PWM_L_B, 0);
	}
	else if (motor == R)
	{
		set_speed(PWM_R_A, 0);
		set_speed(PWM_R_B, 0);
	}
}
#endif

/**
 * @brief: 电机控制；范围为-255 ~ 255（负数为反向转，正为正向转；没有做速度控制）
 * 
 * @param motor: 1 是左电机，2 是右电机
 * @param M_output: 范围为-255 ~ 255，电池电压变化时，电机实际输出功率会变化
 */
void MOTOR_THUNDER::Set_Motor_Output(int motor, int M_output)
{
	if (M_output >= 0)
	{
		Motor_Move(motor, M_output, 1);
	}
	else
	{
		Motor_Move(motor, -1 * M_output, 2);
	}
}

/**
 * @brief: 开环控制电机，电机功率随电压浮动的范围较小，
 * 在电池电压下降时，会提高电机控制PWM脉宽，保持电机实际输出功率相对稳定
 * 
 * @param motor: 1 是左电机，2 是右电机
 * @param power: 范围为-100 ~ 100，电池电压变化时，会相对于Set_Motor_Output稳定的输出电机功率
 */
void MOTOR_THUNDER::Set_Motor_Power(int motor, int power)
{
	Bell_Thunder.Disable_En_Motor();
	float M_power;
	M_power = power;
	M_power = (float)MAX_DRIVE_OUTPUT * ((float)BATTERY_LOW_VALUE / Bell_Thunder.Get_Battery_Value()) * (M_power / 100);

	if (M_power >= 0)
	{
		Motor_Move(motor, M_power, 1);
	}
	else
	{
		Motor_Move(motor, -1 * M_power, 2);
	}
}

// 重置PID计算过程值
void MOTOR_THUNDER::PID_Reset(struct PID_Struct_t *pid)
{
	pid->Err = 0;
	pid->SumErr = 0;
	pid->LastErr = 0;

	pid->OutP = 0;
	pid->OutI = 0;
	pid->OutD = 0;

	pid->LastOutP = 0;
	pid->LastOutI = 0;
	pid->LastOutD = 0;
	// pid->Out = 0;
}
void MOTOR_THUNDER::PID_Reset()
{
	PID_Reset(&Motor_Speed_PID[0]);
	PID_Reset(&Motor_Speed_PID[1]);

	drive_car_pid.last_pid_time = 0;
	drive_car_pid.OutP_left = 0;
	drive_car_pid.OutI_left = 0;
	drive_car_pid.OutI_left_last = 0;
	drive_car_pid.OutD_left = 0;
	drive_car_pid.OutP_right = 0;
	drive_car_pid.OutI_right = 0;
	drive_car_pid.OutI_right_last = 0;
	drive_car_pid.OutD_right = 0;
	drive_car_pid.Out_left = 0;
	drive_car_pid.Out_right = 0;
}

// PID参数初始化
void MOTOR_THUNDER::PID_Init(struct PID_Struct_t *pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->Err = 0;
	pid->SumErr = 0;
	pid->LastErr = 0;

	pid->OutP = 0;
	pid->OutI = 0;
	pid->OutD = 0;

	pid->LastOutP = 0;
	pid->LastOutI = 0;
	pid->LastOutD = 0;

	pid->Out = 0;
}

// 按默认PID参数初始化电机闭环控制的变量
void MOTOR_THUNDER::All_PID_Init()
{
	float Kp, Ki, Kd;
	Kp = PID_Default_Kp;
	Ki = PID_Default_Ki;
	Kd = PID_Default_Kd;

	drive_car_pid.last_pid_time = 0;
	drive_car_pid.OutP_left = 0;
	drive_car_pid.OutI_left = 0;
	drive_car_pid.OutI_left_last = 0;
	drive_car_pid.OutD_left = 0;
	drive_car_pid.Out_left = 0;

	drive_car_pid.OutP_right = 0;
	drive_car_pid.OutI_right = 0;
	drive_car_pid.OutI_right_last = 0;
	drive_car_pid.OutD_right = 0;
	drive_car_pid.Out_right = 0;

	PID_Init(&Motor_Speed_PID[0], Kp, Ki, Kd);
	PID_Init(&Motor_Speed_PID[1], Kp, Ki, Kd);
}

float MOTOR_THUNDER::motor_PID(struct PID_Struct_t *pid) //PID计算
{
	//P计算
	pid->Err = pid->Ref - pid->Fdb; //计算误差
	pid->OutP = pid->Kp * pid->Err;

	//I计算
	pid->SumErr += pid->Err; //计算积累误差
	pid->OutI = pid->Ki * pid->SumErr;
	// if ( pid->Err == 0 )pid->SumErr /= 2;

	// if(pid->OutI > pid->IMax)         pid->OutI = pid->IMax;    //限制最大I值
	// else if(pid->OutI < (-1.0 * pid->IMax))   pid->OutI = -1.0 * pid->IMax;

	//D计算
	pid->OutD = pid->Kd * (pid->Err - pid->LastErr);
	pid->LastErr = pid->Err;
	// if(pid->OutD > pid->DMax)         pid->OutD = pid->DMax;    //限制最大D值
	// else if(pid->OutD < (-1.0 * pid->DMax))   pid->OutD = -1.0 * pid->DMax;

	//计算结果值
	pid->Out += pid->OutP + pid->OutI + pid->OutD - pid->LastOutP - pid->LastOutI - pid->LastOutD; //已经加过的部分减去
	if (pid->Out > pid->OutMax)
		pid->Out = pid->OutMax; //限制结果值范围
	else if (pid->Out < pid->OutMin)
		pid->Out = pid->OutMin;

	// if ((pid->Out < pid->OutLeast) & (pid->Out > -pid->OutLeast) & (pid->Ref == 0))
	// 	pid->Out = 0; //限制最小输出值

	pid->LastOutP = pid->OutP;
	pid->LastOutI = pid->OutI;
	pid->LastOutD = pid->OutD;

	return pid->Out;
}

// 配置左右两个电机编码器
void MOTOR_THUNDER::Setup_Motor_PID()
{
	// Serial.printf("SSSSSSSSSS Setup_Motor_PID SSSSSSSSSS\n");

	pinMode(EN_L_A, INPUT);
	pinMode(EN_L_B, INPUT);
	pinMode(EN_R_A, INPUT);
	pinMode(EN_R_B, INPUT);

	Setup_PID_Timer(); //PID时间中断
	All_PID_Init();	//PID参数初始化

#ifdef ENABLE_ENCODER_INT
	attachInterrupt(EN_L_A, ISR_Encoder_L, RISING);
	attachInterrupt(EN_R_A, ISR_Encoder_R, RISING);

	attachInterrupt(EN_L_B, ISR_Encoder_L2, RISING);
	attachInterrupt(EN_R_B, ISR_Encoder_R2, RISING);
#else
	// set pulse cnt Operate
	pinMatrixInAttach(EN_L_A, PCNT_SIG_CH0_IN0_IDX, false);
	pinMatrixInAttach(EN_L_B, PCNT_CTRL_CH0_IN0_IDX, false);
	pinMatrixInAttach(EN_R_A, PCNT_SIG_CH0_IN1_IDX, false);
	pinMatrixInAttach(EN_R_B, PCNT_CTRL_CH0_IN1_IDX, false);

	/* Prepare configuration for the PCNT unit */
	pcnt_config_t pcnt_config;
	{
		// Set PCNT input signal and control GPIOs
		pcnt_config.pulse_gpio_num = EN_L_A;
		pcnt_config.ctrl_gpio_num = EN_L_B;
		pcnt_config.channel = PCNT_CHANNEL_0;
		pcnt_config.unit = PCNT_UNIT_0;
		// What to do on the positive / negative edge of pulse input?
		pcnt_config.pos_mode = PCNT_COUNT_INC; // Count up on the positive edge
		pcnt_config.neg_mode = PCNT_COUNT_DEC; // Keep the counter value on the negative edge
		// What to do when control input is low or high?
		pcnt_config.lctrl_mode = PCNT_MODE_KEEP;	// Reverse counting direction if low
		pcnt_config.hctrl_mode = PCNT_MODE_REVERSE; // Keep the primary counter mode if high
		// Set the maximum and minimum limit values to watch
		pcnt_config.counter_h_lim = 1000;
		pcnt_config.counter_l_lim = -1000;
	}
	/* Initialize PCNT unit */
	pcnt_unit_config(&pcnt_config);
	pcnt_config.pulse_gpio_num = EN_R_A;
	pcnt_config.ctrl_gpio_num = EN_R_B;
	pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
	pcnt_config.hctrl_mode = PCNT_MODE_KEEP;	// Keep the primary counter mode if high
	pcnt_config.unit = PCNT_UNIT_1;
	pcnt_unit_config(&pcnt_config);

	/* Configure and enable the input filter */
	pcnt_set_filter_value(PCNT_UNIT_0, 1000);
	pcnt_filter_enable(PCNT_UNIT_0);
	pcnt_set_filter_value(PCNT_UNIT_1, 1000);
	pcnt_filter_enable(PCNT_UNIT_1);

	/* Initialize PCNT's counter */
	pcnt_counter_pause(PCNT_UNIT_0);
	pcnt_counter_clear(PCNT_UNIT_0);
	pcnt_counter_pause(PCNT_UNIT_1);
	pcnt_counter_clear(PCNT_UNIT_1);

	/* Everything is set up, now go to counting */
	pcnt_counter_resume(PCNT_UNIT_0);
	pcnt_counter_resume(PCNT_UNIT_1);
#endif
}

// 按PID输出控制左右两个电机
void MOTOR_THUNDER::PID_Speed()
{
	// 在定时器里面获取了 计数器的数值，存在Encoder_Counter[0] Encoder_Counter[1]
	// Get_Encoder_Value();

	Motor_Speed_PID[0].Fdb = Encoder_Counter[0];
	Motor_Speed_PID[1].Fdb = Encoder_Counter[1];

	//SSSSSSSSSS ___ 左轮 ___ SSSSSSSSSS
	motor_PID(&Motor_Speed_PID[0]);

	if (Motor_Speed_PID[0].Out > 0) //正转
	{
		Motor_Move(1, Motor_Speed_PID[0].Out, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	}
	else
	{
		Motor_Move(1, -Motor_Speed_PID[0].Out, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	}

	//SSSSSSSSSS ___ 右轮 ___ SSSSSSSSSS
	motor_PID(&Motor_Speed_PID[1]);
	
	if (Motor_Speed_PID[1].Out > 0) //正转
	{
		Motor_Move(2, Motor_Speed_PID[1].Out, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	}
	else
	{
		Motor_Move(2, -Motor_Speed_PID[1].Out, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
	}
}

void MOTOR_THUNDER::Motor_Position_Control(MotorPosition_Struct *position_ctrl)
{
	// 采用增量式PID
	float pid_result;

	position_ctrl->delta = position_ctrl->target - position_ctrl->variant;
	pid_result = POSITION_Kp * (position_ctrl->delta - position_ctrl->pre_delta);
	pid_result += POSITION_Ki * position_ctrl->delta;
	pid_result += POSITION_Kd * (position_ctrl->delta - 2 * position_ctrl->pre_delta + position_ctrl->pre2_delta);

	position_ctrl->PID_output += pid_result;
	position_ctrl->Pre_output = pid_result;

	position_ctrl->pre2_delta = position_ctrl->pre_delta;
	position_ctrl->pre_delta = position_ctrl->delta;
}

void MOTOR_THUNDER::Set_Motor_Position(int motor, int position_target)
{
	int motor_index;
	CHECK_MOTOR_INDEX(motor);
	motor_index = motor - 1;
	
	Position_Ctrl[motor_index].target = position_target;

	Bell_Thunder.Enable_Motor_Position();
	Position_Ctrl[motor_index].enable_ctrl = true;
}
void MOTOR_THUNDER::Clear_Position_Control()
{
	Position_Ctrl[0].enable_ctrl = false;
	Position_Ctrl[1].enable_ctrl = false;
}

void MOTOR_THUNDER::Position_Control()
{
	if (Position_Ctrl[0].enable_ctrl == true)
	{
		Position_Ctrl[0].variant = Get_RotateValue(1);
		Motor_Position_Control(&Position_Ctrl[0]);
		Set_Motor_Output(1, Position_Ctrl[0].PID_output);
	}
	if (Position_Ctrl[1].enable_ctrl == true)
	{
		Position_Ctrl[1].variant = Get_RotateValue(2);
		Motor_Position_Control(&Position_Ctrl[1]);
		Set_Motor_Output(2, Position_Ctrl[1].PID_output);
	}
}

/* 
 * 控制电机运行，可以设置控制模式：0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
 * 
 * @parameters:
 * 
 * struct MotorRunning_Struct{
  //0代表left 和 right的控制都起效；1代表left控制起效，right无效; 2代表right控制起效，left无效
  byte motor_select;

  //0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
  byte running_mode;
  // 控制模式的参数
  float mode_data;

  //左电机的转动速度
  float left_motor_speed;
  //右电机的转动速度
  float right_motor_speed;
  }; 
 *
 * @return: 
 */
void MOTOR_THUNDER::Control_Motor_Running(MotorRunning_Struct &running_data)
{
	Bell_Thunder.Disable_En_Motor();

	if (running_data.left_motor_speed > 100.0)
	{
		running_data.left_motor_speed = 100.0;
	}
	else if (running_data.left_motor_speed < -100.0)
	{
		running_data.left_motor_speed = -100.0;
	}
	if (running_data.right_motor_speed > 100.0)
	{
		running_data.right_motor_speed = 100.0;
	}
	else if (running_data.right_motor_speed < -100.0)
	{
		running_data.right_motor_speed = -100.0;
	}

	switch (running_data.motor_select)
	{
	case 0:
	{
		Set_Motor_Output(L, running_data.left_motor_speed * MAX_DRIVE_OUTPUT / 100);
		Set_Motor_Output(R, running_data.right_motor_speed * MAX_DRIVE_OUTPUT / 100);
		break;
	}
	case 1:
	{
		Set_Motor_Output(L, running_data.left_motor_speed * MAX_DRIVE_OUTPUT / 100);
		break;
	}
	case 2:
	{
		Set_Motor_Output(R, running_data.right_motor_speed * MAX_DRIVE_OUTPUT / 100);
		break;
	}
	default:
		return;
		break;
	}

	// 运行模式，0: 无模式；1：控制时间（秒）；2：控制角度（度）；3:控制圈数（圈）
	switch (running_data.running_mode)
	{
	case 0:
	{
		// 不用停止电机，立刻返回，
		return;
		break;
	}
	case 1:
	{
		delay(1000 * running_data.mode_data);
		break;
	}
	case 2:
	case 3:
	{
		int32_t last_left_RotateValue;
		int32_t last_right_RotateValue;
		int32_t circle2degrees;

		if (running_data.running_mode == 2)
		{
			circle2degrees = running_data.mode_data;
		}
		else
		{
			circle2degrees = running_data.mode_data * 360;
		}

		last_left_RotateValue = Get_RotateValue(1);
		last_right_RotateValue = Get_RotateValue(2);

		if (running_data.motor_select == 1)
		{
			while (abs(Get_RotateValue(1) - last_left_RotateValue) < circle2degrees && running_data.left_motor_speed != 0)
			{
			}
		}
		else if (running_data.motor_select == 2)
		{
			while (abs(Get_RotateValue(2) - last_right_RotateValue) < circle2degrees && running_data.right_motor_speed != 0)
			{
			}
		}
		else
		{
			while ((abs(Get_RotateValue(1) - last_left_RotateValue) < circle2degrees && running_data.left_motor_speed != 0) ||
				   (abs(Get_RotateValue(2) - last_right_RotateValue) < circle2degrees && running_data.right_motor_speed != 0))
			{
				if (abs(Get_RotateValue(1) - last_left_RotateValue) >= circle2degrees)
				{
					Set_Motor_Output(1, 0);
					break; // 任意一个转到位置，即刻去停止电机
				}
				if (abs(Get_RotateValue(2) - last_right_RotateValue) >= circle2degrees)
				{
					Set_Motor_Output(2, 0);
					break; // 任意一个转到位置，即刻去停止电机
				}
			}
		}
	}
	break;

	default:
		break;
	}

	//运行完后，停止被控制的电机
	switch (running_data.motor_select)
	{
	case 0:
	{
		Set_Motor_Output(L, 0);
		Set_Motor_Output(R, 0);
		break;
	}
	case 1:
	{
		Set_Motor_Output(L, 0);
		break;
	}
	case 2:
	{
		Set_Motor_Output(R, 0);
		break;
	}
	default:
		return;
		break;
	}
}
/**
 * @brief: 
 * 
 * @param _select: 0代表left 和 right的控制都起效；1代表left控制起效，right无效; 2代表right控制起效，left无效
 * @param _mode: 0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
 * @param _data: 控制模式的参数，例如控制时间模式，_data=0.1时，代表控制电机转0.1秒
 * @param _left_speed: 左电机的转动速度
 * @param _right_speed: 右电机的转动速度
 */
void MOTOR_THUNDER::Control_Motor_Running(byte _select, byte _mode, float _data, float _left_speed, float _right_speed)
{
	MotorRunning_Struct _running_data;

	_running_data.motor_select = _select;
	_running_data.running_mode = _mode;
	_running_data.mode_data = _data;
	_running_data.left_motor_speed = _left_speed;
	_running_data.right_motor_speed = _right_speed;

	Control_Motor_Running(_running_data);
}
/* 
 * 控制电机转向运行
 * @parameters:
 * 
struct MotorTurnning_Struct{
  //0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
  byte turnning_mode;

  float mode_data;

  // 方向转向程度，取值范围 -100~100
  float turn_percent;
  float motor_speed;
};
 *
 * @return: 
 */
void MOTOR_THUNDER::Control_Motor_Turnning(MotorTurnning_Struct &turnning_data)
{
	Set_Car_Speed_Direction(turnning_data.motor_speed, turnning_data.turn_percent);

	// 运行模式，0: 无模式；1：控制时间（秒）；2：控制角度（度）；3:控制圈数（圈）
	switch (turnning_data.turnning_mode)
	{
	case 0:
	{
		// 不用停止电机，立刻返回，
		return;
		break;
	}
	case 1:
	{
		delay(1000 * turnning_data.mode_data);
		break;
	}
	case 2:
	{
		int32_t last_RotateValue;
		if (turnning_data.motor_speed == 0)
			break;

		if (turnning_data.turn_percent >= 0)
		{
			last_RotateValue = Get_RotateValue(1);
			while (abs(Get_RotateValue(1) - last_RotateValue) < turnning_data.mode_data)
			{
			}
		}
		else
		{
			last_RotateValue = Get_RotateValue(2);
			while (abs(Get_RotateValue(2) - last_RotateValue) < turnning_data.mode_data)
			{
			}
		}

		break;
	}
	case 3:
	{
		int32_t last_RotateValue;
		int32_t circle2degrees;
		if (turnning_data.motor_speed == 0)
			break;

		circle2degrees = turnning_data.mode_data * 360;

		if (turnning_data.turn_percent >= 0)
		{
			last_RotateValue = Get_RotateValue(1);
			while (abs(Get_RotateValue(1) - last_RotateValue) < circle2degrees)
			{
			}
		}
		else
		{
			last_RotateValue = Get_RotateValue(2);
			while (abs(Get_RotateValue(2) - last_RotateValue) < circle2degrees)
			{
			}
		}
		break;
	}
	default:
		break;
	}

	Set_Car_Speed_Direction(0, 0);
	// Motor_Free(1);
	// Motor_Free(2);
}

/**
 * @brief: 控制L、R电机以一定的差速实现小车转向行驶
 * 
 * @param _mode: 0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
 * @param _data: 控制模式的参数，例如控制时间模式，_data=0.1时，代表控制电机转0.1秒
 * @param _percent: 方向转向程度，取值范围 -100~100
 * @param _speed: L、R电机中最大的转速
 */
void MOTOR_THUNDER::Control_Motor_Turnning(byte _mode, float _data, float _percent, float _speed)
{
	MotorTurnning_Struct _turnning_data;

	_turnning_data.turnning_mode = _mode;
	_turnning_data.mode_data = _data;
	_turnning_data.turn_percent = _percent;
	_turnning_data.motor_speed = _speed;

	Control_Motor_Turnning(_turnning_data);
}

void MOTOR_THUNDER::Calculate_Left_Control()
{
	// calculate left motor out power
	drive_car_pid.OutP_left = drive_car_pid.left_speed_diff * drive_car_pid.Kp;
	drive_car_pid.OutI_left += drive_car_pid.left_speed_diff * drive_car_pid.Ki;
	drive_car_pid.Out_left = drive_car_pid.OutP_left + drive_car_pid.OutI_left;

	if (drive_car_pid.Out_left > MAX_DRIVE_OUTPUT)
	{
		drive_car_pid.Out_left = MAX_DRIVE_OUTPUT;
		drive_car_pid.OutI_left = drive_car_pid.OutI_left_last;
	}
	else if (drive_car_pid.Out_left < -MAX_DRIVE_OUTPUT)
	{
		drive_car_pid.Out_left = -MAX_DRIVE_OUTPUT;
		drive_car_pid.OutI_left = drive_car_pid.OutI_left_last;
	}
	else
	{
		drive_car_pid.OutI_left_last = drive_car_pid.OutI_left;
	}
}
void MOTOR_THUNDER::Calculate_Right_Control()
{
	// calculate right motor out power
	drive_car_pid.OutP_right = drive_car_pid.right_speed_diff * drive_car_pid.Kp;
	drive_car_pid.OutI_right += drive_car_pid.right_speed_diff * drive_car_pid.Ki;
	drive_car_pid.Out_right = drive_car_pid.OutP_right + drive_car_pid.OutI_right;

	if (drive_car_pid.Out_right > MAX_DRIVE_OUTPUT)
	{
		drive_car_pid.Out_right = MAX_DRIVE_OUTPUT;
		drive_car_pid.OutI_right = drive_car_pid.OutI_right_last;
	}
	else if (drive_car_pid.Out_right < -MAX_DRIVE_OUTPUT)
	{
		drive_car_pid.Out_right = -MAX_DRIVE_OUTPUT;
		drive_car_pid.OutI_right = drive_car_pid.OutI_right_last;
	}
	else
	{
		drive_car_pid.OutI_right_last = drive_car_pid.OutI_right;
	}
}

/* 
 * 控制速度和方向的PID函数
 * 
 * @parameters: 
 * @return: 
 */
void MOTOR_THUNDER::Drive_Car_Control()
{
	float time_interval;
	float left_speed, right_speed;

	if (drive_car_pid.last_pid_time == 0)
	{
		time_interval = MOTOR_CONTROL_PERIOD;
		drive_car_pid.last_pid_time = millis();
	}
	else
	{
		time_interval = millis() - drive_car_pid.last_pid_time;
		drive_car_pid.last_pid_time = millis();
	}
	left_speed = Encoder_Counter[0];
	left_speed = left_speed * (float)MOTOR_CONTROL_PERIOD / time_interval;
	right_speed = Encoder_Counter[1];
	right_speed = right_speed * (float)MOTOR_CONTROL_PERIOD / time_interval;

	// Serial.printf("LeftSpeed:%f, RightSpeed:%f \n", left_speed, right_speed);

	drive_car_pid.left_speed_diff = drive_car_pid.left_speed_target - left_speed;
	drive_car_pid.right_speed_diff = drive_car_pid.right_speed_target - right_speed;
	if (drive_direction == 0.0)
	{
		// drive_car_pid.new_left_target = drive_car_pid.left_speed_target;
		// drive_car_pid.new_right_target = drive_car_pid.right_speed_target;

		if ((drive_car_pid.Out_left == 255 && drive_car_pid.left_speed_diff > 0) ||
			(drive_car_pid.Out_left == -255 && drive_car_pid.left_speed_diff < 0))
		{
			drive_car_pid.new_right_target = left_speed;
			drive_car_pid.right_speed_diff = drive_car_pid.new_right_target - right_speed;
			// Serial.printf("Right target:%f \n", drive_car_pid.new_right_target);
		}

		if ((drive_car_pid.Out_right == 255 && drive_car_pid.right_speed_diff > 0) ||
			(drive_car_pid.Out_right == -255 && drive_car_pid.right_speed_diff < 0))
		{
			drive_car_pid.new_left_target = right_speed;
			drive_car_pid.left_speed_diff = drive_car_pid.new_left_target - left_speed;
			// Serial.printf("Left target:%f \n", drive_car_pid.new_left_target);
		}
		// Serial.printf("Left target:%f, Right target:%f \n", drive_car_pid.new_left_target, drive_car_pid.new_right_target);
	}
	else if ((drive_direction > 0.0 && drive_car_pid.Out_left == 255 && drive_car_pid.left_speed_diff > 0) ||
			 (drive_direction > 0.0 && drive_car_pid.Out_left == -255 && drive_car_pid.left_speed_diff < 0))
	{
		drive_car_pid.new_right_target = left_speed * (MAX_DRIVE_DIRECTION / 2 - drive_direction) / (MAX_DRIVE_DIRECTION / 2);
		drive_car_pid.right_speed_diff = drive_car_pid.new_right_target - right_speed;

		// Serial.printf("Left target:%f, Right target:%f \n", (float)left_speed, new_right_target);
	}
	else if ((drive_direction < 0.0 && drive_car_pid.Out_right == 255 && drive_car_pid.right_speed_diff > 0) ||
			 (drive_direction < 0.0 && drive_car_pid.Out_right == -255 && drive_car_pid.right_speed_diff < 0))
	{
		drive_car_pid.new_left_target = right_speed * (MAX_DRIVE_DIRECTION / 2 + drive_direction) / (MAX_DRIVE_DIRECTION / 2);
		drive_car_pid.left_speed_diff = drive_car_pid.new_left_target - left_speed;

		// Serial.printf("Left target:%f, Right target:%f \n", new_left_target, (float)right_speed);
	}

	// Serial.printf("LeftDiff:%f, RightDiff:%f \n", drive_car_pid.left_speed_diff, drive_car_pid.right_speed_diff);
	Calculate_Left_Control();
	Calculate_Right_Control();

	// Serial.printf("LeftOut:%f, RightOut:%f \n\n", drive_car_pid.Out_left, drive_car_pid.Out_right);

	Set_Motor_Output(L, (int)drive_car_pid.Out_left);
	Set_Motor_Output(R, (int)drive_car_pid.Out_right);
}

/* 
 * 控制驾驶的速度和方向，
 * 速度:
 * 设置全速的百分比，值为 -100~100，-100为全速后退，100为全速前进
 * 正速度时，0方向为正前方；
 * 负速度时，0方向为正后方
 * 方向：
 * 0~360为右转，向前时0为正前方，360为顺时针打转；向后时0为正后方，360为逆时针打转；
 * 0~-360为左转，向前时0为正前方，-360为逆时针打转；向后时0为正后方，-360为顺时针打转；
 * 右转时：direction = 180 * (L-R)/L 左转时：direction = 180 * (L-R)/R
 * 
 * @parameters: 
 * @return: 
 */
void MOTOR_THUNDER::Set_Car_Speed_Direction(float speed, float direction)
{
	// All_PID_Init();

	if (speed > 100.0)
	{
		drive_speed = 100.0;
	}
	else if (speed < -100.0)
	{
		drive_speed = -100.0;
	}
	else
	{
		drive_speed = speed;
	}
	drive_speed = (drive_speed / 100.0) * MAX_DRIVE_SPEED;

	if (direction > MAX_DRIVE_DIRECTION)
	{
		drive_direction = MAX_DRIVE_DIRECTION;
	}
	else if (direction < -MAX_DRIVE_DIRECTION)
	{
		drive_direction = -MAX_DRIVE_DIRECTION;
	}
	else
	{
		drive_direction = direction;
	}

	if (drive_direction >= 0.0)
	{
		drive_car_pid.P_direction_divisor = &Encoder_Counter[0];
		drive_car_pid.left_speed_target = drive_speed;
		drive_car_pid.right_speed_target = drive_speed * (MAX_DRIVE_DIRECTION / 2 - drive_direction) / (MAX_DRIVE_DIRECTION / 2);
	}
	else
	{
		drive_car_pid.P_direction_divisor = &Encoder_Counter[1];
		drive_car_pid.right_speed_target = drive_speed;
		drive_car_pid.left_speed_target = drive_speed * (MAX_DRIVE_DIRECTION / 2 + drive_direction) / (MAX_DRIVE_DIRECTION / 2);
	}

	drive_car_pid.new_left_target = drive_car_pid.left_speed_target;
	drive_car_pid.new_right_target = drive_car_pid.right_speed_target;

	// Serial.printf("Left target:%f, Right target:%f \n", drive_car_pid.left_speed_target, drive_car_pid.right_speed_target);

	Bell_Thunder.Enable_Drive_Car();
}

/**
 * @brief: PID闭环控制电机转速，PID控制周期为 MOTOR_CONTROL_PERIOD
 * PID控制周期采集的编码器数值最大为 MAX_DRIVE_SPEED
 * 最大PID控制速度 = MAX_DRIVE_SPEED * 1000 / MOTOR_CONTROL_PERIOD * 60 / ENCODER_NUM_EVERY_CIRCLE
 *                = 277.78(转/分)
 *                = 4.63(转/秒)
 * 
 * @param motor: 电机编号
 * @param target: 最大PID控制速度的百分比
 */
void MOTOR_THUNDER::Set_Target(int motor, float target)
{
	float target_encoder_num;
	int motor_index;
	CHECK_MOTOR_INDEX(motor);
	motor_index = motor - 1;

	target_encoder_num = target / 100 * MAX_DRIVE_SPEED;
	if (Motor_Speed_PID[motor_index].Ref != target_encoder_num)
	{
		// PID_Reset(&Motor_Speed_PID[0]);
		Motor_Speed_PID[motor_index].Ref = target_encoder_num;
	}

	Bell_Thunder.Enable_En_Motor();
}

int16_t MOTOR_THUNDER::Get_Speed_simple(int motor)
{
	int motor_index;
	CHECK_MOTOR_INDEX(motor);
	motor_index = motor - 1;

	return Encoder_Counter[motor_index] * 100 / MAX_DRIVE_SPEED;
}

/**
 * @brief: 获取电机速度, 这个数值是电机最大PID控制速度 的 百分比
 * 也可以用于开环控制的场景，获取电机转速
 * 
 * @param motor: 电机接口编号
 * @param times: 默认为0，可以设置计算周期, 周期越长，速度数值实时性差但是数值比较稳定
 * @return int16_t : 电机速度
 */
int16_t MOTOR_THUNDER::Get_Speed(int motor, uint8_t times)
{
	int motor_index;
	CHECK_MOTOR_INDEX(motor);
	motor_index = motor - 1;

	if(times != encode_amount[motor_index]._times){
		encode_amount[motor_index].times = 0;
		encode_amount[motor_index].amount = 0;
		encode_amount[motor_index]._times = times;
	}

	return encode_amount[motor_index]._amount * 100 / ((encode_amount[motor_index]._times+1)*MAX_DRIVE_SPEED);
}

// 获取目标(编码器计数值)
int16_t MOTOR_THUNDER::Get_Target(int motor)
{
	int motor_index;
	CHECK_MOTOR_INDEX(motor);
	motor_index = motor - 1;

	return Motor_Speed_PID[motor_index].Ref * 100 / MAX_DRIVE_SPEED;
}

/* 
 * 清零电机编码器，在初始化或者获取完计数器数值 等等情况之后可以调用此函数
 * 
 * 
 */
inline void Encoder_Counter_Clear()
{
	pcnt_counter_clear(PCNT_UNIT_0); // Left
	pcnt_counter_clear(PCNT_UNIT_1); // Right
}

/* 
 * 获取编码器计数器数值，保存到相应变量里面
 * 每次获取都会清零计数器
 */
inline void Get_Encoder_Value()
{

	pcnt_get_counter_value(PCNT_UNIT_0, &Encoder_Counter[0]);
	pcnt_get_counter_value(PCNT_UNIT_1, &Encoder_Counter[1]);

	Encoder_Counter_Clear();
}

/*
 * 在Encoder_Counter[0] Encoder_Counter[1]更新之后才能调用此更新过程
 *   不然会产生重复累积，导致数据错误
 * 
 */
inline void Update_Rotate_Value()
{
	for(int i=0; i<MOTOR_AMOUNT; i++)
	{
		rotate_RawValue[i] += Encoder_Counter[i];

		if(encode_amount[i].times < encode_amount[i]._times) {
			encode_amount[i].amount += Encoder_Counter[i];
			encode_amount[i].times++;
		} else {
			encode_amount[i].amount += Encoder_Counter[i];
			encode_amount[i]._amount = encode_amount[i].amount;
			encode_amount[i].times = 0;
			encode_amount[i].amount = 0;
		}
	}
}

/*
 * 获取电机旋转量
 * 电机编码器磁铁是四对磁极对，减速比是27
 * 编码器的计数器，计算了信号上下沿的数量，所以转一圈，编码器数值为216，实际测量也是
 * 轮子(没有轮胎)周长：16cm
 * 
 * @parameters: 
 * @return: 
 */
int32_t MOTOR_THUNDER::Get_RotateValue(int motor)
{
	int motor_index;
	int32_t rotate_value;
	CHECK_MOTOR_INDEX(motor);
	motor_index = motor - 1;

	rotate_value = rotate_RawValue[motor_index] - rotate_Record_Origin[motor_index];
	rotate_value = rotate_value * DEGREES_EVERY_CIRCLE / ENCODER_NUM_EVERY_CIRCLE;

	return rotate_value;
}

/**
 * @brief: 清零电机旋转量记录
 * 
 * @param motor: 电机编号 1 和 2
 */
void MOTOR_THUNDER::Clear_RotateValue(int motor)
{
	int motor_index;
	CHECK_MOTOR_INDEX(motor);
	motor_index = motor - 1;

	rotate_Record_Origin[motor_index] = rotate_RawValue[motor_index];
}
