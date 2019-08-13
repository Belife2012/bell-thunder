#ifndef __MOTOR_SERVO_H__
#define __MOTOR_SERVO_H__

// 舵机
#define SERVO_CHANNEL_0     0    // use 0 channel of 16 channels (started from zero)
#define SERVO_CHANNEL_1     1    // use 0 channel of 16 channels (started from zero)
#define SERVO_TIMER_13_BIT  13   // 定时器精度13位
#define SERVO_BASE_FREQ     50   // 频率50Hz
#define SERVO_A             5    // 舵机A口
#define SERVO_B             25    // 舵机B口

class MOTOR_SERVO
{
private:
    // 舵机 PWM频率50Hz，0度时 脉宽最小500us，180度时 脉宽最大2500us，开机舵机居中为 90度
    // 20000[us] * 计数值 / 8191 = 输出[us]
    float Servo_MIN = 205;
    float Servo_Range = 819; 
    float servo_percent_max[2] = {100, 100};
    float servo_percent_min[2] = {-100, -100};
    float servo_percent_zero[2] = {0, 0};
public:
    void Setup_Servo(void);
    void Servo_Turn(int servo, float angle);

    /*--------------Thunder IDE APIs: -------------*/
    void Servo_Percent_Setting(int servo_index, float max_value, float min_value, float zero_value, int direction);
    void Servo_Turn_Percent(int servo, float percent);
};

#endif
