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
    const float Servo_MIN = 220; // 205 = 500us
    const float Servo_Range = 819; 
    float servo_percent_max[2] = {100, 100};
    float servo_percent_min[2] = {-100, -100};
    float servo_percent_zero[2] = {0, 0};
    uint32_t servo_position[2] = {0, 0};
public:
    typedef enum{
        A = 1,
        B = 2
    } enum_Servo_Index;

    void Setup_Servo(void);
    void Servo_Turn(int servo, float angle);

    /*--------------Thunder IDE APIs: -------------*/
    void Servo_Percent_Setting(int servo_index, float max_value = 100, float min_value = -100, float zero_value=0, int direction=1);
    void Servo_Turn_Percent(int servo, float percent, float speed=100);
};

#endif
