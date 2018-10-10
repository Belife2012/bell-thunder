/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 电机库文件
 * 
 *   创建日期： 20180606
 *   作者：     宋博伟
 *   邮箱：     songbw123@163.com
 *
 *   版本：     v0.2
 *   修改日期   20180721
 *   修改：     宋博伟
 *   邮箱：     songbw123@163.com
 *   修改内容： 
 * 
 *   
 * 
 * 功能列表：
 *  // 定时器
 *  1.  void Setup_PID_Timer(void);         // 配置PID用定时器
 *  2.  void Uninstall_PID_Timer(void);     // 移除PID用定时器(会影响其它用到此定时器的功能)
 * 
 *  // 开环电机
 *  3.  void Setup_Motor(void);                                 // 配置电机
 *  4.  void Motor_Move(int motor, int speed, int direction);   // 开环电机控制函数
 * 
 *  // 闭环电机
 *  5.  void PID_Reset(struct PID_Struct_t *pid);                               // 重置PID计算过程值
 *  6.  void PID_Init(struct PID_Struct_t *pid, float Kp, float Ki, float Kd);  // PID参数初始化
 *  7.  void All_PID_Init();                                                    // 按默认PID参数初始化左右电机
 *  8.  void Setup_Motor_PID(void);                                             // 配置左右两个电机编码器
 *  9.  void PID_Speed(void);                                                   // 按PID输出控制左右两个电机
 *  10. void Set_L_Target(float target);                                        // 设定左轮目标速度(编码器计数值)
 *  11. void Set_R_Target(float target);                                        // 设定右轮目标速度(编码器计数值)
 *  12. int16_t Get_L_Speed(void);                                              // 获取左轮速度(编码器计数值)
 *  13. int16_t Get_R_Speed(void);                                              // 获取右轮速度(编码器计数值)
 *  14. int16_t Get_L_Target(void);                                             // 获取左轮目标(编码器计数值)
 *  15. int16_t Get_R_Target(void);                                             // 获取右轮目标(编码器计数值)
 * 
 * 
 * 
 ************************************************/

#ifndef _THUNDER_MOTOR_H_
#define _THUNDER_MOTOR_H_

#include <Arduino.h>
#include <common_mesg.h>

// 电机
// TB6612驱动驱动直流无刷电机
#define MOTOR_CHANNEL_2     2   //0--16
#define MOTOR_CHANNEL_3     3
#define MOTOR_TIMER_13_BIT  13 //分辨率13位
#define MOTOR_BASE_FREQ     1000 //频率
#define MOTOR_L_PWM         18
#define MOTOR_L_IN1         32
#define MOTOR_L_IN2         19
#define MOTOR_R_PWM         27
#define MOTOR_R_IN1         13
#define MOTOR_R_IN2         23

#define MOTOR_MAX_DUTY      5500 //最大8191 // 8191 = 2 ^ 13 - 1 //电机选型的原因在此进行最大输出限制   //安全duty 5500

// 编码器
#define EN_L_A 14 //编码器A 中断  左
#define EN_L_B 33 //编码器A
#define EN_R_A 34 //编码器B 中断  右
#define EN_R_B 17 //编码器B

#define PID_Default_Kp 8.0      //P 3.3   8.0   8.0
#define PID_Default_Ki 3.0      //I 2.85  4.0   3.0
#define PID_Default_Kd 3.5      //D       3.5   3.5
#define PID_dt 50               //[ms]

// PID时间中断
extern volatile SemaphoreHandle_t Timer_PID_Flag;
extern volatile uint32_t lastIsrAt;

struct PID_Struct_t 
{
    float Ref = 0;     //目标速度
    float Fdb = 0;
    float Err = 0;
    float SumErr = 0;
    float LastErr = 0;
    
    float OutP = 0;
    float OutI = 0;
    float OutD = 0;
    float LastOutP = 0;
    float LastOutI = 0;
    float LastOutD = 0;
    
    // float IMax = 200;
    // float DMax = 200;
    float OutMax = 255;
    float OutMin = -255;
    float OutLeast = 30;

//    float dt = 0;
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float Out = 0;
};

class THUNDER_MOTOR
{
  public:
    // 定时器
    void Setup_PID_Timer(void);         // 配置PID用定时器
    void Uninstall_PID_Timer(void);     // 移除PID用定时器(会影响其它用到此定时器的功能)

    // 开环电机
    void Setup_Motor(void);                                 // 配置电机
    void Motor_Move(int motor, int speed, int direction);   // 开环电机控制函数

    // 闭环电机
    void PID_Reset(struct PID_Struct_t *pid);                               // 重置PID计算过程值
    void PID_Init(struct PID_Struct_t *pid, float Kp, float Ki, float Kd);  // PID参数初始化
    void All_PID_Init();                                                    // 按默认PID参数初始化左右电机
    void Setup_Motor_PID(void);                                             // 配置左右两个电机编码器
    void PID_Speed(void);                                                   // 按PID输出控制左右两个电机
    
    // 获取编码器的计数器数值
    void Get_Encoder_Value(void);

    void Set_L_Target(float target);  // 设定左轮目标速度(编码器计数值)
    void Set_R_Target(float target);  // 设定右轮目标速度(编码器计数值)

    int16_t Get_L_Speed(void);  // 获取左轮速度(编码器计数值)
    int16_t Get_R_Speed(void);  // 获取右轮速度(编码器计数值)

    int16_t Get_L_Target(void);  // 获取左轮目标(编码器计数值)
    int16_t Get_R_Target(void);  // 获取右轮目标(编码器计数值)
   
  private:
    // 电机
    boolean inPin1 = LOW;
    boolean inPin2 = HIGH;
    uint8_t valueMax = 255;

    // 编码器计数器数值
    int16_t Encoder_Counter_Left;
    int16_t Encoder_Counter_Right;

    struct PID_Struct_t Motor_L_Speed_PID;
    struct PID_Struct_t Motor_R_Speed_PID;

    // timer中断
    hw_timer_t * PID_Timer = NULL;

    void set_speed(uint8_t channel, uint32_t value);
    float motor_PID(struct PID_Struct_t *pid);      // PID计算

    //
    void Encoder_Counter_Clear();
};

#endif
