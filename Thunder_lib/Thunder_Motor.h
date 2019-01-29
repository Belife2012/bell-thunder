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
#include <Task_Mesg.h>

#define MOTOR_DRIVER_IC_TB6612    (0x01)
#define MOTOR_DRIVER_IC_PT5126    (0x02)
#define MOTOR_DRIVER_IC           MOTOR_DRIVER_IC_PT5126

// 电机
// TB6612驱动驱动直流无刷电机
#if (MOTOR_DRIVER_IC == MOTOR_DRIVER_IC_TB6612)
#define MOTOR_CHANNEL_2     2   //0--16
#define MOTOR_CHANNEL_3     3
#define MOTOR_TIMER_13_BIT  13 //分辨率13位
#define MOTOR_BASE_FREQ     10000 //频率
#define MOTOR_L_PWM         18
#define MOTOR_L_IN1         32
#define MOTOR_L_IN2         19
#define MOTOR_R_PWM         27
#define MOTOR_R_IN1         13
#define MOTOR_R_IN2         23

#elif (MOTOR_DRIVER_IC == MOTOR_DRIVER_IC_PT5126)
#define MOTOR_CHANNEL_2      2   //0--16
#define MOTOR_CHANNEL_3      3
#define MOTOR_CHANNEL_4      4  //0--16
#define MOTOR_CHANNEL_5      5
#define MOTOR_TIMER_13_BIT   13 //分辨率13位
#define MOTOR_BASE_FREQ      10000 //频率
#define PWM_L_A 27
#define PWM_L_B 19
#define PWM_R_A 13
#define PWM_R_B 23

#endif

#define MOTOR_MAX_DUTY      8191 //最大8191 // 8191 = 2 ^ 13 - 1 //电机选型的原因在此进行最大输出限制   //安全duty 5500
#define MOTOR_INPUT_MAX     255
#define LIMIT_SPEED         10  //电机停止速度，滑行速度

#define MOTOR_INDEX_LEFT          (1)
#define MOTOR_INDEX_RIGHT         (2)
#define MOTOR_DIRECTION_LEFT      (1)
#define MOTOR_DIRECTION_RIGHT     (2)

// 编码器
#define DEGREES_EVERY_CIRCLE      360
#define ENCODER_NUM_EVERY_CIRCLE  216   // 电机编码器磁铁是四对磁极对，减速比是27

#define EN_L_A 14 //编码器A 中断  左
#define EN_L_B 33 //编码器A
#define EN_R_A 34 //编码器B 中断  右
#define EN_R_B 17 //编码器B

#define PID_Default_Kp 10.0      //P 3.3   8.0   8.0
#define PID_Default_Ki 5.6      //I 2.85  4.0   3.0
#define PID_Default_Kd 3.5      //D       3.5   3.5
#define PID_dt 50               //[ms]

// drive car
#define MAX_DRIVE_OUTPUT       255
#define MAX_DRIVE_SPEED       10.0
#define MAX_DRIVE_DIRECTION   100.0

// PID时间中断
extern volatile SemaphoreHandle_t Timer_PID_Flag;

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

struct DriveCarPid_Struct{
  int16_t* P_direction_divisor;

  float left_speed_target;
  float right_speed_target;
  float new_left_target;
  float new_right_target;
  float left_speed_diff;
  float right_speed_diff;

  float direction;
  float dir_diff;

  float Kp = 10.0;
  float Ki = 5.6;
  float Kd = 0;

  uint32_t last_pid_time = 0;

  float OutP_left = 0;
  float OutI_left = 0;
  float OutI_left_last = 0;
  float OutD_left = 0;
  float OutP_right = 0;
  float OutI_right = 0;
  float OutI_right_last = 0;
  float OutD_right = 0;
  float Out_left = 0;
  float Out_right = 0;
};

struct MotorRunning_Struct{
  //0代表left 和 right的控制都起效；1代表left控制起效，right无效; 2代表right控制起效，left无效
  byte motor_select;

  //0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
  byte running_mode;

  float mode_data;

  float left_motor_speed;
  float right_motor_speed;
};

struct MotorTurnning_Struct{
  //0: 无模式；1：控制时间（秒）；2：控制角度（度）；3：控制圈数（圈）
  byte turnning_mode;

  float mode_data;

  // 取值范围 -100~100
  float turn_percent;
  float motor_speed;
};

class THUNDER_MOTOR
{
  public:
    // 定时器
    void Setup_PID_Timer(void);         // 配置PID用定时器
    void Disable_PID_Timer(void);     // 移除PID用定时器(会影响其它用到此定时器的功能)
    void Enable_PID_Timer(void);

    // 开环电机
    void Setup_Motor(void);                                 // 配置电机
    void Motor_Move(int motor, int speed, int direction);   // 开环电机控制函数
    void Motor_Brake(int motor);
    void Motor_Free(int motor);
    void Set_L_Motor_Output( int M_output );
    void Set_R_Motor_Output( int M_output );
    void Set_L_Motor_Power( int Lpower );
    void Set_R_Motor_Power( int Rpower );

    // 闭环电机
    void PID_Reset(struct PID_Struct_t *pid);                               // 重置PID计算过程值
    void PID_Reset();
    void PID_Init(struct PID_Struct_t *pid, float Kp, float Ki, float Kd);  // PID参数初始化
    void All_PID_Init();                                                    // 按默认PID参数初始化左右电机
    void Setup_Motor_PID(void);                                             // 配置左右两个电机编码器
    void PID_Speed(void);                                                   // 按PID输出控制左右两个电机

    void Drive_Car_Control();
    void Set_Car_Speed_Direction(float speed, float direction);

    void Set_L_Target(float target);  // 设定左轮目标速度(编码器计数值)
    void Set_R_Target(float target);  // 设定右轮目标速度(编码器计数值)
    void Control_Motor_Running(MotorRunning_Struct &running_data);
    void Control_Motor_Turnning(MotorTurnning_Struct &turnning_data);
    MotorRunning_Struct motor_running_data;
    MotorTurnning_Struct motor_turnning_data;

    void Update_Encoder_Value();

    int16_t Get_L_Speed(void);  // 获取左轮速度(编码器计数值)
    int16_t Get_R_Speed(void);  // 获取右轮速度(编码器计数值)

    int16_t Get_L_Target(void);  // 获取左轮目标(编码器计数值)
    int16_t Get_R_Target(void);  // 获取右轮目标(编码器计数值)

    int32_t Get_L_RotateValue(void); // 获取左轮旋转量（这个值是累积的，调用清零接口时才会清零）
    int32_t Get_R_RotateValue(void); // 获取右轮旋转量（这个值是累积的，调用清零接口时才会清零）
    void Clear_L_RotateValue(void); // 清零左轮旋转量
    void Clear_R_RotateValue(void); // 清零右轮旋转量
  
  private:
    // 电机
    boolean inPin1 = LOW;
    boolean inPin2 = HIGH;
    uint8_t valueMax = 255;

    struct PID_Struct_t Motor_L_Speed_PID;
    struct PID_Struct_t Motor_R_Speed_PID;

    // drive car 
    struct DriveCarPid_Struct drive_car_pid;
    float drive_speed = 0;
    float drive_direction = 0;

    // timer中断
    hw_timer_t * PID_Timer = NULL;

    void set_speed(uint8_t motor_channel,uint32_t speed);
    float motor_PID(struct PID_Struct_t *pid);      // PID计算
    void Calculate_Left_Control();
    void Calculate_Right_Control();

};

#endif
