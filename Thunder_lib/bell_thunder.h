#ifndef _THUNDER_H_
#define _THUNDER_H_

#include <Arduino.h>
#include "Esp.h"
#include <Wire.h>
#include "data_type.h"
#include "generalFunc.h"

// 音频
#include "speaker_thunder.h"
// 超声波
#include <sensor_us.h>
// 颜色识别
#include <sensor_color.h>
// 触碰传感器
#include <sensor_touch.h>
// 光电传感器
#include <sensor_light.h>
// 雷霆
#include <ble_thundergo.h>
#include <ble_client.h>
#include <display_thunder.h>
#include <motor_thunder.h>
#include <system_task.h>
#include <bell_barbette.h>
// 串口多机通信
#include <mult_devices.h>
// 九轴姿态传感器
#include "Sensor_Attitude.h"
// 火焰传感器
#include "sensor_flame.h"
// 风扇电动机
#include "motor_fan.h"
// 空气温湿度
#include "sensor_ht.h"
// 有毒气体传感器
#include "sensor_gas.h"
// 温度探头
#include "sensor_temp.h"
// 土壤湿度传感器
#include "sensor_soil.h"
// 声音传感器
#include "sensor_sound.h"
// 人体移动传感器
#include "sensor_human.h"
// 红外遥控器
#include "sensor_infrared.h"
// 遥控器
#include <sensor_remoter.h>

/* 通用宏 */
#define MOTOR_WITHOUT_CTRL_FOR_USER   1

// I2C
#define SDA_PIN            21   // SDA_PIN
#define SCL_PIN            22   // SCL_PIN

// 电池电压
#if 1
/* mV 干电池的设置 */
#define BATTERY_RESTART_VALUE   5000
#define BATTERY_LOW_VALUE       7000
#else
/* mV 镍氢电池的设置 */
#define BATTERY_RESTART_VALUE   6500
#define BATTERY_LOW_VALUE       7500
#endif
#define BATTERY_FILTER_NUM      10
#define BATTERY_ADC_PIN         35
#define ADC_R_1                 33.0   // 分压电阻 33K:100K
#define ADC_R_2                 100.0

// 舵机
#define SERVO_CHANNEL_0     0    // use 0 channel of 16 channels (started from zero)
#define SERVO_CHANNEL_1     1    // use 0 channel of 16 channels (started from zero)
#define SERVO_TIMER_13_BIT  13   // 定时器精度13位
#define SERVO_BASE_FREQ     50   // 频率50Hz
#define SERVO_A             5    // 舵机A口
// #define SERVO_B             2    // PIN 机械爪   //V15 V16
#define SERVO_B             25    // 舵机B口   //V16 V17

// 巡线IR
#define IR_1                36   // IR脚
#define IR_2                39   // IR脚

// 音频
#define AUDIO               16   // AUDIO脚
#define BUSY                4    // BUSY脚

// 通信
// #define ADD_BLE_NAME        0x00   //自定义蓝牙名称存储地址
#define COMMUNICATION_DATA_LENGTH_MAX     (20)
#define COMMUNICATION_DATA_LENGTH_MIN     (3) // 包头、长度、校验
#define COMMUNICATION_DATA_LOC_LENGTH     (1) // 数据包长度信息所在位置
#define COMMUNICATION_FIXED_LENGTH_MAX    (6)

// 超声波模块
#define ADD_I2C_US          0x01   //超声波模块I2C器件地址

// 颜色识别模块
#define ADD_I2C_COLOUR      0x38   //颜色识别模块I2C器件地址

// 触碰模块
#define TOUCH_ADDR_DEVICE   0x10   //触碰模块I2C器件地址

// 彩色灯条LED
#define CLOOUR_LED_DEVICE   0x11   //彩色灯条IIC器件地址

// 光电模块
#define LIGHT_ADDR_DEVICE   0x52   //光电模块I2C器件地址

// 按钮及LED指示
#define BUTTON_START        12
#define LED_INDICATION      2
#define BUTTON_FILTER_PARAM 50    // 小于50ms的按键状态被滤除
#define BUTTON_LONG_PRESS_TIME      1500  // 长按的时间长度
#define BUTTON_CONTINUE_CLICK_TIME  600   // 连续click的最大间隔

// 雷霆
extern BLE_THUNDERGO BLE_ThunderGo;
extern BLE_CLIENT BLE_Client;
extern MOTOR_THUNDER Motor_Thunder;
// 音频
extern SPEAKER_WT588 Speaker_Thunder;
// 超声波
extern SENSOR_US Sensor_Ultrasonic;
// 火焰传感器
extern SENSOR_FLAME Sensor_Flame;
// 风扇电动机
extern MOTOR_FAN Motor_Fan;
// 空气温湿度
extern SENSOR_HT Sensor_HumTemp;
// 有毒气体传感器
extern SENSOR_GAS Sensor_Gas;
// 温度探头
extern SENSOR_TEMP Sensor_Temp;
// 土壤湿度传感器
extern SENSOR_SOIL Sensor_Soil;
// 声音传感器
extern SENSOR_SOUND Sensor_Sound;
extern SENSOR_HUMAN Sensor_Human;
extern SENSOR_INFRARED Sensor_Infrared;
// 颜色识别
extern SENSOR_COLOR Sensor_Color;
// 彩色LED
extern LED_COLOR LED_Color;
// 触碰传感器
extern SENSOR_TOUCH Sensor_Touch;
// 光电传感器
extern SENSOR_LIGHT Sensor_Light;
// 单色LED
extern DISPLAY_THUNDER Display_Screen;
// 遥控器
extern SENSOR_REMOTER BLE_Remoter;
extern BELL_BARBETTE Barbette_Thunder;

extern uint32_t thunder_system_parameter;

class BELL_THUNDER
{
  private:
    // 舵机 PWM频率50Hz，0度时 脉宽最小500us，180度时 脉宽最大2500us，开机舵机居中为 90度
    // 20000[us] * 计数值 / 8191 = 输出[us]
    float Servo_MIN = 205;
    float Servo_Range = 819; 
    float servo_percent_max[2] = {100, 100};
    float servo_percent_min[2] = {-100, -100};
    float servo_percent_zero[2] = {0, 0};

    // 编码电机
    volatile uint8_t En_Motor_Flag = 0;
    int16_t L_Speed = 0;
    int16_t R_Speed = 0;

    // 电池电压
    uint16_t battery_filter_data[BATTERY_FILTER_NUM] = {0,0,0,0,0,0,0,0,0,0};
    uint16_t Low_Power_Old = 9000; //初始值设置为9000mV, 记录报警低电压时的电压值，每下降300mv报警一次
    uint32_t Battery_Value; //记录每次采集电压值时的电压值
    uint8_t lowpower_flag = 0; // 1 为低电压状态

    /* 按键、指示灯LED */
    // 毫秒为单位，精度为10ms
    struct struct_Led_Indication_Param{
      uint8_t led_indication_once_flag; // 当为 1 时，指示灯只做一次动作
      uint8_t led_indication_amount; // 一次动作闪灯多少回
      uint32_t led_indication_period; // 一次重复动作的时间（一次动作可能闪多次）
      uint32_t led_indication_on_duty; // 闪一次的亮灯时间
      uint32_t led_indication_off_duty; // 闪一次的熄灯时间
      uint32_t wait_led_timer = 0; // 设置好后等待多久开始闪灯动作
    } led_indication_param;
    uint32_t wait_key_timer = 0;// 倒计时器
    uint32_t button_press_time;
    uint32_t button_release_time; 
    uint32_t button_press_result_time;
    uint32_t button_release_result_time; 
    uint8_t button_press_counter;
    uint8_t button_status_record;
    uint8_t button_active;
    enum_Key_Value function_button_event = KEY_NONE;

    // 程序切换
    enum_Process_Status process_status = PROCESS_STOP;
    enum_Process_Status program_user;
    uint8_t system_program_mode = 0; // 开机后确定执行的系统程序：用户程序、APP程序。。。

    // 巡线IR
    // [0]是左边数据，[1]是右边数据
    uint8_t IR_Data[2];
    
    unsigned long Line_current_time = 0;
    unsigned long Line_last_time = 0;
    unsigned long Line_last_led_time = 0;
    unsigned long Line_last_sound_time = 0;

    #ifdef LIGHT_ROBOT_MODEL
    // 轻构型
    int Line_H_Speed = 90;
    int Line_M_Speed = 70;
    int Line_L_Speed = 50;
    int Line_B_Speed = -50;
    #else
    // 重构型（轮子版）
    int Line_H_Speed = 60;
    int Line_M_Speed = 50;
    int Line_L_Speed = 35;
    int Line_B_Speed = -35;

    #endif


    // 串口通信标志位
    bool need_communication = false;
    uint8_t Usart_Communication = 0;
    enum_Ble_Type ble_type = BLE_TYPE_NONE;

    // 内置表情/动画
    uint8_t LED_show_No = 0;
    uint8_t LED_counter = 0;
    unsigned long current_time = 0;
    unsigned long last_led_time = 0;
    uint16_t LED_delay_time = 0;

    uint8_t LED_show_3[8] = {14,15,16,14,17,14,17,14};
    uint8_t LED_show_4[5] = {18,19,18,19,18};
    uint8_t LED_show_5[8] = {20,21,22,23,24,25,26,27};
    uint16_t LED_time_5[8] = {1000,50,50,50,50,50,50,50};
    uint8_t LED_show_6[4] = {17,14,17,28};

    uint8_t LED_show_8[3] = {30,31,32};
    uint16_t LED_time_8[3] = {200,200,1000};
    uint8_t LED_show_9[5] = {33,34,33,34,33};

    uint8_t LED_show_10[3] = {35,36,37};
    uint8_t LED_show_11[4] = {14,38,14,38};
    uint8_t LED_show_12[4] = {39,40,41,42};

    uint8_t LED_show_13[5] = {43,44,43,44,43};
    uint8_t LED_show_14[9] = {45,46,47,45,46,47,45,46,47};
    uint8_t LED_show_15[5] = {72,73,74,75,76};

    // 超声波数据
    unsigned short US_Data[2] = {0,0};

    // 颜色识别数据
    unsigned short RGBC[4] = {0};
    float HSV[3] = {0};
    uint8_t Colour_Num = 0;

    // 模拟计时器
    uint32_t timer_value[5] = {0,0,0,0,0};

    // 多机通信
    std::vector<struct_Int_Message> recv_int_message;

  public:
    // 单色图案
    uint8_t LED_BUFF_Dot[29] =  
    {
      0x00, //地址
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00  //不用的
    };

    // 彩色灯图案
    uint8_t LED_Color_BUFF1[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t LED_Color_BUFF2[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8_t line_state = 0;  // 直->0; 左->1; 右->2; 假左->3; 假右->4; 未开始/停止->5;
  // bit0~bit7每一个bit代表一次记录值，总共记录8次，[0]是左边数据，[1]是右边数据
    uint8_t history_data[2]; 

    // 巡线模式
    uint8_t line_tracing_running = false;

    void Setup_All(void); // 所有模块初始化
    void Reset_All_Components(); // 复位各个组件
    void Stop_All(void);  // 全部终止(电机)

    // 电池电压
    void Setup_Battery(void);     // 电池电压检测初始化
    uint32_t Battery_Power_Filter(uint32_t new_data);
    uint32_t Get_Battery_Data(void); // 获取电池电压
    uint32_t Get_Battery_Value();
    void Indicate_Lowpower(uint32_t Battery_Voltage); // 电压低于 8V 后的提示

    /* 按键、指示灯LED */
    void Update_Function_Timer();
    void Setup_Led_Indication();
    void Set_Led_Indication_param();
    void Update_Led_Indication_Status(uint32_t &current_counter);
    void Setup_Function_Button();
    uint8_t Get_Function_Button_Status();
    enum_Key_Value Check_Function_Button_Value();
    bool Check_Function_Button_Event(enum_Key_Value key_event);
    void Set_Process_Status(enum_Process_Status new_status);
    void Update_Process_Status(enum_Key_Value button_event);
    void Reset_Process_Status();
    void Set_Program_User(enum_Process_Status new_program_user);
    void Set_Program_Run_Index(enum_Process_Status new_program);
    void Toggle_Led_mode(uint32_t period, uint32_t on_duty, uint32_t off_duty, uint8_t amount);

    /* 程序切换 */
    static enum_Program_Index program_change_to;

    // 编码电机  闭环计算
    void En_Motor(void);          // 编码电机  闭环计算
    void Enable_En_Motor(void);   // 打开编码电机计算
    void Enable_Drive_Car(void);
    void Enable_Motor_Position(void);
    void Disable_En_Motor(void);  // 关闭编码电机计算

    // 舵机
    void Setup_Servo(void);                 // 舵机初始化配置
    void Servo_Turn(int servo, float angle);  // 舵机角度控制
    void Servo_Turn_Percent(int servo, float percent); // 设置舵机范围 -100~100
    void Servo_Percent_Setting(int servo_index, 
            float max_value, float min_value, float zero_value, int direction);

    // 巡线IR
    void Setup_IR(void);                // 巡线IR传感器初始化配置
    void Get_IR_Data(uint8_t data[]);   // 获取巡线IR数据
    void Wait_For_Motor_Slow();
    void Line_Tracing(void);            // 使用开环控制电机的巡线模式
    void Motor_Slow_Go();               // 缓慢前行，拐弯后要恢复前行状态
    int Car_Shake_Left_Right(int dir);
    int Car_Rotate_90_Left_Right(int dir);
    void Line_Tracing_Speed_Ctrl(void); // 使用驾驶模式闭环控制的巡线模式

    // 动作表情
    void Start_Show(void);                  // 开机动画/声效
    void Set_LED_Show_No(uint8_t Show_No);  // 设置将要播放的内置动画编号
    void LED_Show(void);                    // 循环执行的内置动画控制程序
    
    // 指令通讯
    void Wait_Communication(void);            // 等待蓝牙连接动画 (有串口数据也跳出)
    void Get_Serial_Command(void);
    void Check_BLE_Communication(void); // 通信确认，蓝牙
    void Check_UART_Communication(void); // 通信确认，串口
    void Check_Protocol(void);      // 协议解析
    void Reset_Rx_Data(void);       // 清空接收数据
    void Set_Need_Communication(bool);
    void Set_Ble_Type(enum_Ble_Type new_type);

    // 计时器接口
    uint32_t Get_Virtual_Timer(uint32_t timer_index);
    void Reset_Virtual_Timer(uint32_t timer_index);

    // 多机通信
    void Open_Multi_Message();
    void Close_Multi_Message();
    int SendNameVarInt(unsigned char addr, char *name, int var_value);
    int RecvNameVarInt(char *name);
    void InitNameVarInt(char *name, int init_value);
};

extern BELL_THUNDER Thunder;

#endif