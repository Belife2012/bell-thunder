/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 雷霆库文件
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
 *  1.  void Setup_All(void);                   // 所有模块初始化
 *  2.  void Stop_All(void);                    // 全部终止(电机)
 *  3.  void Setup_Battery(void);               // 电池电压检测初始化配置
 *  4.  float Get_Battery_Data(void);           // 获取电池电压
 *  5.  void En_Motor(void);                    // 编码电机  闭环计算
 *  6.  void Enable_En_Motor(void);             // 打开编码电机计算
 *  7.  void Disable_En_Motor(void);            // 关闭编码电机计算
 *  8.  void Setup_Servo(void);                 // 舵机初始化配置
 *  9.  void Servo_Turn(int servo, int angle);  // 舵机角度控制
 *  10. void Setup_IR(void);                    // 巡线IR传感器初始化配置
 *  11. void Get_IR_Data(uint8_t data[]);       // 获取巡线IR数据
 *  12. void Line_Tracing(void);                // 巡线模式
 *  13. void Start_Show(void);                  // 开机动画/声效
 *  14. void Wait_BLE(void);                    // 等待蓝牙连接动画 (有串口数据也跳出)
 *  15. void Set_LED_Show_No(uint8_t Show_No);  // 设置将要播放的内置动画编号
 *  16. void LED_Show(void);                    // 循环执行的内置动画控制程序
 *  17. void Check_Communication(void);         // 通信确认，蓝牙/串口
 *  18. void Check_Protocol(void);              // 协议解析
 *  19. void Reset_Rx_Data(void);               // 清空接收数据
 * 
 * 
 * 
 ************************************************/

#ifndef _THUNDER_H_
#define _THUNDER_H_

#include <Arduino.h>
#include <Wire.h>

// 音频
#include "WT588.h"

// 超声波
#include <US_I2C.h>

// 颜色识别
#include <BH1745NUC.h>

// 触碰传感器
#include <Touch_I2C.h>

// 光电传感器
#include <LightDetect_I2C.h>

// 雷霆
#include <Thunder_BLE.h>
#include <Thunder_Display.h>
#include <Thunder_Motor.h>
#include <Task_Mesg.h>

// 使用新的 Version 变量Version_FW
// #define VERSION            0.21   // version

// I2C
#define SDA_PIN            21   // SDA_PIN
#define SCL_PIN            22   // SCL_PIN

// 电池电压
#define BATTERY_ADC_PIN    35
#define ADC_R_1            33.0   // 分压电阻 33K:100K
#define ADC_R_2            100.0

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

// // 自定义蓝牙名称
// #define ADD_BLE_NAME        0x00   //自定义蓝牙名称存储地址

// 超声波模块
#define ADD_I2C_US          0x01   //超声波模块I2C器件地址

// 颜色识别模块
#define ADD_I2C_COLOUR      0x38   //颜色识别模块I2C器件地址

// 触碰模块
#define TOUCH_ADDR_DEVICE   0x10   //触碰模块I2C器件地址

// 光电模块
#define LIGHT_ADDR_DEVICE   0x52   //光电模块I2C器件地址

// 雷霆
extern THUNDER_BLE Thunder_BLE;
extern THUNDER_MOTOR Thunder_Motor;

// 音频
extern WT588 Speaker;

// 超声波
extern US_I2C US;

// 颜色识别
extern BH1745NUC Colour_Sensor;

// 彩色LED
extern XT1511_I2C I2C_LED;

// 触碰传感器
extern TOUCH_I2C Touch_Sensor;

// 光电传感器
extern LIGHTDETECT_I2C Light_Sensor;

// 单色LED
extern DOT_MATRIX_LED Dot_Matrix_LED;
extern HT16D35B HT16D35B;// IIC Address: 0x69

class THUNDER
{
  private:
    // 舵机
    // 20000[us] * 计数值 / 8191 = 输出[us]
    int Servo_MIN = 205;
    int Servo_Range = 819; 

    // 编码电机
    uint8_t En_Motor_Flag = 0;
    int16_t L_Speed = 0;
    int16_t R_Speed = 0;

    // 电池电压
    uint16_t ADC_Battery = 0;
    uint8_t lowpower_flag = 0; // 1 为低电压状态

    // 巡线IR
    // [0]是左边数据，[1]是右边数据
    uint8_t IR_Data[2];
    
    unsigned long Line_current_time = 0;
    unsigned long Line_last_time = 0;
    unsigned long Line_last_led_time = 0;
    unsigned long Line_last_sound_time = 0;

    // 150 100 50 -50
    int Line_H_Speed = 160;
    int Line_M_Speed = 120;
    int Line_L_Speed = 80;
    int Line_B_Speed = -80;

    // 串口通信标志位
    uint8_t Usart_Communication = 0;

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
    
    // I2C端口选通
    uint8_t I2C_channel_opened;
    uint8_t Set_I2C_Chanel(uint8_t channelData);

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
    uint8_t I2C_LED_BUFF1[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t I2C_LED_BUFF2[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8_t line_state = 0;  // 直->0; 左->1; 右->2; 假左->3; 假右->4; 未开始/停止->5;
  // bit0~bit7每一个bit代表一次记录值，总共记录8次，[0]是左边数据，[1]是右边数据
    uint8_t history_data[2]; 

    void Setup_All(void); // 所有模块初始化
    void Stop_All(void);  // 全部终止(电机)

    // 电池电压
    void Setup_Battery(void);     // 电池电压检测初始化
    float Get_Battery_Data(void); // 获取电池电压
    float Indicate_Lowpower(uint16_t adc_value); // 电压低于 8V 后的提示

    // 编码电机  闭环计算
    void En_Motor(void);          // 编码电机  闭环计算
    void Enable_En_Motor(void);   // 打开编码电机计算
    void Disable_En_Motor(void);  // 关闭编码电机计算

    // 只能开环控制电机时使用
    void Get_Queue_Encoder(void);

    // 舵机
    void Setup_Servo(void);                 // 舵机初始化配置
    void Servo_Turn(int servo, int angle);  // 舵机角度控制

    // 巡线IR
    void Setup_IR(void);                // 巡线IR传感器初始化配置
    void Get_IR_Data(uint8_t data[]);   // 获取巡线IR数据
    void Line_Tracing(void);            // 巡线模式

    // 动作表情
    void Start_Show(void);                  // 开机动画/声效
    void Wait_BLE(void);                    // 等待蓝牙连接动画 (有串口数据也跳出)
    void Set_LED_Show_No(uint8_t Show_No);  // 设置将要播放的内置动画编号
    void LED_Show(void);                    // 循环执行的内置动画控制程序
    
    // 协议解析
    void Check_Communication(void); // 通信确认，蓝牙/串口
    void Check_Protocol(void);      // 协议解析
    void Reset_Rx_Data(void);       // 清空接收数据

    // 传感器端口选择
    uint8_t Select_Sensor_Channel(uint8_t sensorChannel);
    uint8_t Select_Sensor_AllChannel();
};

extern THUNDER Thunder;

#endif
