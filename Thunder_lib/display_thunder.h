#ifndef _THUNDER_DISPLAY_H_
#define _THUNDER_DISPLAY_H_

#include <Arduino.h>

// 彩色IC灯驱动
#include <led_color.h>

// 单色点阵驱动
#include <ht16d35b.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LED_MATRIX_ROW_NUM 18
#define LED_MATRIX_COL_NUM 12
#define SINGLE_CHARACTER_WIDTH 6

typedef struct
{
    byte rowIndex;
    byte comIndex;
} LedDotLocation;

#ifdef __cplusplus
}
#endif

// 单色点阵
class DISPLAY_SCREEN
{
private:
    HT16D35B ht16d35b = HT16D35B(HT16D35B_DEVICE_ADDRESS_69);
    byte display_start_X, display_string_len;
    const byte (*play_string_data[40])[LED_MATRIX_COL_NUM][SINGLE_CHARACTER_WIDTH];
    byte play_string_index;
    byte play_string_offset;
    byte string_row_dots_num;
    byte start_roll_delay;

    // 内置表情/动画
    uint8_t LED_show_No = 0;
    uint8_t LED_counter = 0;
    unsigned long current_time = 0;
    unsigned long last_led_time = 0;
    uint16_t LED_delay_time = 0;
    const uint8_t LED_show_3[8] = {14, 15, 16, 14, 17, 14, 17, 14};
    const uint8_t LED_show_4[5] = {18, 19, 18, 19, 18};
    const uint8_t LED_show_5[8] = {20, 21, 22, 23, 24, 25, 26, 27};
    const uint16_t LED_time_5[8] = {1000, 50, 50, 50, 50, 50, 50, 50};
    const uint8_t LED_show_6[4] = {17, 14, 17, 28};
    const uint8_t LED_show_8[3] = {30, 31, 32};
    const uint16_t LED_time_8[3] = {200, 200, 1000};
    const uint8_t LED_show_9[5] = {33, 34, 33, 34, 33};
    const uint8_t LED_show_10[3] = {35, 36, 37};
    const uint8_t LED_show_11[4] = {14, 38, 14, 38};
    const uint8_t LED_show_12[4] = {39, 40, 41, 42};
    const uint8_t LED_show_13[5] = {43, 44, 43, 44, 43};
    const uint8_t LED_show_14[9] = {45, 46, 47, 45, 46, 47, 45, 46, 47};
    const uint8_t LED_show_15[5] = {72, 73, 74, 75, 76};

public:
    typedef unsigned char t_picture_buff[LED_MATRIX_COL_NUM][LED_MATRIX_ROW_NUM];
    class PICTURE_BUFF
    {
    private:
        t_picture_buff data;

    public:
        PICTURE_BUFF() {memset(data, 0, sizeof(data));}
        PICTURE_BUFF(unsigned char s_data[LED_MATRIX_COL_NUM][LED_MATRIX_ROW_NUM])
        {
            memcpy(data, s_data, sizeof(data));
        }
    };

    void Setup(void);     // 初始化单色点阵灯驱动
    void Test_LEDs(void); // 跑马灯
    void Play_LEDs(const unsigned char *data, int size);
    void Play_Thunder_Picture(int picture_index); // 内置单色点阵图案
    void Play_LED_String(const char *playString); // 显示字符，长字符串以滚动方式呈现
    void Play_LED_String(double number);
    void Play_LED_String(float number);
    void Play_LED_String(int number);
    void Play_LED_String(uint32_t number);
    void Play_String_NextFrame(void);
    void Display_Picture(const byte picture_dots[LED_MATRIX_COL_NUM][LED_MATRIX_ROW_NUM],
                         byte display_flag = 1);
    void Move_Picture_To(int x, int y);
    void Set_Single_Dot(uint8_t x, uint8_t y);
    void Clear_Single_Dot(uint8_t x, uint8_t y);
    void Play_Animation(uint8_t Show_No); // 设置将要播放的内置动画编号
    void Animation_Control(void);         // 循环执行的内置动画控制程序
};

#endif
