#ifndef _THUNDER_DISPLAY_H_
#define _THUNDER_DISPLAY_H_

#include <Arduino.h>

// 彩色IC灯驱动
#include <led_color.h>

// 单色点阵驱动
#include <ht16d35b.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_MATRIX_ROW_NUM      18
#define LED_MATRIX_COL_NUM      12
#define SINGLE_CHARACTER_WIDTH  6

typedef struct{
  byte rowIndex;
  byte comIndex;
} LedDotLocation;

#ifdef __cplusplus
}
#endif

// 单色点阵
class DISPLAY_THUNDER
{
  private:
    HT16D35B ht16d35b = HT16D35B(HT16D35B_DEVICE_ADDRESS_69);
    byte display_start_X, display_string_len;
    const byte (*play_string_data[40])[LED_MATRIX_COL_NUM][SINGLE_CHARACTER_WIDTH];
    byte play_string_index;
    byte play_string_offset;
    byte string_row_dots_num;
    byte start_roll_delay;

  public:
    void Setup(void);                               // 初始化单色点阵灯驱动
    void Play_LEDs(const unsigned char *data, int size);
    void Play_LED_HT16F35B(void);                   // 跑马灯
    void Play_LED_HT16F35B_Show(int LED_Show_No);   // 内置单色点阵图案
    void Play_LED_String(const char *playString);         // 显示字符，长字符串以滚动方式呈现
    void Play_LED_String(double number);
    void Play_LED_String(float number);
    void Play_LED_String(int number);
    void Play_LED_String(uint32_t number);
    void Play_String_NextFrame(void);
    void Display_Picture(const byte picture_dots[LED_MATRIX_COL_NUM][LED_MATRIX_ROW_NUM],
                                    byte display_flag);
    void Move_Picture_To(int x, int y);
    void Set_Single_Dot(uint8_t x, uint8_t y);
    void Clear_Single_Dot(uint8_t x, uint8_t y);

};

#endif
