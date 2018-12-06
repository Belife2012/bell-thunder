/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 灯板库文件
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
 * 功能列表(单色)：
 *  1.  void Setup(void);                               // 初始化单色点阵灯驱动
 *  2.  void Play_LED_HT16F35B(void);                   // 跑马灯
 *  3.  void Play_LED_HT16F35B_Show(int LED_Show_No);   // 内置单色点阵图案
 * 
 ************************************************/

#ifndef _THUNDER_DISPLAY_H_
#define _THUNDER_DISPLAY_H_

#include <Arduino.h>

// 彩色IC灯驱动
#include <XT1511_I2C.h>

// 单色点阵驱动
#include <HT16D35B.h>

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
class DOT_MATRIX_LED
{
  private:
    byte display_start_X, display_string_len;
    const byte (*play_string_data[40])[LED_MATRIX_COL_NUM][SINGLE_CHARACTER_WIDTH];
    byte play_string_index;
    byte play_string_offset;
    byte string_row_dots_num;
    byte start_roll_delay;

  public:
    void Setup(void);                               // 初始化单色点阵灯驱动
    void Play_LED_HT16F35B(void);                   // 跑马灯
    void Play_LED_HT16F35B_Show(int LED_Show_No);   // 内置单色点阵图案
    void Play_LED_String(const char *playString);         // 显示字符，长字符串以滚动方式呈现
    void Play_String_NextFrame(void);
    void Display_Picture(const byte picture_dots[LED_MATRIX_COL_NUM][LED_MATRIX_ROW_NUM],
                                    byte display_flag);
    void Move_Picture_To(int x, int y);
    void Set_Single_Dot(uint8_t x, uint8_t y);
    void Clear_Single_Dot(uint8_t x, uint8_t y);

    void Display_Number(float number);
};

#endif
