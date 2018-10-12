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

// 单色点阵
class DOT_MATRIX_LED
{
  private:

  public:
    void Setup(void);                               // 初始化单色点阵灯驱动
    void Play_LED_HT16F35B(void);                   // 跑马灯
    void Play_LED_HT16F35B_Show(int LED_Show_No);   // 内置单色点阵图案
    void Play_LED_String(char *playString);         // 显示字符，长字符串以滚动方式呈现
};

#endif
