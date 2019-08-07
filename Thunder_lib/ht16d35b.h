#include "iic_thunder.h"

#ifndef _HT16D35B_H_
#define _HT16D35B_H_

#define HT16D35B_DEVICE_ADDRESS_68            (0x68)    // 7bit Addrss
#define HT16D35B_DEVICE_ADDRESS_69            (0x69)    // 7bit Addrss

#define HT16D35B_CONTROL_COM                  (0x41)    // COM 7个
#define HT16D35B_CONTROL_ROW                  (0x42)    // ROW 28个

#define HT16D35B_MODE_GRAY                    (0x31)    // 灰度或者二进制模式，默认灰度
#define HT16D35B_COM_SCAN                     (0x32)    // 默认高电平扫描，0-7
#define HT16D35B_SYSTEM_CONTROL               (0x35)    // 0X03  开启系统振荡器，开启显示
#define HT16D35B_GLOBAL_BRIGHT                (0x37)    // 全局亮度设置：该命令用于控制 64 级 PWM 亮度

#define HT16D35B_DISPLAY_RAM                  (0x80)    // 灰度模式为00h~fbh

class HT16D35B : public SENSOR_IIC
{
  public:
    HT16D35B(int slave_address);                    // 配置I2C地址

    byte Setup(void);                               // 初始化设置
    byte LED_Show(const unsigned char *data, int size);   // 显示图案，参数*data为灯数据，参数size为数据长度
    
  private:
    byte device_detected;// 0为未插入设备，!0为已插入设备
};

#endif // _HT16D35B_H_
