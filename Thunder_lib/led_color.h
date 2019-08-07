#include "iic_thunder.h"

#ifndef _XT1511_I2C_H_
#define _XT1511_I2C_H_

#define XT1511_I2C_DEVICE_ADDRESS            (0x11)    // 7bit Addrss

#define XT1511_I2C_CONTROL_REG               (0x00)    // 总共控制的灯的个数，默认为12个 (预留)
//灯状态寄存器 从 0x01 开始

#define XT1511_I2C_COM_OFF                   (0xA0)    // 全关，立即刷新
#define XT1511_I2C_COM_UODATA                (0xA1)    // 按照现有数据刷新
#define XT1511_I2C_COM_SHOW                  (0xA2)    // 刷新预存数据

#define RGB_LED_AMOUNT      (12)
#define RGB_LED_DATA_SIZE   (RGB_LED_AMOUNT*3)

typedef enum{
  COLOR_MODE_STATIC = 0,
  COLOR_MODE_BLINK,
  COLOR_MODE_ROLL,
  COLOR_MODE_BREATH
}enum_ColorLED_Mode_Type;

class LED_COLOR : public SENSOR_IIC
{
  public:
    LED_COLOR(uint8_t slave_address);  // 配置I2C地址

    void Set_LED_Num(uint8_t number);   // 写入一共可以控制多少个灯

    void Set_LED_Data(uint8_t address, uint8_t r, uint8_t g, uint8_t b);  // 写入单个寄存器数据
    void Set_LEDs_Data(uint8_t address, uint8_t *data, uint8_t size);     // 写入多个寄存器数据

    void LED_OFF(void);                 // 0xA0  全关，立即刷新
    void LED_Updata(void);              // 0xA1  按照现有数据刷新
    void LED_Show(uint8_t number);      // 0xA2  刷新预存数据 (指令预留)

    void Set_LED_Dynamic(uint8_t dynamicMode);
    void LED_Flush(void);

  private:
      byte LEDs_Data[RGB_LED_DATA_SIZE];
      byte LEDs_DataResult[RGB_LED_DATA_SIZE];

      byte LED_Dynamic;
      unsigned int ledDynamicIndex;

      void LED_Flush_Static(void);
      void LED_Flush_Breath(void);
      void LED_Flush_Blink(void);
      void LED_Flush_Roll(void);
};

#endif // _XT1511_I2C_H_
