#ifndef _TOUCH_I2C_
#define _TOUCH_I2C_
#include "iic_thunder.h"

// 触碰模块
#define TOUCH_IIC_ADDR 0x10 //触碰模块I2C器件地址

#define TOUCH_ADDRESS_RGB       (0x01)
#define TOUCH_ADDRESS_VALUE     (0x02)
#define TOUCH_ADDRESS_RELEASE   (0x03)
#define TOUCH_ADDRESS_PRESS     (0x04)
#define TOUCH_ADDRESS_TOUCH     (0x05)
#define TOUCH_ADDRESS_CONTROL   (0xff)

typedef enum{
  TOUCH_EVENT_RELEASE = 0,
  TOUCH_EVENT_PRESS,
  TOUCH_EVENT_TOUCH,
} enum_touch_event;

class SENSOR_TOUCH : public SENSOR_IIC
{
public:
  SENSOR_TOUCH(int slave_address); // 配置I2C地址

  byte Reset_Mode(unsigned char channel=0);

  /*--------------Thunder IDE APIs: -------------*/
  byte Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue, unsigned char channel=0);
  bool Check_Event(int check_event, unsigned char channel=0);
  int Get_Event(uint8_t sensorChannel=0);
private:
  byte Get_Status(byte *readValue, unsigned char channel=0);
};

#endif
