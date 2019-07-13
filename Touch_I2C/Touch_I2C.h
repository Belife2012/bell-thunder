#include "Sensor_IIC.h"

#ifndef _TOUCH_I2C_
#define _TOUCH_I2C_

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

class TOUCH_I2C : public SENSOR_IIC
{
public:
  TOUCH_I2C(int slave_address); // 配置I2C地址

  byte Get_Status(byte *readValue, unsigned char channel=0);
  byte Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue, unsigned char channel=0);
  byte Reset_Mode(unsigned char channel=0);
  bool Check_Touch_Event(enum_touch_event check_event, unsigned char channel=0);

  int Thunder_Get_Touch_Data(uint8_t sensorChannel);
  void Thunder_Set_Touch_Value(uint8_t sensorChannel, byte RedValue, byte GreenValue, byte BlueValue);
  bool Thunder_Get_Touch_Status(uint8_t sensorChannel, int status);
private:
};

#endif
