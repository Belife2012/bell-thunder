#ifndef _TOUCH_I2C_
#define _TOUCH_I2C_

#define TOUCH_ADDRESS_RGB       (0x01)
#define TOUCH_ADDRESS_VALUE     (0x02)
#define TOUCH_ADDRESS_RELEASE   (0x03)
#define TOUCH_ADDRESS_PRESS     (0x04)
#define TOUCH_ADDRESS_TOUCH     (0x05)
#define TOUCH_ADDRESS_CONTROL   (0xff)

typedef enum{
  TOUCH_EVENT_RELEASE = 1,
  TOUCH_EVENT_PRESS,
  TOUCH_EVENT_TOUCH,
} enum_touch_event;

class TOUCH_I2C
{
public:
  TOUCH_I2C(int slave_address); // 配置I2C地址

  byte Get_Status(byte *readValue);
  byte Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue);
  byte Reset_Mode(void);
  bool Check_Touch_Event(enum_touch_event check_event);

private:
  byte _device_address;
  byte write(unsigned char memory_address, unsigned char *data, unsigned char size);
  byte read(unsigned char memory_address, unsigned char *data, unsigned char size);
};

#endif
