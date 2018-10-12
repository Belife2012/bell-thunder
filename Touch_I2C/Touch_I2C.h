#ifndef _TOUCH_I2C_
#define _TOUCH_I2C_

#include <Arduino.h>

class TOUCH_I2C
{
public:
    byte Get_Status(void);
    void Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue);
};

#endif
