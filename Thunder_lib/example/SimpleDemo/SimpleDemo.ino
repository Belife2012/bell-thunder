#include "sensor_touch.h"

void setup()
{
    Serial.begin(115200);
    SENSOR_IIC::IIC_Init();
}

SENSOR_TOUCH touch(TOUCH_IIC_ADDR);
void loop()
{
    int status;
    
    status = touch.Get_Event(1);
    Serial.printf("touch: %d\n", status);

    delay(100);
}