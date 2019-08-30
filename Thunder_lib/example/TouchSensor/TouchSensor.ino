#include <sensor_touch.h>

SENSOR_TOUCH touch(TOUCH_IIC_ADDR);

void setup() 
{
  Serial.begin(115200);
  while (!Serial);
}

void loop() 
{
  uint8_t touch_data;
  
  touch_data = touch.Get_Event(4);
  
  // Serial.println("ultraSonic distance value is = ");
    Serial.printf(" %d", touch_data);  
    Serial.println();
 
  delay(100);
}
