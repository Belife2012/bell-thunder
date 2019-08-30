#include <sensor_us.h>

SENSOR_US ultraSonic(US_IIC_ADDR);
uint16_t Distance = 0;

void setup() 
{
  Serial.begin(115200);
  while (!Serial);
}

void loop() 
{
  byte rc;
  uint8_t US_Data[2];
  rc = ultraSonic.Get_Data(US_Data, 4);
  Distance = US_Data[1];
  Distance <<= 8;
  Distance += US_Data[0];
  
  // Serial.println("ultraSonic distance value is = ");
    Serial.printf(" %d", Distance);  
    Serial.printf(" %d", US_Data[0]);  
    Serial.printf(" %d", US_Data[1]); 
    Serial.println();
 
  delay(100);
}
