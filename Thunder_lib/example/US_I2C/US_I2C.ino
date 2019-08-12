#include <Wire.h>
#include <sensor_us.h>

SENSOR_US SENSOR_US(0x01);
unsigned short L_Distance = 0;
unsigned short M_Distance = 0;
unsigned short Distance = 0;

void setup() 
{
  byte rc;
  Serial.begin(115200);
  while (!Serial);
  Wire.begin(); //  开启I2C总线，主
}

void loop() 
{
  byte rc;
  unsigned short US_Data[4] = {1,2,3,4};
  rc = SENSOR_US.Get_Data(US_Data);
  L_Distance = US_Data[0];
  M_Distance = US_Data[1];
  Distance = L_Distance + M_Distance;
  
  Serial.write("SENSOR_US distance value is = ");
    Serial.println(Distance);  
    Serial.println(US_Data[0]);  
    Serial.println(US_Data[1]); 
 
  delay(500);
}
