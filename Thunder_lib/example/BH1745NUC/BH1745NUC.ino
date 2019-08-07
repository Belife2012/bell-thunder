/*****************************************************************************
  SENSOR_COLOR.ino

******************************************************************************/
#include <Wire.h>
#include <sensor_color.h>

SENSOR_COLOR bh1745nuc(BH1745NUC_DEVICE_ADDRESS_38);

void setup() 
{
  byte rc;

  Serial.begin(115200);
  while (!Serial);
  
  Wire.begin();
  
  rc = bh1745nuc.Setup();
}

void loop() 
{
  byte rc;
  unsigned short RGBC[4];
  float HSV[3];
  
  rc = bh1745nuc.Get_RGBC_Data(RGBC);
  if (rc == 0) 
  {
    Serial.write("SENSOR_COLOR (RED)   = ");
    Serial.println(RGBC[0]);
    Serial.write("SENSOR_COLOR (GREEN) = ");
    Serial.println(RGBC[1]);
    Serial.write("SENSOR_COLOR (BLUE)  = ");
    Serial.println(RGBC[2]);
    Serial.write("SENSOR_COLOR (CLEAR) = ");
    Serial.println(RGBC[3]);    
    Serial.println();
  }

  bh1745nuc.RGBtoHSV(RGBC,HSV);
  Serial.printf("SSSSSSSSSS ___ H  %f ___ S : %f ___ V : %f ___ SSSSSSSSSS\n",HSV[0],HSV[1],HSV[2]);  //HSV

  delay(500);
}
