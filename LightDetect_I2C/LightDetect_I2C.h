#include "Sensor_IIC.h"

#ifndef _LIGHTDETECT_I2C_
#define _LIGHTDETECT_I2C_

class LIGHTDETECT_I2C : public SENSOR_IIC
{
public:
  LIGHTDETECT_I2C(int slave_address);

  byte Set_Led_Brightness(byte bright_level);
  byte Set_Operate_Mode(byte optMode);
  void Set_Dark_Value(unsigned char channel, float new_value);
  void Set_Bright_Value(unsigned char channel, float new_value);
  void Reset_All_Value(unsigned char channel );
  float Get_Light_Value(unsigned char channel );

  byte Get_Light_Value_original(float *readValue);

  int Thunder_Get_Light_Data(uint8_t sensorChannel);
  void Thunder_Set_Light_Mode(uint8_t sensorChannel, byte mode);
  void Thunder_Set_Light_Extremum(uint8_t sensorChannel, int mode, float value);
  void Thunder_Set_Light_Reset(uint8_t sensorChannel);
private:
  float dark_value[6] = {0,0,0,0,0,0};
  float bright_value[6] = {100,100,100,100,100,100};
};

#endif
