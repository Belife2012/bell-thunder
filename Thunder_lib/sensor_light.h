#include "iic_thunder.h"

#ifndef _LIGHTDETECT_I2C_
#define _LIGHTDETECT_I2C_

class SENSOR_LIGHT : public SENSOR_IIC
{
public:
  SENSOR_LIGHT(int slave_address);

  byte Set_Led_Brightness(byte bright_level,unsigned char channel=0);
  byte Set_Operate_Mode(byte optMode,unsigned char channel=0);
  void Set_Dark_Value(float new_value, unsigned char channel=0);
  void Set_Bright_Value(float new_value, unsigned char channel=0);
  void Reset_All_Value(unsigned char channel=0);
  float Get_Light_Value(unsigned char channel=0);

  byte Get_Light_Value_original(float *readValue, unsigned char channel=0);

  int Thunder_Get_Light_Data(uint8_t sensorChannel);
  void Thunder_Set_Light_Mode(uint8_t sensorChannel, byte mode);
  void Thunder_Set_Light_Extremum(uint8_t sensorChannel, int mode, float value);
  void Thunder_Set_Light_Reset(uint8_t sensorChannel);
private:
  float dark_value[6] = {0,0,0,0,0,0};
  float bright_value[6] = {100,100,100,100,100,100};
};

#endif
