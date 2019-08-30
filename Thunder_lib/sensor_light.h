#include "iic_thunder.h"

#ifndef _LIGHTDETECT_I2C_
#define _LIGHTDETECT_I2C_

// 光电模块
#define LIGHT_IIC_ADDR 0x52 //光电模块I2C器件地址

#define LIGHT_IIC_REG_SETLIGHT     0x01
#define LIGHT_IIC_REG_READLIGHT    0x02

class SENSOR_LIGHT : public SENSOR_IIC
{
private:
  float dark_value[6] = {0,0,0,0,0,0};
  float bright_value[6] = {100,100,100,100,100,100};

public:
  SENSOR_LIGHT(int slave_address);

  byte Set_Led_Brightness(byte bright_level,unsigned char channel=0);
  void Set_Dark_Value(float new_value, unsigned char channel=0);
  void Set_Bright_Value(float new_value, unsigned char channel=0);
  byte Get_Light_Value_original(float *readValue, unsigned char channel=0);

  /*--------------Thunder IDE APIs: -------------*/
  float Get_Light_Value(unsigned char channel=0);
  byte Set_Operate_Mode(byte optMode,unsigned char channel=0);
  void Set_Extremum(int mode, float value, uint8_t sensorChannel=0);
  void Reset(unsigned char channel=0);
};

#endif
