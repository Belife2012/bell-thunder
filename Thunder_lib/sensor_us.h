#include "iic_thunder.h"

#ifndef _US_I2C_H_
#define _US_I2C_H_

class SENSOR_US : public SENSOR_IIC
{
public:
  SENSOR_US(int slave_address); // 配置I2C地址

  byte Get_US_Data(unsigned short *data, unsigned char channel=0); // 获取超声波数据
  float Get_US_cm(unsigned char channel=0);                  // 获取超声波数据，0.1[cm]

  int Thunder_Get_US_Data(uint8_t sensorChannel);
  int Thunder_Detect_Obstacle(uint8_t sensorChannel);

private:

  byte get_rawval(unsigned char *data, unsigned char channel=0);                                              // 类内部使用，I2C通讯，读取超声波数据
};

#endif
