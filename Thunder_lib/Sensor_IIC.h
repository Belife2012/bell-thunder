#pragma once

#ifndef __SENSOR_IIC__
#define __SENSOR_IIC__

#include <Arduino.h>
#include <Wire.h>
#include <Task_Mesg.h>

class SENSOR_IIC
{
public:
  SENSOR_IIC(int slave_address);

  char Get_Flame_Angle();
  char Get_Flame_Intensity();

protected:
  unsigned char write(unsigned char memory_address, unsigned char *data, unsigned char size);
  unsigned char read(unsigned char memory_address, unsigned char *data, unsigned char size);

private:
  unsigned char _device_address;
};

#endif
