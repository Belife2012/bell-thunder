#pragma once

#ifndef __SENSOR_IIC__
#define __SENSOR_IIC__

#include <Arduino.h>
#include <Wire.h>
#include <Task_Mesg.h>

#define MAX_CHANNEL_INDEX     (3)

class SENSOR_IIC
{
public:
  SENSOR_IIC(int slave_address);
  static uint8_t Select_Sensor_Channel(uint8_t sensorChannel);
  static uint8_t Select_Sensor_AllChannel();

protected:
  unsigned char write(unsigned char memory_address,const unsigned char *data, unsigned char size, unsigned char channel = 0);
  unsigned char read(unsigned char memory_address, unsigned char *data, unsigned char size, unsigned char channel = 0);

private:
  unsigned char _device_address;
  static uint8_t i2c_channel;

  static uint8_t Set_I2C_Chanel(uint8_t channelData);
};

#endif
