#pragma once

#ifndef __SENSOR_IIC__
#define __SENSOR_IIC__

#include <Arduino.h>
#include <Wire.h>

#define MAX_CHANNEL_INDEX     (3)

#define CHECK_RANGE(value, min, max) do{if(value > max) { \
                                          value = max; \
                                        } else if (value < min) { \
                                          value = min; \
                                        } \
                                      } while(0)
#define CHECK_IIC_RETURN(ret) do{ if(ret != 0) { \
                                    log_e("%s IIC error", __FUNCTION__);\
                                  } \
                                }while(0)

class SENSOR_IIC
{
private:
  unsigned char _device_address;
  static uint8_t i2c_channel;
  static SemaphoreHandle_t xSemaphore_IIC;

  static uint8_t Set_I2C_Chanel(uint8_t channelData);

public:
  SENSOR_IIC(int slave_address);
  static uint8_t Select_Sensor_Channel(uint8_t sensorChannel);
  static uint8_t Select_Sensor_AllChannel();
  static void CreateSemaphoreIIC();
  static void Take_Semaphore_IIC();
  static void Give_Semaphore_IIC();

protected:
  unsigned char write(unsigned char memory_address,const unsigned char *data, unsigned char size, unsigned char channel = 0);
  unsigned char read(unsigned char memory_address, unsigned char *data, unsigned char size, unsigned char channel = 0);
};

#endif
