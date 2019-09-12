#pragma once

#ifndef __SENSOR_IIC__
#define __SENSOR_IIC__

#include <Arduino.h>
#include <Wire.h>

// I2C
#define SDA_PIN 21 // SDA_PIN
#define SCL_PIN 22 // SCL_PIN
// Port4 IIC
#define PORT4_SDA_PIN 39
#define PORT4_SCL_PIN 36
// PortU IIC
#define PORTU_SCL_PIN		18
#define PORTU_SDA_PIN		32

#define MAX_CHANNEL_INDEX     (3)

#define CHECK_RANGE(value, min, max) do{if( value > (max) ) { \
                                          value = (max); \
                                        } else if ( value < (min) ) { \
                                          value = (min); \
                                        } \
                                      } while(0)
#define CHECK_IIC_RETURN(ret) do{ if(ret != 0) { \
                                    log_e("%s IIC error", __FUNCTION__);\
                                  } \
                                }while(0)

typedef enum {
  DEFAULT_PORTS = 0,
  ENABLE_PORT_4,
} enum_AllChannel_Flag;

class SENSOR_IIC
{
private:
  unsigned char _device_address;
  static uint8_t i2c_channel;
  static bool i2c_enable;
  static SemaphoreHandle_t xSemaphore_IIC;
  bool i2c_u_enable = false;

  static inline void SELECT_IIC_CHANNEL(uint8_t channel);

  static uint8_t Select_Sensor_Channel(uint8_t sensorChannel);
public:
  typedef enum {
    PORT_1 = 1,
    PORT_2 = 2,
    PORT_3 = 3,
    PORT_4 = 4,
    PORT_A = 5,
    PORT_B = 6,
    PORT_U = 7
  } enum_iic_port;

  SENSOR_IIC(int slave_address);
  static void IIC_Init();
  static uint8_t Select_Sensor_AllChannel(uint8_t flag=DEFAULT_PORTS);
  static void CreateSemaphoreIIC();
  static void Take_Semaphore_IIC();
  static void Give_Semaphore_IIC();
  static void Set_Port4_IIC(bool setting);

protected:
  unsigned char write(unsigned char memory_address,const unsigned char *data, unsigned char size, unsigned char channel = 0);
  unsigned char read(unsigned char memory_address, unsigned char *data, unsigned char size, unsigned char channel = 0);
};

#endif
