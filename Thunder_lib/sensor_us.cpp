#include <Arduino.h>
#include <Wire.h>
#include <sensor_us.h>

// #define DEBUG_US_IIC

// #define DELAY_BEFORE_READ_US

// 配置I2C地址
SENSOR_US::SENSOR_US(int slave_address) : SENSOR_IIC(slave_address)
{}

// 获取超声波数据
// data[0]=高8位数据，data[1]=低8位数据
/**
 * 返回未处理的数据
 * 如果返回 0 ，   代表获取了传感器的原始数据
 * 如果返回 非0 ， 则代表无法获取数据，数据无效
 */
byte SENSOR_US::Get_US_Data(unsigned short *data, unsigned char channel)
{
  byte rc;
  unsigned char val[2];

  rc = get_rawval(val, channel);

  if (rc != 0)
  {
    return (rc);
  }

  data[0] = (unsigned short)val[0];
  data[1] = (unsigned short)val[1] << 8;

  return (0);
}

// 获取超声波数据，0.1[cm]
// 高8位，低8位数据相加
/**
 * 量程, 最小3.0,  最大255.0，有效返回值为：3.0~255.0
 * 如果返回300.0， 代表超出量程；可能超出最大，可能超出最小
 * 如果返回 0 ，   则代表无法获取数据，数据无效
 */
float SENSOR_US::Get_US_cm(unsigned char channel)
{
  byte rc;
  unsigned char val[2];
  int data;

  rc = get_rawval(val, channel);

  if (rc != 0)
  {
    #ifdef DEBUG_US_IIC
    Serial.printf("### Sensor_Ultrasonic IIC R error: %d ###\n", rc);
    #endif
    return 0.0; //最小3.0, 如果返回 0 则代表无法获取数据，数据无效
  }

  data = ((unsigned short)val[0] + ((unsigned short)val[1] << 8));
  //Serial.printf("Sensor_Ultrasonic raw: %d \n", data);

  if (data < 30) //限制数据范围,最小3.0, 如果返回 0 则代表无法获取数据，数据无效
  {
    return 3.0;
  }
  else if (data > 3000)
  { //返回300.0，代表超出量程；
    return 300.0;
  }

  return (float)data / 10;
}

// 类内部使用，I2C通讯，读取超声波数据
byte SENSOR_US::get_rawval(unsigned char *data, unsigned char channel)
{
  byte rc;

  rc = read(0x01, data, 2, channel);

  // if (rc != 0) {
  //   Serial.println(F("### 无法获取超声波数据 ###"));
  // }

  return (rc);
}

int SENSOR_US::Thunder_Get_US_Data(uint8_t sensorChannel)
{
  float US_Data_cm = 0;
  US_Data_cm = Get_US_cm(sensorChannel);
  return (int) US_Data_cm;
}

int SENSOR_US::Thunder_Detect_Obstacle(uint8_t sensorChannel)
{
  float US_Data_cm = 0;
  US_Data_cm = Get_US_cm(sensorChannel);
  if(US_Data_cm == 0.0) {
      return 0;
  } else if(US_Data_cm == 300.0) {
      return 0;
  } else if(US_Data_cm < 20) {
      // 小于20cm认定为遇到障碍物
      return 1;
  } else {
      return 0;
  }
}