#include <Arduino.h>
#include <Wire.h>
#include <sensor_us.h>

#define DEBUG_US_IIC

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
byte SENSOR_US::Get_Data(uint8_t *data, unsigned char channel)
{
  byte rc;
  unsigned char val[2];

  rc = get_rawval(val, channel);

  if (rc != 0)
  {
    return (rc);
  }

  data[0] = (unsigned short)val[0];
  data[1] = (unsigned short)val[1];

  return (0);
}

// 类内部使用，I2C通讯，读取超声波数据
byte SENSOR_US::get_rawval(unsigned char *data, unsigned char channel)
{
  byte rc;
  byte counter=0;

  do{
    counter++;
    rc = read(0x01, data, 2, channel);
  } while(rc != 0 && counter < 5); // 超声波模块的IIC是模拟IIC，需要做错误处理

  return (rc);
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * 获取超声波数据[cm]
 * 
 * 量程, 最小3.0,  最大255.0，有效返回值为：3.0~255.0
 * 如果返回300.0， 代表超出量程；可能超出最大，可能超出最小
 * 如果返回 0 ，   则代表无法获取数据，数据无效
 */
float SENSOR_US::Get_Distance(unsigned char channel)
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

/**
 * 检测是否存在障碍物（距离 < 20cm）
 * 
 * @return：0 没有障碍物， 1 检测到障碍物
 */
int SENSOR_US::Detect_Obstacle(uint8_t sensorChannel)
{
  float US_Data_cm = 0;
  US_Data_cm = Get_Distance(sensorChannel);
  if(US_Data_cm == 0.0) {
      return 0;
  } else if(US_Data_cm < 20.0) {
      // 小于20cm认定为遇到障碍物
      return 1;
  } else {
      return 0;
  }
}