/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 超声波模块库文件
 * 
 *   创建日期： 20180606
 *   作者：     宋博伟
 *   邮箱：     songbw123@163.com
 *
 *   版本：     v0.2
 *   修改日期   20180721
 *   修改：     宋博伟
 *   邮箱：     songbw123@163.com
 *   修改内容： 
 * 
 *   
 * 
 * 功能列表：
 *  1.  US_I2C(int slave_address);                // 配置I2C地址
 *  2.  byte Get_US_Data(unsigned short *data);   // 获取超声波数据
 *  3.  float Get_US_cm(void);                    // 获取超声波数据，0.1[cm] 
 * 
 ************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <US_I2C.h>
#include <Task_Mesg.h>

// #define DELAY_BEFORE_READ_US

// 配置I2C地址
US_I2C::US_I2C(int slave_address)
{
  _device_address = slave_address;
}

// 获取超声波数据
// data[0]=高8位数据，data[1]=低8位数据
/**
 * 返回未处理的数据
 * 如果返回 0 ，   代表获取了传感器的原始数据
 * 如果返回 非0 ， 则代表无法获取数据，数据无效
 */
byte US_I2C::Get_US_Data(unsigned short *data)
{
  byte rc;
  unsigned char val[2];

  rc = get_rawval(val);

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
float US_I2C::Get_US_cm(void)
{
  byte rc;
  unsigned char val[2];
  int data;

  rc = get_rawval(val);

  if (rc != 0)
  {
    Serial.printf("### US IIC R error: %d ###\n", rc);
    return 0.0; //最小3.0, 如果返回 0 则代表无法获取数据，数据无效
  }

  data = ((unsigned short)val[0] + ((unsigned short)val[1] << 8));
  //Serial.printf("US raw: %d \n", data);

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
byte US_I2C::get_rawval(unsigned char *data)
{
  byte rc;

  rc = read(0X01, data, 2);

  // if (rc != 0) {
  //   Serial.println(F("### 无法获取超声波数据 ###"));
  // }

  return (rc);
}

// 类内部使用，I2C通讯，发送
byte US_I2C::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;

  Task_Mesg.Take_Semaphore_IIC();
  Wire.beginTransmission(_device_address);
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission();
  if (rc == I2C_ERROR_BUSY)
  {
    Wire.reset();
  }
  Task_Mesg.Give_Semaphore_IIC();
  return (rc);
}

// 类内部使用，I2C通讯，发送并读取
byte US_I2C::read(unsigned char memory_address, unsigned char *data, int size)
{
  byte rc;
  unsigned char cnt = 0;

  uint32_t lastMillis;
  uint32_t currentMillis;

  Task_Mesg.Take_Semaphore_IIC();
  Wire.beginTransmission(_device_address); // 开启发送
#ifdef DELAY_BEFORE_READ_US
  lastMillis = millis();
  while (1)
  {
    currentMillis = millis();
    if (currentMillis > lastMillis + 3)
    {
      break;
    }
  }
#endif
  Wire.write(memory_address);
  rc = Wire.endTransmission(false); // 结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  if (!(rc == 0 || rc == 7))
  {
    if (rc == I2C_ERROR_BUSY)
    {
      Wire.reset();
    }
    Task_Mesg.Give_Semaphore_IIC();
    return (rc);
  }

  Wire.requestFrom(_device_address, size, true); //地址，长度，停止位
  cnt = 0;
  while (Wire.available())
  {
    data[cnt] = Wire.read();
    cnt++;
  }
  Task_Mesg.Give_Semaphore_IIC();
  if (cnt == 0)
  {
    return (0xff);
  }

  return (0);
}
