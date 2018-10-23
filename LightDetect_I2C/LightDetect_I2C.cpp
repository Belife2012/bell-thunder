#include <Arduino.h>
#include <Wire.h>
#include <LightDetect_I2C.h>

LIGHTDETECT_I2C::LIGHTDETECT_I2C(int slave_address)
{
  _device_address = slave_address;
}

/* 
 * 设置光电传感器的模式
 * 
 * @parameters: 
 *      0 为环境光模式，不点亮灯
 *      1 为反射光模式，需要点亮灯
 * @return: 
 */
byte LIGHTDETECT_I2C::Set_Operate_Mode(byte optMode)
{
  byte ret;
  ret = write(0x01, &optMode, 1);

  return ret;
}

/* 
 * 获取光电传感器的值
 * 
 * @parameters: 返回未处理过的亮度值
 * @return: 
 */
byte LIGHTDETECT_I2C::Get_Light_Value(unsigned short* readValue)
{
    unsigned short retValue;
    byte getValue[2], bakCode;

    bakCode = read(0x02, getValue, 2);

    *readValue = ((unsigned short)getValue[0] + ((unsigned short)getValue[1] << 8));

    return bakCode;
}

// 类内部使用，I2C通讯，发送；返回 0 表示成功完成，非零表示没有成功
byte LIGHTDETECT_I2C::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;

  Wire.beginTransmission(_device_address);
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission();
  return (rc);
}

// 类内部使用，I2C通讯，发送并读取；返回值 非0 表示失败，其中0xFF表示没有读取数量有误
byte LIGHTDETECT_I2C::read(unsigned char memory_address, unsigned char *data, int size)
{
  byte rc;
  unsigned char cnt = 0;

  Wire.beginTransmission(_device_address); // 开启发送
  Wire.write(memory_address);
  rc = Wire.endTransmission(false);        // 结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  if (rc != 0) 
  {
      return rc;
  }

  Wire.requestFrom(_device_address, size, true);  //地址，长度，停止位
  cnt = 0;
  while(Wire.available()) 
  {
    data[cnt] = Wire.read();
    cnt++;
  }

  if(cnt == size)
    return 0;
  else 
    return 0xFF;
}
