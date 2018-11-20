#include <Arduino.h>
#include <Wire.h>
#include <LightDetect_I2C.h>
#include <Task_Mesg.h>

LIGHTDETECT_I2C::LIGHTDETECT_I2C(int slave_address)
{
  _device_address = (byte)slave_address;
}

/* 
 * 设置光电传感器的模式
 * 
 * @parameters: 
 *      LED亮灯可设置等级为 0 ~ 100，可以设置对应的LED亮度等级，用于不同模式
 * @return: 
 *      0 写数据正常
 *      非0 写数据出错
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
 * @parameters: 获取光亮程度的百分比数值
 * @return: 
 *      0 获取数据正常
 *      非0 获取数据出错
 */
byte LIGHTDETECT_I2C::Get_Light_Value(float *readValue)
{
  unsigned short retValue;
  byte getValue[2], bakCode;

  bakCode = read(0x02, getValue, 2);

  retValue = (unsigned short)getValue[0] + ((unsigned short)getValue[1] << 8);
  // 百分比 = 100 * ADC_value/2400, 设置2400为最大值
  retValue = retValue > 2400 ? 2400 : retValue ;
  *readValue = ( (float)retValue ) / 24;

  return bakCode;
}

// 类内部使用，I2C通讯，发送；返回 0 表示成功完成，非零表示没有成功
byte LIGHTDETECT_I2C::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;

  Task_Mesg.Take_Semaphore_IIC();
  Wire.beginTransmission(_device_address);
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission();
  #ifdef COMPATIBILITY_OLD_ESP_LIB
  if (rc == I2C_ERROR_BUSY)
  {
    Wire.reset();
  }
  #endif
  Task_Mesg.Give_Semaphore_IIC();
  return (rc);
}

// 类内部使用，I2C通讯，发送并读取；返回值 非0 表示失败，其中0xFF表示没有读取数量有误
byte LIGHTDETECT_I2C::read(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;
  unsigned char cnt = 0;

  Task_Mesg.Take_Semaphore_IIC();
  Wire.beginTransmission(_device_address); // 开启发送
  Wire.write(memory_address);
  rc = Wire.endTransmission(false); // 结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  if (!(rc == 0 || rc == 7))
  {
    #ifdef COMPATIBILITY_OLD_ESP_LIB
    if (rc == I2C_ERROR_BUSY){
      Wire.reset();
    }
    #endif
    Task_Mesg.Give_Semaphore_IIC();
    return rc;
  }

  cnt = 0;
  if( 0 != Wire.requestFrom(_device_address, size, (byte)true) ){
    while (Wire.available())
    {
      data[cnt] = Wire.read();
      cnt++;
    }
  }
  Task_Mesg.Give_Semaphore_IIC();

  return (cnt != 0) ? 0 : 0xff;
}
