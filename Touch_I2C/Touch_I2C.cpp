#include <Arduino.h>
#include <Wire.h>
#include <Touch_I2C.h>
#include <Task_Mesg.h>

/* 
 * 构造函数
 * 
 * @parameters: 触碰模块的 IIC 地址
 * @return: 
 */
TOUCH_I2C::TOUCH_I2C(int slave_address)
{
  _device_address = slave_address;
}

/* 
 * 获取触碰状态，0为未按下状态，1为按下状态，其他的过程值 可以通过连续获取状态做判断触碰过程
 * 
 * @parameters: 
 *      0 未按下状态
 *      1 按下状态
 * @return: 
 *      0 获取数据正常
 *      非0 获取数据出错
 */
byte TOUCH_I2C::Get_Status(byte *readValue)
{
  unsigned char ret;

  ret = read(TOUCH_ADDRESS_VALUE, readValue, 1);

  return ret;
}

/* 
 * 复位工作模式：自动模式，未按下亮绿灯，按下亮红灯
 * 
 * @parameters: 
 * @return: 
 */
byte TOUCH_I2C::Reset_Mode(void)
{
  unsigned char ctl_data, ret;

  ctl_data = 0;

  ret = write(TOUCH_ADDRESS_CONTROL, &ctl_data, 1);

  return ret;
}

/* 
 * 检查触碰按键的相关按键事件是否有发生
 * 
 * @parameters: 
 * @return: true是有发生过，false是未发生过
 */
bool TOUCH_I2C::Check_Touch_Event(enum_touch_event check_event)
{
  byte ret;
  byte readValue = 0;

  if(check_event == TOUCH_EVENT_RELEASE){
    ret = read(TOUCH_ADDRESS_RELEASE, &readValue, 1);
  }else if(check_event == TOUCH_EVENT_PRESS){
    ret = read(TOUCH_ADDRESS_PRESS, &readValue, 1);
  }else if(check_event == TOUCH_EVENT_TOUCH){
    ret = read(TOUCH_ADDRESS_TOUCH, &readValue, 1);
  }

  if(ret != 0){
    return false;
  }else{
    return readValue;
  }
}

/* 
 * 设置触碰模块的LED灯颜色，范围 0-255
 * 
 * @parameters: 全部传入0 值时，即为关闭LED灯
 *      RedValue LED亮度的红色分量，范围 0-255
 *      GreenValue LED亮度的绿色分量，范围 0-255
 *      BlueValue LED亮度的蓝色分量，范围 0-255
 * @return: 
 *      0 写数据正常
 *      非0 写数据出错
 */
byte TOUCH_I2C::Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue)
{
  unsigned char rgb[3], ret;

  rgb[0] = RedValue;
  rgb[1] = GreenValue;
  rgb[2] = BlueValue;

  ret = write(TOUCH_ADDRESS_RGB, rgb, 3);

  return ret;
}

// 类内部使用，I2C通讯，发送；返回 0 表示成功完成，非零表示没有成功
byte TOUCH_I2C::write(unsigned char memory_address, unsigned char *data, unsigned char size)
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
byte TOUCH_I2C::read(unsigned char memory_address, unsigned char *data, unsigned char size)
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
    if(rc == I2C_ERROR_BUSY){
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
