/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 单色点阵灯库文件
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
 *  1.  HT16D35B(int slave_address);                    // 配置I2C地址
 *  2.  byte Setup(void);                               // 初始化设置
 *  3.  byte LED_Show(unsigned char *data, int size);   // 显示图案，参数*data为灯数据，参数size为数据长度
 * 
 ************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <HT16D35B.h>
#include <Thunder_Display.h>
#include <Task_Mesg.h>

// #define PRINT_DEBUG_ERROR

// 配置I2C地址
HT16D35B::HT16D35B(int slave_address)
{
  _device_address = slave_address;
}

// 初始化设置
byte HT16D35B::Setup(void)
{
  Serial.printf("\nstart Init LED Matrix...\n");

  byte rc;
  uint8_t ROW_BUFF[4] = {0x7F, 0XFF, 0XFF, 0XFF}; //最后一个ROW不用
  uint8_t LED_BUFF[29] =
      {
          0x00, //地址
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00 //不用的
      };

  unsigned char reg;

  reg = 0xff;
  rc = write(HT16D35B_CONTROL_COM, &reg, sizeof(reg)); //COM全开
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_CONTROL_COM fail #"));
    return (rc);
  }

  rc = write(HT16D35B_CONTROL_ROW, ROW_BUFF, sizeof(ROW_BUFF)); // 最后一个ROW不用
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_CONTROL_ROW fail #"));
    return (rc);
  }

  reg = 0x01;
  rc = write(HT16D35B_MODE_GRAY, &reg, sizeof(reg)); // 选择模式：灰度模式: 0x00; 二进制模式0x01
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_MODE_GRAY fail #"));
    return (rc);
  }

  reg = 20;
  rc = write(HT16D35B_GLOBAL_BRIGHT, &reg, sizeof(reg)); // 全局亮度设置：该命令用于控制 64 级 PWM 亮度
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_GLOBAL_BRIGHT fail #"));
    return (rc);
  }

  rc = write(HT16D35B_DISPLAY_RAM, LED_BUFF, sizeof(LED_BUFF)); //写入数据
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_DISPLAY_RAM fail #"));
    return (rc);
  }

  reg = 0X03;
  rc = write(HT16D35B_SYSTEM_CONTROL, &reg, sizeof(reg));
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_SYSTEM_CONTROL fail #"));
    return (rc);
  }

  Serial.printf("Init LED Matrix end\n\n");
  return 0;
}

// 显示图案
// 参数*data --> 灯数据
// 参数size --> 数据长度
byte HT16D35B::LED_Show(const unsigned char *data, int size)
{
  byte rc;

  Task_Mesg.Take_Semaphore_IIC();
  Wire.beginTransmission(_device_address); // 开启发送
  Wire.write(HT16D35B_DISPLAY_RAM);
  Wire.write(data, size);
  rc = Wire.endTransmission(); //  结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  #ifdef COMPATIBILITY_OLD_ESP_LIB
  if (rc == I2C_ERROR_BUSY)
  {
    Wire.reset();
  }
  #endif
  Task_Mesg.Give_Semaphore_IIC();

  if (rc != 0)
  {
  #ifdef PRINT_DEBUG_ERROR
    Serial.printf("### LED IIC Error! ###%d#### \n", rc);
  #endif
  }
  return (rc);
}

// 类内部使用，I2C通讯，发送
byte HT16D35B::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;
  Task_Mesg.Take_Semaphore_IIC();
  Wire.beginTransmission(_device_address); // 开启发送
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission(); //  结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  #ifdef COMPATIBILITY_OLD_ESP_LIB
  if (rc == I2C_ERROR_BUSY)
  {
    Wire.reset();
  }
  #endif
  Task_Mesg.Give_Semaphore_IIC();
  return (rc);
}

byte HT16D35B::read(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;
  unsigned char cnt;

  Task_Mesg.Take_Semaphore_IIC();
  Wire.beginTransmission(_device_address);
  Wire.write(memory_address);
  rc = Wire.endTransmission(false);
  if (!(rc == 0 || rc == 7))
  {
    #ifdef COMPATIBILITY_OLD_ESP_LIB
    if (rc == I2C_ERROR_BUSY)
    {
      Wire.reset();
    }
    #endif
    Task_Mesg.Give_Semaphore_IIC();
    return (rc);
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