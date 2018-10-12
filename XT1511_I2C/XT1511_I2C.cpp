/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * I2C灯板库文件(彩灯驱动)
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
 *  1.  XT1511_I2C(uint8_t slave_address);    // 配置I2C地址
 *  2.  void Set_LED_Num(uint8_t number);     // 写入一共可以控制多少个灯
 *  3.  void Set_LED_Data(uint8_t address, uint8_t r, uint8_t g, uint8_t b);    // 写入单个寄存器数据
 *  4.  void Set_LEDs_Data(uint8_t address, uint8_t *data, uint8_t size);       // 写入多个寄存器数据
 *  5.  void LED_OFF(void);                   // 0xA0  全关，立即刷新
 *  6.  void LED_Updata(void);                // 0xA1  按照现有数据刷新
 *  7.  void LED_Show(uint8_t number);        // 0xA2  刷新预存数据 (指令预留)
 * 
 ************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <XT1511_I2C.h>

// 配置I2C地址
XT1511_I2C::XT1511_I2C(uint8_t slave_address)
{
  _device_address = slave_address;
}

// 写入一共可以控制多少个灯(预留)
// 参数 --> 需要控制灯的个数
void XT1511_I2C::Set_LED_Num(uint8_t number)
{
  byte rc;

  rc = write(XT1511_I2C_CONTROL_REG, &number, sizeof(number));
  if (rc != 0) 
  {
    Serial.println(F(" SSS___ 彩灯 XT1511_I2C_CONTROL 配置失败 ___SSS"));
  }
}

// 写入单个寄存器数据
// 参数1 --> 单个彩灯地址
// 参数2 --> 单个彩灯R值
// 参数3 --> 单个彩灯G值
// 参数4 --> 单个彩灯B值
void XT1511_I2C::Set_LED_Data(uint8_t address, uint8_t r, uint8_t g, uint8_t b)
{
  byte rc;
  uint8_t rgb[3];

  rgb[0] = r;
  rgb[1] = g;
  rgb[2] = b;

  rc = write(address, rgb, sizeof(rgb));
  if (rc != 0) 
  {
    Serial.println(F("SSS___ 写人单个彩灯数据失败 ___SSS"));
  }
}

// 写入多个寄存器数据
void XT1511_I2C::Set_LEDs_Data(uint8_t address, uint8_t *data, uint8_t size)
{
  byte rc;

  // rc = write(address, data, sizeof(data));
  rc = write(address, data, size);
  if (rc != 0) 
  {
    Serial.println(F("SSS___ 写人一组彩灯数据失败 ___SSS"));
  }
}

// 0xA0  全关，立即刷新
void XT1511_I2C::LED_OFF(void)
{
  Wire.beginTransmission(_device_address);    // 开启发送
  Wire.write(XT1511_I2C_COM_OFF);
  Wire.endTransmission();  //  结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
}

// 0xA1  按照现有数据刷新
void XT1511_I2C::LED_Updata(void)
{
  Wire.beginTransmission(_device_address);    // 开启发送
  Wire.write(XT1511_I2C_COM_UODATA);
  Wire.endTransmission();  //  结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
}

// 0xA2  刷新预存数据 (指令预留)
void XT1511_I2C::LED_Show(uint8_t number)
{
  byte rc;
  uint8_t reg;

  reg = number;

  rc = write(XT1511_I2C_COM_SHOW, &reg, sizeof(reg));    //COM全开
  if (rc != 0)
  {
    Serial.println(F("SSS___ 彩灯 XT1511_I2C_COM_SHOW 写入失败 ___SSS"));
  }
}

// 类内部使用，I2C通讯，发送
byte XT1511_I2C::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;

  Wire.beginTransmission(_device_address);    // 开启发送
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission();  //  结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  return (rc);
}

/* 
 * 设置彩灯的动态显示模式，有四种模式，
 *   当设置好模式后，编辑好的彩灯显示将一直以该模式显示直到自行改变
 * 
 * @parameters: 动态模式的参数，有四种 0：静态 1：闪烁 2：滚动 3：呼吸
 * @return: 
 */
void XT1511_I2C::Set_LED_Dynamic(uint8_t dynamicMode)
{

}

