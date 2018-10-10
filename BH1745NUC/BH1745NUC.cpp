/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 颜色传感器模块库文件
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
 *  1.  BH1745NUC(int slave_address);                                   // 配置I2C地址
 *  2.  byte Setup(void);                                               // 初始化设置
 *  3.  byte Get_RGBC_Data(unsigned short *data);                       // 获取RGBC
 *  4.  void RGBtoHSV(unsigned short *RGBC, float *HSV);                // 计算HSV
 *  5.  uint8_t Colour_Recognition(unsigned short *RGBC, float *HSV);   // 识别颜色
 * 
 ************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <BH1745NUC.h>

// 配置I2C地址
BH1745NUC::BH1745NUC(int slave_address)
{
  _device_address = slave_address;
}

// 初始化设置
byte BH1745NUC::Setup(void)
{
  Serial.printf("开始配置颜色传感器...\n");

  byte rc;
  unsigned char reg;

  // 确认 Part ID
  rc = read(BH1745NUC_SYSTEM_CONTROL, &reg, sizeof(reg));
  if (rc != 0) 
  {
    Serial.printf(" SSS___ 颜色传感器未连接 ___SSS\n");
    return (rc);
  }
  reg = reg & 0x3F;
  Serial.print(F(" 颜色传感器 Part ID : "));
  Serial.println(reg, HEX);

  if (reg != BH1745NUC_PART_ID_VAL) 
  {
    Serial.printf(" Part ID 获取失败\n");
    return (rc);
  }

  // 确认 MANUFACTURER ID
  rc = read(BH1745NUC_MANUFACTURER_ID, &reg, sizeof(reg));
  if (rc != 0) 
  {
    Serial.printf(" SSS___ 颜色传感器未连接 ___SSS\n");
    return (rc);
  }
  Serial.print(F(" 颜色传感器 MANUFACTURER ID : "));
  Serial.println(reg, HEX);

  if (reg != BH1745NUC_MANUFACT_ID_VAL)
  {
    Serial.printf(" MANUFACTURER ID 获取失败\n");
    return (rc);
  }

  // 初始化配置
  reg = BH1745NUC_MODE_CONTROL1_VAL;
  rc = write(BH1745NUC_MODE_CONTROL1, &reg, sizeof(reg));
  if (rc != 0) 
  {
    Serial.println(F(" SSS___ 颜色传感器 MODE_CONTROL1 配置失败 ___SSS"));
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL2_VAL;
  rc = write(BH1745NUC_MODE_CONTROL2, &reg, sizeof(reg));
  if (rc != 0) 
  {
    Serial.println(F(" SSS___ 颜色传感器 MODE_CONTROL2 配置失败 ___SSS"));
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL3_VAL;
  rc = write(BH1745NUC_MODE_CONTROL3, &reg, sizeof(reg));
  if (rc != 0) 
  {
    Serial.println(F(" SSS___ 颜色传感器 MODE_CONTROL3 配置失败 ___SSS"));
    return (rc);
  }

  Serial.printf(" 颜色传感器配置完成\n");

  return 0;
}

// 获取RGBC，并将结果存入*data
byte BH1745NUC::Get_RGBC_Data(unsigned short *data)   
{
  byte rc;
  unsigned char val[8];

  rc = get_rawval(val);
  if (rc != 0) {
    return (rc);
  }

  data[0] = ((unsigned short)val[1] << 8) | val[0];
  data[1] = ((unsigned short)val[3] << 8) | val[2];
  data[2] = ((unsigned short)val[5] << 8) | val[4];
  data[3] = ((unsigned short)val[7] << 8) | val[6];

  return (rc);
}

// 利用*RGBC 计算HSV值，并存入*HSV
void BH1745NUC::RGBtoHSV(unsigned short *RGBC, float *HSV)
{
  float m_min, m_max, delta;

  m_min = min(RGBC[0], min(RGBC[1], RGBC[2]));
  m_max = max(RGBC[0], max(RGBC[1], RGBC[2]));
  delta = m_max - m_min;  
  if (delta == 0) delta = 1;  //被除的数

  float r = (float)RGBC[0];
  float g = (float)RGBC[1];
  float b = (float)RGBC[2];
  
  //求H
  if (r == m_max)
    HSV[0] = (g - b) / delta;     // between yellow & magenta
  else if (g == m_max)
    HSV[0] = 2 + (b - r) / delta; // between cyan & yellow
  else
    HSV[0] = 4 + (r - g) / delta; // between magenta & cyan
  HSV[0] *= 60;                   // degrees
  if (HSV[0] < 0)
    HSV[0] += 360;

  //求V
  HSV[2] = m_max;                 // v

  //求S
  if (m_max != 0)
    HSV[1] = delta / m_max;       // s
  else 
  {
    // r = g = b = 0        // s = 0, v is undefined
    HSV[1] = 0;
    HSV[0] = -1;
    return;
  }
}

// 识别颜色
uint8_t BH1745NUC::Colour_Recognition(unsigned short *RGBC, float *HSV)
{
  float m_min, m_max, delta;

  m_min = min(RGBC[0], min(RGBC[1], RGBC[2]));
  m_max = max(RGBC[0], max(RGBC[1], RGBC[2]));
  delta = m_max - m_min;  
  if (delta == 0) delta = 1;  //被除的数

  //先处理特殊的
  if(m_max <= 100)  //3个值都小于100 --> 没有东西反光
  {
    return 0xFF; //没有东西反光
  }
  else
  {
    if(m_min <= 800) //最小值小于800
    {
      if(delta <= 210)  //最小值小于800且最大值小于1010 --> 黑色
      {
        return 0xFE; //黑色
      }
    }
    else if(m_max < m_min * 1.2)  //均大于800并且差值不大
    {
      if(m_max <= 2100) //差值不大且数值都很小
      {
        return 0xFF; //没有东西
      }
      else
      {
        return 0x00;  //白色
      }
    }
  }

  //非特殊情况的颜色判别
  RGBtoHSV(RGBC,HSV);

  if((HSV[0] < 4) | (HSV[0] > 345))
  {
    return 0x09;  //粉红色
  }
  else if(HSV[0] < 18)
  {
    return 0x01;  //红色
  }
  else if(HSV[0] < 62)
  {
    return 0x02;  //橙色
  }
  else if(HSV[0] < 100)
  {
    return 0x03;  //黄色
  }
  else if(HSV[0] < 136)
  {
    return 0x04;  //绿色
  }
  else if(HSV[0] < 150)
  {
    return 0x05;  //青色
  }
  else if(HSV[0] < 192)
  {
    return 0x06;  //天蓝色
  }
  else if(HSV[0] < 213)
  {
    return 0x07;  //深蓝色
  }
  else if(HSV[0] < 345)
  {
    return 0x08;  //紫色
  }

  return 0;
}

// 类内部使用，读取传感器数据
byte BH1745NUC::get_rawval(unsigned char *data)
{
  byte rc;

  rc = read(BH1745NUC_RED_DATA_LSB, data, 8);

  return (rc);
}

// 类内部使用，I2C通讯，发送
byte BH1745NUC::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;

  Wire.beginTransmission(_device_address);
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission();
  return (rc);
}

// 类内部使用，I2C通讯，发送并读取
byte BH1745NUC::read(unsigned char memory_address, unsigned char *data, int size)
{
  byte rc;
  unsigned char cnt;

  Wire.beginTransmission(_device_address);
  Wire.write(memory_address);
  rc = Wire.endTransmission(false);
  if (rc != 0) 
  {
    return (rc);
  }

  Wire.requestFrom(_device_address, size, true);   // Wire.requestFrom(_device_address, size, true);

  cnt = 0;
  while(Wire.available()) 
  {
    data[cnt] = Wire.read();
    cnt++;
  }

  return (0);
}
