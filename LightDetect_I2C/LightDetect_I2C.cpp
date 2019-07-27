#include <Arduino.h>
#include <Wire.h>
#include <LightDetect_I2C.h>

LIGHTDETECT_I2C::LIGHTDETECT_I2C(int slave_address):
  SENSOR_IIC(slave_address)
{}

byte LIGHTDETECT_I2C::Set_Led_Brightness(byte bright_level,unsigned char channel)
{
  byte ret;
  ret = write(0x01, &bright_level, 1, channel);

  return ret;
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
byte LIGHTDETECT_I2C::Set_Operate_Mode(byte optMode,unsigned char channel)
{
  byte ret;
  ret = write(0x01, &optMode, 1, channel);

  return ret;
}

void LIGHTDETECT_I2C::Set_Dark_Value(float new_value,unsigned char channel)
{
  dark_value[channel] = new_value;
}

void LIGHTDETECT_I2C::Set_Bright_Value(float new_value,unsigned char channel)
{
  bright_value[channel] = new_value;
}

void LIGHTDETECT_I2C::Reset_All_Value(unsigned char channel )
{
  dark_value[channel] = 0;
  bright_value[channel] = 100;
}
/* 
 * 获取光电传感器的值
 * 
 * @parameters: 获取光亮程度的百分比数值
 * @return: 
 *      0 获取数据正常
 *      非0 获取数据出错
 */
float LIGHTDETECT_I2C::Get_Light_Value(unsigned char channel )
{
  float readValue;
  unsigned short retValue;
  byte getValue[2], bakCode;

  bakCode = read(0x02, getValue, 2, channel);
  if(bakCode != 0){
    return 0;
  }

  retValue = (unsigned short)getValue[0] + ((unsigned short)getValue[1] << 8);
  // 百分比 = 100 * ADC_value/2400, 设置2400为最大值
  retValue = retValue > 2400 ? 2400 : retValue ;
  readValue = ( (float)retValue ) / 24;

  // Serial.printf("dark:%f, bright:%f \n", dark_value[channel], bright_value[channel]);

  //计算 在区间 dark_value[channel] ~ bright_value[channel] 的位置百分比
  if(readValue <= dark_value[channel]){
    readValue = 0;
  }else if(readValue >= bright_value[channel]){
    readValue = 100;
  }else{
    readValue = (readValue - dark_value[channel])*100 / (bright_value[channel] - dark_value[channel]);
  }

  return readValue;
}
byte LIGHTDETECT_I2C::Get_Light_Value_original(float *readValue, unsigned char channel)
{
  unsigned short retValue;
  byte getValue[2], bakCode;

  bakCode = read(0x02, getValue, 2, channel);
  if(bakCode != 0){
    *readValue = 0;
    return bakCode;
  }

  retValue = (unsigned short)getValue[0] + ((unsigned short)getValue[1] << 8);
  // 百分比 = 100 * ADC_value/2400, 设置2400为最大值
  retValue = retValue > 2400 ? 2400 : retValue ;
  *readValue = ( (float)retValue ) / 24;

  return 0;
}

int LIGHTDETECT_I2C::Thunder_Get_Light_Data(uint8_t sensorChannel)
{
  float lightValue;
  lightValue = Get_Light_Value(sensorChannel);
  return lightValue;
}

void LIGHTDETECT_I2C::Thunder_Set_Light_Mode(uint8_t sensorChannel, byte mode)
{
  Set_Operate_Mode(mode, sensorChannel);
}

void LIGHTDETECT_I2C::Thunder_Set_Light_Extremum(uint8_t sensorChannel, int mode, float value)
{
  if(mode == 0) {
    Set_Bright_Value(sensorChannel, value);
  } else if(mode == 1) {
    Set_Dark_Value(sensorChannel, value);
  }
}
void LIGHTDETECT_I2C::Thunder_Set_Light_Reset(uint8_t sensorChannel)
{
  Reset_All_Value(sensorChannel);
}
