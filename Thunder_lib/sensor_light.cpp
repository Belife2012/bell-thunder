#include <Arduino.h>
#include <Wire.h>
#include <sensor_light.h>

SENSOR_LIGHT::SENSOR_LIGHT(int slave_address):
  SENSOR_IIC(slave_address)
{}

byte SENSOR_LIGHT::Set_Led_Brightness(byte bright_level,unsigned char channel)
{
  byte ret;
  ret = write(LIGHT_IIC_REG_SETLIGHT, &bright_level, 1, channel);

  return ret;
}

void SENSOR_LIGHT::Set_Dark_Value(float new_value,unsigned char channel)
{
  CHECK_RANGE(new_value, 0, 100);
  dark_value[channel] = new_value;
}

void SENSOR_LIGHT::Set_Bright_Value(float new_value,unsigned char channel)
{
  CHECK_RANGE(new_value, 0, 100);
  bright_value[channel] = new_value;
}

byte SENSOR_LIGHT::Get_Light_Value_original(float *readValue, unsigned char channel)
{
  unsigned short retValue;
  byte getValue[2], bakCode;

  bakCode = read(LIGHT_IIC_REG_READLIGHT, getValue, 2, channel);
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

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 获取光电传感器相对数值（0~100）
 * 
 * @param channel: 传感器接口编号
 * @return float : 返回值
 */
float SENSOR_LIGHT::Get_Light_Value(unsigned char channel )
{
  float readValue;
  unsigned short retValue;
  byte getValue[2], bakCode;

  bakCode = read(LIGHT_IIC_REG_READLIGHT, getValue, 2, channel);
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

/**
 * @brief: 校准传感器的最大值或最小值
 * 
 * @param mode: 0 设置值为最大值，1 设置值为最小值, 2 重置设置值
 * @param value: 新设置的数值（0~100）
 * @param sensorChannel: 传感器接口编号
 */
void SENSOR_LIGHT::Set_Extremum(int mode, float value, uint8_t sensorChannel)
{
  if(mode == 0) {
    Set_Bright_Value(value, sensorChannel);
  } else if(mode == 1) {
    Set_Dark_Value(value, sensorChannel);
  } else if(mode == 2) {
    Set_Bright_Value(100, sensorChannel);
    Set_Dark_Value(0, sensorChannel);
  }
}

/**
 * @brief 设置光线检测的最大值和最小值
 * 
 * @param max_value 最大值
 * @param min_value 最小值
 * @param sensorChannel 
 */
void SENSOR_LIGHT::SetDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
  CHECK_RANGE(max_value, 0, 100);
  CHECK_RANGE(min_value, 0, 100);

  dark_value[sensorChannel] = min_value;
  bright_value[sensorChannel] = max_value;
}

/**
 * @brief: 复位光电传感器的设置
 * 
 * @param channel:传感器接口编号 
 */
void SENSOR_LIGHT::Reset(unsigned char channel )
{
  dark_value[channel] = 0;
  bright_value[channel] = 100;
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
byte SENSOR_LIGHT::Set_Operate_Mode(byte optMode,unsigned char channel)
{
  byte ret;
  CHECK_RANGE(optMode, 0, 100);
  ret = write(LIGHT_IIC_REG_SETLIGHT, &optMode, 1, channel);

  return ret;
}
