#include <Arduino.h>
#include <Wire.h>
#include <sensor_color.h>

// 配置I2C地址
SENSOR_COLOR::SENSOR_COLOR(int slave_address):
  SENSOR_IIC(slave_address),
  env_backlight_c(NO_COLOR_CARD_C)
{}

// 初始化设置
byte SENSOR_COLOR::Setup(unsigned char channel)
{
  CHECK_RANGE(channel, 1, 6);
  Serial.printf("\nstart Init Color Sensor...\n");

  byte rc;
  unsigned char reg;

  // 确认 Part ID
  rc = read(BH1745NUC_SYSTEM_CONTROL, &reg, sizeof(reg), channel);
  if (rc != 0)
  {
    Serial.printf("# No Color Sensor\n");
    return (rc);
  }
  reg = reg & 0x3F;
  Serial.print(F("* Part ID : "));
  Serial.println(reg, HEX);

  if (reg != BH1745NUC_PART_ID_VAL)
  {
    Serial.printf("# Part ID fail\n");
    return (rc);
  }

  // 确认 MANUFACTURER ID
  rc = read(BH1745NUC_MANUFACTURER_ID, &reg, sizeof(reg), channel);
  if (rc != 0)
  {
    Serial.printf("# No Color Sensor\n");
    return (rc);
  }
  Serial.print(F("* Color Sensor MANUFACTURER ID : "));
  Serial.println(reg, HEX);

  if (reg != BH1745NUC_MANUFACT_ID_VAL)
  {
    Serial.printf("# MANUFACTURER ID fail\n");
    return (rc);
  }

  // 初始化配置
  reg = BH1745NUC_MODE_CONTROL1_VAL;
  rc = write(BH1745NUC_MODE_CONTROL1, &reg, sizeof(reg), channel);
  if (rc != 0)
  {
    Serial.println(F("# MODE_CONTROL1 fail #"));
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL2_VAL;
  rc = write(BH1745NUC_MODE_CONTROL2, &reg, sizeof(reg), channel);
  if (rc != 0)
  {
    Serial.println(F("# MODE_CONTROL2 fail #"));
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL3_VAL;
  rc = write(BH1745NUC_MODE_CONTROL3, &reg, sizeof(reg), channel);
  if (rc != 0)
  {
    Serial.println(F("# MODE_CONTROL3 fail #"));
    return (rc);
  }

  Serial.printf("Color Sensor Init end\n\n");
  device_detected[channel - 1] = 1;

  return 0;
}

void SENSOR_COLOR::Env_Backlight_Filter(unsigned short new_data)
{
  env_backlight_c += ( (float)(new_data - env_backlight_c) ) / 30;
}

// 利用*RGBC 计算HSV值，并存入*HSV
/* 
 * 如果用于动态的数据上，需要对HSV的结果值进行滤波
 * 
 * @parameters: *RGBC RGBC数据的地址；*HSV是计算结果的地址 
 * @return: 
 */
void SENSOR_COLOR::RGBtoHSV(unsigned short *RGBC, float *HSV)
{
  float m_min, m_max, delta;

  m_min = min(RGBC[0], min(RGBC[1], RGBC[2]));
  m_max = max(RGBC[0], max(RGBC[1], RGBC[2]));
  delta = m_max - m_min;
  if (delta == 0)
    delta = 1; //被除的数

  float r = (float)RGBC[0];
  float g = (float)RGBC[1];
  float b = (float)RGBC[2];

  //求H
  if (r == m_max)
    HSV[0] = (g - b) / delta; // between yellow & magenta
  else if (g == m_max)
    HSV[0] = 2 + (b - r) / delta; // between cyan & yellow
  else
    HSV[0] = 4 + (r - g) / delta; // between magenta & cyan
  HSV[0] *= 60;                   // degrees
  if (HSV[0] < 0)
    HSV[0] += 360;

  //求V
  HSV[2] = m_max; // v

  //求S
  if (m_max != 0)
    HSV[1] = delta / m_max; // s
  else
  {
    // r = g = b = 0        // s = 0, v is undefined
    HSV[1] = 0;
    HSV[0] = -1;
    return;
  }
}

// 识别颜色
/* 
 * 根据C进行判断亮度，所有色卡靠近都有反射光，C值小于一定的值 NO_COLOR_CARD_C 为无卡状态
 * 
 * @parameters: 
 * @return: enum_Color_Card
 */
uint8_t SENSOR_COLOR::Colour_Recognition(unsigned short *RGBC)
{
  float m_min, m_max, HSV[3];

  m_min = min(RGBC[0], min(RGBC[1], RGBC[2]));
  m_max = max(RGBC[0], max(RGBC[1], RGBC[2]));
  
  #if 0
  short compensation_c_high = 0, compensation_c_low = 0;
  // 根据环境光 计算黑卡的 C值补偿
  if(env_backlight_c > NO_COLOR_CARD_C){
    // compensation_c_high = ( (unsigned short)env_backlight_c - NO_COLOR_CARD_C ) * 2;
    // compensation_c_high = (compensation_c_high > BLACK_CARD_MAX_C) ? BLACK_CARD_MAX_C : compensation_c_high;
    compensation_c_high = 30;
  }
  if( BLACK_CARD_MIN_C < env_backlight_c && env_backlight_c < BLACK_CARD_MAX_C ){
    compensation_c_low = ( BLACK_CARD_MAX_C - BLACK_CARD_MIN_C ) / 2;
    if( env_backlight_c < compensation_c_low + BLACK_CARD_MIN_C ){
      compensation_c_low = (env_backlight_c - BLACK_CARD_MIN_C) + 2;
    }else{
      compensation_c_low = 0;
      compensation_c_high = (env_backlight_c - BLACK_CARD_MAX_C) - 2;
    }
  }
  #endif

  //非特殊情况的颜色判别, 依据 颜色 H值
  RGBtoHSV(RGBC, HSV);
  
  //先处理特殊的, 无色卡情况
  // if (RGBC[3] < NO_COLOR_CARD_C) //C值小于NO_COLOR_CARD_C ，没有东西反光
  // {
  //   return CARD_NO; //没有东西反光
  // }
  // else
  {
    // 优先判断黑白，因为颜色有 H值 作为比较标准的判断条件, 而黑白具有较低的 S值
    if( HSV[1] < COLORLESS_S ){
      if (m_max <= BLACK_CARD_MAX_RGB && 
          // ( BLACK_CARD_MIN_C + compensation_c_low < RGBC[3]) && (RGBC[3] < BLACK_CARD_MAX_C + compensation_c_high ) && 
          ( BLACK_CARD_MIN_C < RGBC[3]) && (RGBC[3] < BLACK_CARD_MAX_C )
          // && (BLACK_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < BLACK_MAX_H) 
           ) 
      {
        Env_Backlight_Filter(RGBC[3]);
        return CARD_BLACK; //黑色
      }
      else if (m_min >= WHITE_CARD_MIN_RGB && 
          ( WHITE_CARD_MIN_C < RGBC[3]) && (RGBC[3] < WHITE_CARD_MAX_C ) 
          // && (WHITE_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < WHITE_MAX_H) 
          ) 
      {
        return CARD_WHITE; //白色
      }
    }else{
      if ( (RED_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < RED_CARD_MAX_H)
            //  &&  ( RED_CARD_MIN_C < RGBC[3]) && (RGBC[3] < RED_CARD_MAX_C )
             )
      {
        return CARD_RED; // 
      }
      else if ( (BROWN_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < BROWN_CARD_MAX_H)
            //  &&  ( BROWN_CARD_MIN_C < RGBC[3]) && (RGBC[3] < BROWN_CARD_MAX_C )
             )
      {
        return CARD_BROWN; //
      }
      // 黄色与棕色的交叉区域使用 S 值进行区分
      else if ( (BROWN_CARD_MAX_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < YELLOW_CARD_MIN_H)
             )
      {
        if( HSV[1] < 0.85 ){
          return CARD_BROWN; //
        }
        else{
          return CARD_YELLOW; //
        }
      }
      else if ( (YELLOW_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < YELLOW_CARD_MAX_H)
            //  &&  ( YELLOW_CARD_MIN_C < RGBC[3]) && (RGBC[3] < YELLOW_CARD_MAX_C )
             )
      {
        return CARD_YELLOW; //
      }
      else if ( (GREEN_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < GREEN_CARD_MAX_H)
            //  &&  ( GREEN_CARD_MIN_C < RGBC[3]) && (RGBC[3] < GREEN_CARD_MAX_C )
             )
      {
        return CARD_GREEN; //
      }
      else if ( (BLUE_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < BLUE_CARD_MAX_H)
            //  &&  ( BLUE_CARD_MIN_C < RGBC[3]) && (RGBC[3] < BLUE_CARD_MAX_C )
             )
      {
        return CARD_BLUE; //
      }
    }
    
  }
  Env_Backlight_Filter(RGBC[3]);
  return CARD_NO;
}

// 类内部使用，读取传感器数据
byte SENSOR_COLOR::get_rawval(unsigned char *data, unsigned char channel)
{
  byte rc;
  CHECK_RANGE(channel, 1, 6);
  rc = read(BH1745NUC_RED_DATA_LSB, data, 8, channel);

  // 如果读取数值失败，则标志设备未检测到，否则查看是否需要重新初始化设备
  if(rc != 0){
    device_detected[channel - 1] = 0;
  }else if(device_detected[channel - 1] == 0){
    Setup(channel);
    // 初始化完成后，不会重新读取，下一次的读取数值才是有效的
  }

  return (rc);
}

// 获取RGBC，并将结果存入*data
byte SENSOR_COLOR::Get_RGBC_Data(unsigned short *data, unsigned char channel)
{
  byte rc;
  unsigned char val[8];

  rc = get_rawval(val, channel);
  if (rc != 0)
  {
    return (rc);
  }

  data[0] = ((unsigned short)val[1] << 8) | val[0];
  data[1] = ((unsigned short)val[3] << 8) | val[2];
  data[2] = ((unsigned short)val[5] << 8) | val[4];
  data[3] = ((unsigned short)val[7] << 8) | val[6];

  return (rc);
}

/**
 * @brief: 获取颜色（红色）分量
 * 
 * @param channel: 传感器接口编号
 * @return uint16_t : 颜色分量
 */
uint16_t SENSOR_COLOR::Get_Red(uint8_t channel) 
{
  uint16_t data[4];

  Get_RGBC_Data(data, channel);
  return data[0];
}
/**
 * @brief: 获取颜色（绿色）分量
 * 
 * @param channel: 传感器接口编号
 * @return uint16_t : 颜色分量
 */
uint16_t SENSOR_COLOR::Get_Green(uint8_t channel) 
{
  uint16_t data[4];

  Get_RGBC_Data(data, channel);
  return data[1];
}
/**
 * @brief: 获取颜色（蓝牙）分量
 * 
 * @param channel: 传感器接口编号
 * @return uint16_t : 颜色分量
 */
uint16_t SENSOR_COLOR::Get_Blue(uint8_t channel) 
{
  uint16_t data[4];

  Get_RGBC_Data(data, channel);
  return data[2];
}
/**
 * @brief: 获取颜色（明亮）分量
 * 
 * @param channel: 传感器接口编号
 * @return uint16_t : 颜色分量
 */
uint16_t SENSOR_COLOR::Get_Clear(uint8_t channel) 
{
  uint16_t data[4];

  Get_RGBC_Data(data, channel);
  return data[3];
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 获取颜色传感器检测到的色卡颜色
 * 
 * @param sensorChannel:传感器接口编号
 * @return uint8_t :色卡颜色编号
 */
uint8_t SENSOR_COLOR::Get_Color_Result(uint8_t sensorChannel)
{    
  unsigned short RGBC[4] = {
      0
  };
  Get_RGBC_Data(RGBC, sensorChannel);
  return Colour_Recognition(RGBC);
}