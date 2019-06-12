#include <Arduino.h>
#include <Wire.h>
#include <BH1745NUC.h>
#include <Task_Mesg.h>

// 配置I2C地址
BH1745NUC::BH1745NUC(int slave_address):
  SENSOR_IIC(slave_address),
  env_backlight_c(NO_COLOR_CARD_C)
{
  device_detected = 0;
}

// 初始化设置
byte BH1745NUC::Setup(void)
{
  Serial.printf("\nstart Init Color Sensor...\n");

  byte rc;
  unsigned char reg;

  // 确认 Part ID
  rc = read(BH1745NUC_SYSTEM_CONTROL, &reg, sizeof(reg));
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
  rc = read(BH1745NUC_MANUFACTURER_ID, &reg, sizeof(reg));
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
  rc = write(BH1745NUC_MODE_CONTROL1, &reg, sizeof(reg));
  if (rc != 0)
  {
    Serial.println(F("# MODE_CONTROL1 fail #"));
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL2_VAL;
  rc = write(BH1745NUC_MODE_CONTROL2, &reg, sizeof(reg));
  if (rc != 0)
  {
    Serial.println(F("# MODE_CONTROL2 fail #"));
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL3_VAL;
  rc = write(BH1745NUC_MODE_CONTROL3, &reg, sizeof(reg));
  if (rc != 0)
  {
    Serial.println(F("# MODE_CONTROL3 fail #"));
    return (rc);
  }

  Serial.printf("Color Sensor Init end\n\n");
  device_detected = 1;

  return 0;
}

void BH1745NUC::Env_Backlight_Filter(unsigned short new_data)
{
  env_backlight_c += ( (float)(new_data - env_backlight_c) ) / 30;
}

// 获取RGBC，并将结果存入*data
byte BH1745NUC::Get_RGBC_Data(unsigned short *data)
{
  byte rc;
  unsigned char val[8];

  rc = get_rawval(val);
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

// 利用*RGBC 计算HSV值，并存入*HSV
/* 
 * 如果用于动态的数据上，需要对HSV的结果值进行滤波
 * 
 * @parameters: *RGBC RGBC数据的地址；*HSV是计算结果的地址 
 * @return: 
 */
void BH1745NUC::RGBtoHSV(unsigned short *RGBC, float *HSV)
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
uint8_t BH1745NUC::Colour_Recognition(unsigned short *RGBC)
{
  float m_min, m_max, HSV[3];
  short compensation_c_high = 0, compensation_c_low = 0;

  m_min = min(RGBC[0], min(RGBC[1], RGBC[2]));
  m_max = max(RGBC[0], max(RGBC[1], RGBC[2]));
  
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

  //非特殊情况的颜色判别, 依据 颜色 H值
  RGBtoHSV(RGBC, HSV);
  
  //先处理特殊的, 无色卡情况
  // if (RGBC[3] < NO_COLOR_CARD_C) //C值小于NO_COLOR_CARD_C ，没有东西反光
  // {
  //   return NO_CARD; //没有东西反光
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
        return BLACK_CARD; //黑色
      }
      else if (m_min >= WHITE_CARD_MIN_RGB && 
          ( WHITE_CARD_MIN_C < RGBC[3]) && (RGBC[3] < WHITE_CARD_MAX_C ) 
          // && (WHITE_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < WHITE_MAX_H) 
          ) 
      {
        return WHITE_CARD; //白色
      }
    }else{
      if ( (RED_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < RED_CARD_MAX_H)
            //  &&  ( RED_CARD_MIN_C < RGBC[3]) && (RGBC[3] < RED_CARD_MAX_C )
             )
      {
        return RED_CARD; // 
      }
      else if ( (BROWN_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < BROWN_CARD_MAX_H)
            //  &&  ( BROWN_CARD_MIN_C < RGBC[3]) && (RGBC[3] < BROWN_CARD_MAX_C )
             )
      {
        return BROWN_CARD; //
      }
      // 黄色与棕色的交叉区域使用 S 值进行区分
      else if ( (BROWN_CARD_MAX_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < YELLOW_CARD_MIN_H)
             )
      {
        if( HSV[1] < 0.85 ){
          return BROWN_CARD; //
        }
        else{
          return YELLOW_CARD; //
        }
      }
      else if ( (YELLOW_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < YELLOW_CARD_MAX_H)
            //  &&  ( YELLOW_CARD_MIN_C < RGBC[3]) && (RGBC[3] < YELLOW_CARD_MAX_C )
             )
      {
        return YELLOW_CARD; //
      }
      else if ( (GREEN_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < GREEN_CARD_MAX_H)
            //  &&  ( GREEN_CARD_MIN_C < RGBC[3]) && (RGBC[3] < GREEN_CARD_MAX_C )
             )
      {
        return GREEN_CARD; //
      }
      else if ( (BLUE_CARD_MIN_H < (uint32_t)HSV[0]) && ((uint32_t)HSV[0] < BLUE_CARD_MAX_H)
            //  &&  ( BLUE_CARD_MIN_C < RGBC[3]) && (RGBC[3] < BLUE_CARD_MAX_C )
             )
      {
        return BLUE_CARD; //
      }
    }
    
  }
  Env_Backlight_Filter(RGBC[3]);
  return NO_CARD;
}

// 类内部使用，读取传感器数据
byte BH1745NUC::get_rawval(unsigned char *data)
{
  byte rc;

  rc = read(BH1745NUC_RED_DATA_LSB, data, 8);

  // 如果读取数值失败，则标志设备未检测到，否则查看是否需要重新初始化设备
  if(rc != 0){
    device_detected = 0;
  }else if(device_detected == 0){
    Setup();
    // 初始化完成后，不会重新读取，下一次的读取数值才是有效的
  }

  return (rc);
}

uint8_t BH1745NUC::Thunder_Get_Color_Sensor_Data(uint8_t sensorChannel)
{    
  uint8_t Colour_Num = 0;
  unsigned short RGBC[4] = {
      0
  };
  SENSOR_IIC::Select_Sensor_Channel(sensorChannel);
  Get_RGBC_Data(RGBC);
  switch (Colour_Recognition(RGBC)) {
      case BLACK_CARD:
          return Colour_Num = 7; //黑色
          break;
      case WHITE_CARD:
          return Colour_Num = 6; //白色
          break;
      case RED_CARD:
          return Colour_Num = 1; //红色
          break;
      case BROWN_CARD:
          return Colour_Num = 5; //棕色
          break;
      case YELLOW_CARD:
          return Colour_Num = 2; //黄色
          break;
      case GREEN_CARD:
          return Colour_Num = 3; //绿色
          break;
      case BLUE_CARD:
          return Colour_Num = 4; //蓝色
          break;
      case NO_CARD:
          return Colour_Num = 0; //无颜色
          break;
      default:
          return Colour_Num = 0; //bad return
          break;
  }
}