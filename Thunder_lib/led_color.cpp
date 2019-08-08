#include <Arduino.h>
#include <Wire.h>
#include <led_color.h>

// 配置I2C地址
LED_COLOR::LED_COLOR(uint8_t slave_address) : 
  SENSOR_IIC(slave_address),
  LEDs_Data({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}),
  LED_Dynamic(0),
  ledDynamicIndex(0)
{}

// 写入一共可以控制多少个灯(预留)
// 参数 --> 需要控制灯的个数
void LED_COLOR::Set_LED_Num(uint8_t number)
{
  byte rc;

  rc = write(XT1511_I2C_CONTROL_REG, &number, sizeof(number));
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.println("# XT1511_I2C_CONTROL fail");
    #endif
  }
}

// 写入单个寄存器数据
// 参数1 --> 单个彩灯地址
// 参数2 --> 单个彩灯R值
// 参数3 --> 单个彩灯G值
// 参数4 --> 单个彩灯B值
void LED_COLOR::Set_LED_Data(uint8_t address, uint8_t r, uint8_t g, uint8_t b)
{
  byte rc;
  uint8_t rgb[3];

  rgb[0] = r;
  rgb[1] = g;
  rgb[2] = b;

  if (address > sizeof(LEDs_Data) / 3)
    return;

  rc = write(address, rgb, sizeof(rgb));
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.println("# LED_COLOR Set LED fail");
    #endif
  }

  byte startIndex;
  startIndex = (address - 1) * 3;
  LEDs_Data[startIndex] = rgb[0];
  LEDs_Data[startIndex + 1] = rgb[1];
  LEDs_Data[startIndex + 2] = rgb[2];
}

// 写入多个寄存器数据
void LED_COLOR::Set_LEDs_Data(uint8_t address, uint8_t *data, uint8_t size)
{
  byte rc;
  byte startIndex;
  startIndex = (address - 1) * 3;

  if (startIndex + size > sizeof(LEDs_Data))
    return;

  Set_LED_Dynamic(0);

  for (byte i = 0; i < size; i++)
  {
    LEDs_Data[startIndex + i] = data[i] * 1 / 10;
  }

  // rc = write(address, data, sizeof(data));
  rc = write(address, &(LEDs_Data[startIndex]), size);
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.println("# LED_COLOR Set LEDs fail");
    #endif
  }
}

// 0xA0  全关，立即刷新
void LED_COLOR::LED_OFF(void)
{
  byte rc;

  Set_LED_Dynamic(0xff);
  memset(LEDs_Data, 0, RGB_LED_DATA_SIZE);

  write(XT1511_I2C_COM_OFF, NULL, 0);
}

// 0xA1  按照现有数据刷新
void LED_COLOR::LED_Updata(void)
{
  byte rc;

  write(XT1511_I2C_COM_UODATA, NULL, 0);
}

// 0xA2  刷新预存数据 (指令预留)
void LED_COLOR::LED_Show(uint8_t number)
{
  byte rc;
  uint8_t reg;

  reg = number;

  rc = write(XT1511_I2C_COM_SHOW, &reg, sizeof(reg)); //COM全开
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.println("# LED_COLOR show fail");
    #endif
  }
}

/* 
 * 设置彩灯的动态显示模式，有四种模式，
 *   当设置好模式后，编辑好的彩灯显示将一直以该模式显示直到自行改变
 * 
 * @parameters: 动态模式的参数，有四种 0：静态 1：闪烁 2：滚动 3：呼吸
 * @return: 
 */
void LED_COLOR::Set_LED_Dynamic(uint8_t dynamicMode)
{
  LED_Dynamic = dynamicMode;
  ledDynamicIndex = 0;
}

/* 
 * 彩灯刷新，根据设置的动态效果 LED_Dynamic 进行对应的更新操作
 * 
 * @parameters: 
 * @return: 
 */
void LED_COLOR::LED_Flush(void)
{
  switch (LED_Dynamic)
  {
  case 0:
    LED_Flush_Static();
    break;
  case 1:
    LED_Flush_Blink();
    break;
  case 2:
    LED_Flush_Roll();
    break;
  case 3:
    LED_Flush_Breath();
    break;
  case 0xff: // 熄灭
    break;
  default:
    Serial.println("### No LED_Dynamic ###");
    break;
  }
}

void LED_COLOR::LED_Flush_Static()
{
  byte rc;

  // 静态显示：只有第一次会去写彩灯灯板的显示数据寄存器
  if (ledDynamicIndex >= 10)
  {
    // 不能一次连续12个，分为两次写入
    rc = write(0x01, LEDs_Data, 18);
    if (rc != 0)
    {
      #ifdef DEBUG_COLOR_LEDS
      Serial.printf("#color Static W1 error ###%d#### \n", rc);
      #endif
      return;
    }
    rc = write(0x07, LEDs_Data + 18, 18);
    if (rc != 0)
    {
      #ifdef DEBUG_COLOR_LEDS
      Serial.printf("#color Static W2 error ###%d#### \n", rc);
      #endif
      return;
    }

    LED_Updata();
    ledDynamicIndex = 0; // 静态显示只会做一次显示的动作，把ledDynamicIndex设置为5，这个函数就不会做事情了
  }
  else if (ledDynamicIndex < 10){
    ledDynamicIndex++; // 延时一段时间去更新，防止重复频繁更新会卡死 彩灯的IIC
  }else{

  }
}

void LED_COLOR::LED_Flush_Breath()
{
  byte rc;

  for (byte i = 0; i < sizeof(LEDs_Data); i++)
  {
    LEDs_DataResult[i] = (byte)((((float)LEDs_Data[i]) / 2) * (cos(ledDynamicIndex * 2 * PI / 200 - PI) + 1));
  }
  // 不能一次连续12个，分为两次写入
  rc = write(0x01, LEDs_DataResult, 18);
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.printf("#color Breath W1 error ###%d#### \n", rc);
    #endif
    return;
  }
  rc = write(0x07, LEDs_DataResult + 18, 18);
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.printf("#color Breath W2 error ###%d#### \n", rc);
    #endif
    return;
  }

  LED_Updata();

  if (ledDynamicIndex++ > 200)
  {
    ledDynamicIndex = 0;
  }
}

#define BLINK_PERIOD 20 // BLINK_PERIOD*30ms
void LED_COLOR::LED_Flush_Blink()
{
  byte rc;

  if (ledDynamicIndex == 1)
  {
    rc = write(0x01, LEDs_Data, 18);
    if (rc != 0)
    {
      #ifdef DEBUG_COLOR_LEDS
      Serial.printf("#color Blink W1 error ###%d#### \n", rc);
      #endif
      return;
    }
    rc = write(0x07, LEDs_Data + 18, 18);
    if (rc != 0)
    {
      #ifdef DEBUG_COLOR_LEDS
      Serial.printf("#color Blink W2 error ###%d#### \n", rc);
      #endif
      return;
    }
    LED_Updata(); // turn on
  }
  else if (ledDynamicIndex == BLINK_PERIOD)
  {
    write(XT1511_I2C_COM_OFF, NULL, 0);
  }
  else if (ledDynamicIndex >= BLINK_PERIOD + BLINK_PERIOD)
  {
    ledDynamicIndex = 0; // reset
  }
  else{

  }

  ledDynamicIndex++;
}

void LED_COLOR::LED_Flush_Roll()
{
  byte rc, resultIndex;

  resultIndex = ledDynamicIndex / 5;
  if (resultIndex >= 6)
  {
    ledDynamicIndex = 0;
    resultIndex = 0;
  }
  for (byte i = 0; i < 6; i++)
  {
    resultIndex++;
    if (resultIndex > 5)
      resultIndex -= 6;
    LEDs_DataResult[resultIndex * 3] = LEDs_Data[i * 3];
    LEDs_DataResult[resultIndex * 3 + 1] = LEDs_Data[i * 3 + 1];
    LEDs_DataResult[resultIndex * 3 + 2] = LEDs_Data[i * 3 + 2];
    LEDs_DataResult[resultIndex * 3 + 18] = LEDs_Data[i * 3 + 18];
    LEDs_DataResult[resultIndex * 3 + 1 + 18] = LEDs_Data[i * 3 + 1 + 18];
    LEDs_DataResult[resultIndex * 3 + 2 + 18] = LEDs_Data[i * 3 + 2 + 18];
  }
  // 不能一次连续12个，分为两次写入
  rc = write(0x01, LEDs_DataResult, 18);
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.printf("#color Roll W1 error ###%d#### \n", rc);
    #endif
    return;
  }
  rc = write(0x07, LEDs_DataResult + 18, 18);
  if (rc != 0)
  {
    #ifdef DEBUG_COLOR_LEDS
    Serial.printf("#color Roll W2 error ###%d#### \n", rc);
    #endif
    return;
  }

  LED_Updata();

  ledDynamicIndex++;
}