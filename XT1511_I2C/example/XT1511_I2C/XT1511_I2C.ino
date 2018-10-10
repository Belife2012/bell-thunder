/*****************************************************************************
  XT1511_I2C.ino

******************************************************************************/
#include <Wire.h>
#include <XT1511_I2C.h>

XT1511_I2C XT1511_I2C(0x11);

uint8_t LED_Data1[18] = { 20, 0, 25, 0, 25, 25, 0, 0, 25, 0, 25, 25,  0, 25, 25, 0, 25, 25};
uint8_t LED_Data2[18] = { 10, 0,  0, 0, 25,  0, 0, 0, 25, 0, 25, 25, 20,  0, 25, 0, 25, 25};
  
void setup() 
{
  Serial.begin(115200);
  while (!Serial);
  
  Wire.begin();

  delay(500);
//  XT1511_I2C.Set_LED_Num(12); //写入一共可以控制多少个灯
}

void loop()
{
//  XT1511_I2C.Set_LED_Data(0x01,25,25,25);   //第几个灯(1开始)，R,G,B
  for(int i = 1; i < 13; i++)
  {
    XT1511_I2C.Set_LED_Data(i,5,5,5);   //第几个灯(1开始)，R,G,B
    delay(100);
    XT1511_I2C.LED_Updata();  //按照现有数据刷新
    delay(500);
    XT1511_I2C.Set_LED_Data(i,0,0,0);   //第几个灯(1开始)，R,G,B
    delay(500);
  }

  XT1511_I2C.LED_OFF();   //全关，立即刷新
  delay(1000);
  
  XT1511_I2C.Set_LEDs_Data(1, LED_Data1, sizeof(LED_Data1));   //写入多个寄存器数据
  delay(100);
  XT1511_I2C.LED_Updata();  //按照现有数据刷新
  delay(1000);
  XT1511_I2C.Set_LEDs_Data(7, LED_Data2, sizeof(LED_Data2));   //写入多个寄存器数据
  delay(100);
  XT1511_I2C.LED_Updata();  //按照现有数据刷新
  delay(1000);
  
  XT1511_I2C.LED_OFF();   //全关，立即刷新
  delay(1000);
//  XT1511_I2C.LED_Show(1);   //刷新预存数据
}
