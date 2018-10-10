/*****************************************************************************
  HT16D35B.ino

 Copyright (c) 2016 ROHM Co.,Ltd.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
#include <Wire.h>
#include <HT16D35B.h>

HT16D35B HT16D35B(0x69);

uint8_t LED_BUFF[29] =  
{
  0x00, //地址
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00  //不用的
};
  
void setup() 
{
  byte rc;

  Serial.begin(115200);
  while (!Serial);
  
  Wire.begin(21,22); //Wire.begin();
  
  rc = HT16D35B.Setup();
}

void loop() 
{
  for(int i = 1;i < 29; i++)
  {
      delay(500);
      LED_BUFF[i] = 0xFF; 
      HT16D35B.LED_Show(LED_BUFF, sizeof(LED_BUFF));
      LED_BUFF[i] = 0x00;
  }
  delay(1000);
}
