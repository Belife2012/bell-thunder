#include "WT588.h"

WT588 Speaker = WT588(16,4); //配置引脚16 --> DATA 和 4 --> BUSY

void setup()
{
  Serial.begin(115200); 
  
  Speaker.Set_Sound_Volume(10);
}
void loop()
{
  int busy_flag;
  
  for(int i=30; i<40; i++)
  {
    Speaker.Play_Song(i);
    Serial.print("Play_Song : ");// 0  播放中  1 停止
    Serial.println(i); 
    delay(5000); 
    busy_flag = Speaker.WT588_Busy_Check(); 
    Serial.print("songstatus is : ");// 0  播放中  1 停止
    Serial.println(busy_flag); 
  }
}
