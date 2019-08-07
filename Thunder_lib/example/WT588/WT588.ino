#include "speaker_thunder.h"

SPEAKER_WT588 Speaker_Thunder = SPEAKER_WT588(16,4); //配置引脚16 --> DATA 和 4 --> BUSY

void setup()
{
  Serial.begin(115200); 
  
  Speaker_Thunder.Set_Sound_Volume(10);
}
void loop()
{
  int busy_flag;
  
  for(int i=30; i<40; i++)
  {
    Speaker_Thunder.Play_Song(i);
    Serial.print("Play_Song : ");// 0  播放中  1 停止
    Serial.println(i); 
    delay(5000); 
    busy_flag = Speaker_Thunder.WT588_Busy_Check(); 
    Serial.print("songstatus is : ");// 0  播放中  1 停止
    Serial.println(busy_flag); 
  }
}
