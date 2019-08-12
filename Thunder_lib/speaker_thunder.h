#ifndef _WT588_H_
#define _WT588_H_

#include "Arduino.h"
#include <FreeRTOS.h>

class SPEAKER_WT588
{
public:
  SPEAKER_WT588(int DataPin, int BusyPin); // 配置引脚

  int WT588_Busy_Check(void);      // 获取播放状态

  /*--------------Thunder IDE APIs: -------------*/
  void Set_Sound_Volume(int data); // 音量调节 范围0~15
  void Play_Song(int data);        // 播放声音
private:
  int Data_pin;
  int Busy_pin;
  FreeRTOS::Semaphore semaphorePlay = FreeRTOS::Semaphore("play");

  void send_data(int data); // 类内部使用，按WT588一线串口时序图发送数据
};
#endif