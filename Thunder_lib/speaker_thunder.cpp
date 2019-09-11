#include "speaker_thunder.h"
#include <Arduino.h>
#include "os_function.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"

//#define SERIAL_8N0 0x800000c
#define ENABLE_WT588_SPINLOCK 1

#if ENABLE_WT588_SPINLOCK
// portMUX_TYPE spinlockMUX_WT588;
#endif

// 配置引脚
// 参数DP --> DataPin,主控接芯片WT588的一线串口通讯数据脚，IO输出口
// 参数BP --> BusyPin,主控接芯片WT588的BUSY状态脚，IO输入口
SPEAKER_THUNDER::SPEAKER_THUNDER(int DP, int BP)
{
  pinMode(DP, OUTPUT);
  pinMode(BP, INPUT);

  Data_pin = DP;
  Busy_pin = BP;

  //使用 UART 模拟一线串口 输出到 SPEAKER_THUNDER data线
  //speakSerial = uartBegin(2, 9600, SERIAL_8N0, 4, 16, 12, false);

#if ENABLE_WT588_SPINLOCK
  //initialize spinlockMUX_WT588
  // vPortCPUInitializeMutex(&spinlockMUX_WT588);
#endif
}

// 类内部使用，按WT588一线串口时序图发送数据
void SPEAKER_THUNDER::send_data(int data)
{
  uint32_t currentTime;
  uint32_t lastTime;
  TASK_SUPREME this_supreme;

  semaphorePlay.take(std::string("send"));

  digitalWrite(Data_pin, HIGH);
  //delay(6);
  lastTime = millis();
  for (;;)
  {
    currentTime = millis();
    if (currentTime > lastTime + 5)
      break;
  }

  int ddata_temp;
  int pdata_temp;
  int sddata_temp;
  int spdata_temp;
  int ddtata_tempp;
  int B_DATA;
  ddata_temp = data;

  pdata_temp = ddata_temp & 0X00FF;
  sddata_temp = ddata_temp >> 8;
  spdata_temp = pdata_temp << 8;
  ddtata_tempp = sddata_temp | spdata_temp;

  digitalWrite(Data_pin, LOW);
  //delay(6);
  lastTime = millis();
  for (;;)
  {
    currentTime = millis();
    if (currentTime > lastTime + 5)
      break;
  }

  B_DATA = (ddtata_tempp & 0X0001);

#if ENABLE_WT588_SPINLOCK
  //enter critical
  //taskENTER_CRITICAL(&spinlockMUX_WT588);
  this_supreme.Set_Current_Task_Supreme();
#endif
  for (int i = 0; i < 16; i++)
  {
    if (i == 8)
    {
      digitalWrite(Data_pin, HIGH);

      #if ENABLE_WT588_SPINLOCK
        //exit critical
        //taskEXIT_CRITICAL(&spinlockMUX_WT588);
        this_supreme.Clear_Current_Task_Supreme();
      #endif
      //delay(2);
      lastTime = millis();
      for (;;)
      {
        currentTime = millis();
        if (currentTime > lastTime + 2)
          break;
      }
      digitalWrite(Data_pin, LOW);
      //delay(5);
      lastTime = millis();
      for (;;)
      {
        currentTime = millis();
        if (currentTime > lastTime + 5)
          break;
      }

    #if ENABLE_WT588_SPINLOCK
      //enter critical
      //taskENTER_CRITICAL(&spinlockMUX_WT588);
      this_supreme.Set_Current_Task_Supreme();
    #endif
    }

    digitalWrite(Data_pin, HIGH);
    if (B_DATA == 0)
    {
      delayMicroseconds(65);
      digitalWrite(Data_pin, LOW);
      delayMicroseconds(195);
    }
    else
    {
      delayMicroseconds(195);
      digitalWrite(Data_pin, LOW);
      delayMicroseconds(65);
    }
    ddtata_tempp = ddtata_tempp >> 1;
    B_DATA = (ddtata_tempp & 0x0001);
  }
  digitalWrite(Data_pin, HIGH);
#if ENABLE_WT588_SPINLOCK
  //exit critical
  // taskEXIT_CRITICAL(&spinlockMUX_WT588);
  this_supreme.Clear_Current_Task_Supreme();
#endif
  
  semaphorePlay.give();
}

// 获取播放状态
// 返回 --> 音频播放状态，即读取BUSY脚的状态
int SPEAKER_THUNDER::WT588_Busy_Check()
{
  int busy_flag;
  busy_flag = digitalRead(Busy_pin);
  return busy_flag;
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief 物理上音量有15档，此函数输入有效范围 0~100 
 * 
 * @param data：最大音量的百分比
 */
void SPEAKER_THUNDER::Set_Sound_Volume(int data)
{
  if (data > 100){
    data = 15;
  }
  else if (data < 0){
    data = 0;
  }
  else {
    data = data * 15 / 100;
  }

  send_data(0xFFE0 + data);
}

/**
 * @brief: 播放声音
 * 
 * @param data: 歌曲的地址编号，
 * 例如 音符A音调0的编号，可以使用 SPEAKER_THUNDER::SOUND_MUSIC_A0 来代表 
 */
void SPEAKER_THUNDER::Play_Song(int data)
{
  send_data(data);
}