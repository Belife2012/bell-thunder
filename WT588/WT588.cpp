/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 语音控制芯片库文件
 * 
 *   创建日期： 20180606
 *   作者：     宋博伟
 *   邮箱：     songbw123@163.com
 *
 *   版本：     v0.2
 *   修改日期   20180721
 *   修改：     宋博伟
 *   邮箱：     songbw123@163.com
 *   修改内容： 
 * 
 *   
 * 
 * 功能列表：
 *  1.  WT588(int DataPin,int BusyPin);		// 配置引脚
 * 	2.	void Set_Sound_Volume(int data);	// 音量调节 范围0~15
 * 	3.	void Play_Song(int data);			// 播放声音
 * 	4.	int WT588_Busy_Check(void);			// 获取播放状态
 * 
 ************************************************/

#include "WT588.h"
#include <Arduino.h>
#include <Task_Mesg.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"

//#define SERIAL_8N0 0x800000c
#define ENABLE_WT588_SPINLOCK    1

#if ENABLE_WT588_SPINLOCK
// portMUX_TYPE spinlockMUX_WT588;
#endif


// 配置引脚
// 参数DP --> DataPin,主控接芯片WT588的一线串口通讯数据脚，IO输出口
// 参数BP --> BusyPin,主控接芯片WT588的BUSY状态脚，IO输入口
WT588::WT588(int DP, int BP)
{
	pinMode(DP,OUTPUT);
	pinMode(BP,INPUT);

	Data_pin=DP;
	Busy_pin=BP;

	//使用 UART 模拟一线串口 输出到 WT588 data线
	//speakSerial = uartBegin(2, 9600, SERIAL_8N0, 4, 16, 12, false);

#if ENABLE_WT588_SPINLOCK
    //initialize spinlockMUX_WT588
    // vPortCPUInitializeMutex(&spinlockMUX_WT588);
#endif
}

// 音量调节 范围0~15
// 参数 --> 声音音量，范围0-15，0为静音
void WT588::Set_Sound_Volume(int data)
{
	if(data > 15)
	{
		data = 15;
	}
	else if(data < 0)
	{
		data = 0;
	}
	
	send_data(0xFFE0 + data);
}

// 播放声音
// 参数 --> 歌曲的地址编号
void WT588::Play_Song(int data)
{
	send_data(data);
}

// 获取播放状态
// 返回 --> 音频播放状态，即读取BUSY脚的状态
int WT588::WT588_Busy_Check()
{
	int busy_flag;
	busy_flag = digitalRead(Busy_pin);
	return busy_flag;
}

// 类内部使用，按WT588一线串口时序图发送数据
void WT588::send_data(int data)
{
	uint32_t currentTime;
	uint32_t lastTime;

	digitalWrite(Data_pin, HIGH); 
    //delay(6);
	lastTime = millis();
	for(;;){
		currentTime = millis();
		if(currentTime > lastTime+5) break;
	}

	int ddata_temp;
	int pdata_temp;
	int sddata_temp;
	int spdata_temp;
	int ddtata_tempp;
	int B_DATA;	
	ddata_temp=data;

	pdata_temp = ddata_temp & 0X00FF;
	sddata_temp = ddata_temp>>8;
	spdata_temp = pdata_temp<<8;
	ddtata_tempp=sddata_temp|spdata_temp;

	digitalWrite(Data_pin, LOW);
	//delay(6);
	lastTime = millis();
	for(;;){
		currentTime = millis();
		if(currentTime > lastTime+5) break;
	}

	B_DATA = (ddtata_tempp & 0X0001);

#if ENABLE_WT588_SPINLOCK
	//enter critical
	//taskENTER_CRITICAL(&spinlockMUX_WT588);
	Task_Mesg.Suspend_Others_AppsTask();
#endif
    for(int i=0;i<16;i++)
	{
		if(i==8)
		{
			digitalWrite(Data_pin, HIGH); 
			
		#if ENABLE_WT588_SPINLOCK
			//exit critical
			//taskEXIT_CRITICAL(&spinlockMUX_WT588);
			Task_Mesg.Resume_Others_AppsTask();
		#endif
			//delay(2);
			lastTime = millis();
			for(;;){
				currentTime = millis();
				if(currentTime > lastTime+2) break;
			}
			digitalWrite(Data_pin, LOW);
			//delay(5);
			lastTime = millis();
			for(;;){
				currentTime = millis();
				if(currentTime > lastTime+5) break;
			}

		#if ENABLE_WT588_SPINLOCK
			//enter critical
			//taskENTER_CRITICAL(&spinlockMUX_WT588);
			Task_Mesg.Suspend_Others_AppsTask();
		#endif
		}
		
		digitalWrite(Data_pin, HIGH); 
 		if(B_DATA==0)
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
		ddtata_tempp=ddtata_tempp>>1;
		B_DATA=(ddtata_tempp&0x0001);
	}
	digitalWrite(Data_pin, HIGH); 

#if ENABLE_WT588_SPINLOCK
	//exit critical
	// taskEXIT_CRITICAL(&spinlockMUX_WT588);
	Task_Mesg.Resume_Others_AppsTask();
#endif
	//Serial.println("unlock");
}