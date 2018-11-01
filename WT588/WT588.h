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

#ifndef _WT588_H_
#define _WT588_H_

#include "Arduino.h"

class WT588
{
public:
  WT588(int DataPin, int BusyPin); // 配置引脚

  void Set_Sound_Volume(int data); // 音量调节 范围0~15
  void Play_Song(int data);        // 播放声音
  int WT588_Busy_Check(void);      // 获取播放状态

private:
  int Data_pin;
  int Busy_pin;

  void send_data(int data); // 类内部使用，按WT588一线串口时序图发送数据
};
#endif