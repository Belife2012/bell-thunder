/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 超声波模块库文件
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
 *  1.  US_I2C(int slave_address);                // 配置I2C地址
 *  2.  byte Get_US_Data(unsigned short *data);   // 获取超声波数据
 *  3.  float Get_US_cm(void);                    // 获取超声波数据，0.1[cm] 
 * 
 ************************************************/
#include "Sensor_IIC.h"

#ifndef _US_I2C_H_
#define _US_I2C_H_

class US_I2C : public SENSOR_IIC
{
public:
  US_I2C(int slave_address); // 配置I2C地址

  byte Get_US_Data(unsigned short *data, unsigned char channel=0); // 获取超声波数据
  float Get_US_cm(unsigned char channel=0);                  // 获取超声波数据，0.1[cm]

  int Thunder_Get_US_Data(uint8_t sensorChannel);
  int Thunder_Detect_Obstacle(uint8_t sensorChannel);

private:

  byte get_rawval(unsigned char *data, unsigned char channel=0);                                              // 类内部使用，I2C通讯，读取超声波数据
};

#endif
