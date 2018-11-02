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

#ifndef _US_I2C_H_
#define _US_I2C_H_

class US_I2C
{
public:
  US_I2C(int slave_address); // 配置I2C地址

  byte Get_US_Data(unsigned short *data); // 获取超声波数据
  float Get_US_cm(void);                  // 获取超声波数据，0.1[cm]

private:
  byte _device_address;

  byte get_rawval(unsigned char *data);                                              // 类内部使用，I2C通讯，读取超声波数据
  byte write(unsigned char memory_address, unsigned char *data, unsigned char size); // 类内部使用，I2C通讯，发送
  byte read(unsigned char memory_address, unsigned char *data, unsigned char size);            // 类内部使用，I2C通讯，发送并读取
};

#endif
