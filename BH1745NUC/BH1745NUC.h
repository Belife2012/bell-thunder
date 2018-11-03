/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 颜色传感器模块库文件
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
 *  1.  BH1745NUC(int slave_address);                                   // 配置I2C地址
 *  2.  byte Setup(void);                                               // 初始化设置
 *  3.  byte Get_RGBC_Data(unsigned short *data);                       // 获取RGBC
 *  4.  void RGBtoHSV(unsigned short *RGBC, float *HSV);                // 计算HSV
 *  5.  uint8_t Colour_Recognition(unsigned short *RGBC, float *HSV);   // 识别颜色 (预留)
 * 
 ************************************************/

#ifndef _BH1745NUC_H_
#define _BH1745NUC_H_

#define BH1745NUC_DEVICE_ADDRESS_38            (0x38)    // 7bit Addrss
#define BH1745NUC_DEVICE_ADDRESS_39            (0x39)    // 7bit Addrss
#define BH1745NUC_PART_ID_VAL                  (0x0B)
#define BH1745NUC_MANUFACT_ID_VAL              (0xE0)

#define BH1745NUC_SYSTEM_CONTROL               (0x40)
#define BH1745NUC_MODE_CONTROL1                (0x41)
#define BH1745NUC_MODE_CONTROL2                (0x42)
#define BH1745NUC_MODE_CONTROL3                (0x44)
#define BH1745NUC_RED_DATA_LSB                 (0x50)
#define BH1745NUC_MANUFACTURER_ID              (0x92)

#define BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS (0x00)

#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X1    (0)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X2    (1)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16   (2)
#define BH1745NUC_MODE_CONTROL2_RGBC_EN        (1 << 4)

#define BH1745NUC_MODE_CONTROL1_VAL            (BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS)
#define BH1745NUC_MODE_CONTROL2_VAL            (BH1745NUC_MODE_CONTROL2_RGBC_EN | BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16)
#define BH1745NUC_MODE_CONTROL3_VAL            (0x02)

class BH1745NUC
{
  public:
    BH1745NUC(int slave_address);                     // 配置I2C地址
    
    byte Setup(void);                                 // 初始化设置
    byte Get_RGBC_Data(unsigned short *data);         // 获取RGBC
    void RGBtoHSV(unsigned short *RGBC, float *HSV);  // 计算HSV

    uint8_t Colour_Recognition(unsigned short *RGBC, float *HSV); // 识别颜色 (预留)

  private:
    byte _device_address;
    byte device_detected;// 0为未插入设备，!0为已插入设备

    byte get_rawval(unsigned char *data);                                               // 类内部使用，读取传感器数据
    byte write(unsigned char memory_address, unsigned char *data, unsigned char size);  // 类内部使用，I2C通讯，发送
    byte read(unsigned char memory_address, unsigned char *data, unsigned char size);             // 类内部使用，I2C通讯，发送并读取
};

#endif // _BH1745NUC_H_
