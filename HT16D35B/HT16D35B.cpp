/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 单色点阵灯库文件
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
 *  1.  HT16D35B(int slave_address);                    // 配置I2C地址
 *  2.  byte Setup(void);                               // 初始化设置
 *  3.  byte LED_Show(unsigned char *data, int size);   // 显示图案，参数*data为灯数据，参数size为数据长度
 * 
 ************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <HT16D35B.h>
#include <Thunder_Display.h>

// #define PRINT_DEBUG_ERROR

// 配置I2C地址
HT16D35B::HT16D35B(int slave_address) : 
  SENSOR_IIC(slave_address)
{
  device_detected = 0;
}

// 初始化设置
byte HT16D35B::Setup(void)
{
  Serial.printf("\nstart Init LED Matrix...\n");

  byte rc;
  uint8_t ROW_BUFF[4] = {0x7F, 0XFF, 0XFF, 0XFF}; //最后一个ROW不用
  uint8_t LED_BUFF[29] =
      {
          0x00, //地址
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00 //不用的
      };

  unsigned char reg;

  reg = 0xff;
  rc = write(HT16D35B_CONTROL_COM, &reg, sizeof(reg)); //COM全开
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_CONTROL_COM fail #"));
    return (rc);
  }

  rc = write(HT16D35B_CONTROL_ROW, ROW_BUFF, sizeof(ROW_BUFF)); // 最后一个ROW不用
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_CONTROL_ROW fail #"));
    return (rc);
  }

  reg = 0x01;
  rc = write(HT16D35B_MODE_GRAY, &reg, sizeof(reg)); // 选择模式：灰度模式: 0x00; 二进制模式0x01
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_MODE_GRAY fail #"));
    return (rc);
  }

  reg = 35;
  rc = write(HT16D35B_GLOBAL_BRIGHT, &reg, sizeof(reg)); // 全局亮度设置：该命令用于控制 64 级 PWM 亮度
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_GLOBAL_BRIGHT fail #"));
    return (rc);
  }

  rc = write(HT16D35B_DISPLAY_RAM, LED_BUFF, sizeof(LED_BUFF)); //写入数据
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_DISPLAY_RAM fail #"));
    return (rc);
  }

  reg = 0X03;
  rc = write(HT16D35B_SYSTEM_CONTROL, &reg, sizeof(reg));
  if (rc != 0)
  {
    Serial.println(F("# HT16D35B_SYSTEM_CONTROL fail #"));
    return (rc);
  }

  device_detected = 1;
  Serial.printf("Init LED Matrix end\n\n");
  return 0;
}

// 显示图案
// 参数*data --> 灯数据
// 参数size --> 数据长度
byte HT16D35B::LED_Show(const unsigned char *data, int size)
{
  byte rc, reg_value;

  // 测试 HTHT16D35B IIC是否连上，是否需要重新Setup()
  reg_value = 0xff;
  rc = write(HT16D35B_CONTROL_COM, &reg_value, sizeof(reg_value)); //COM全开
  if (rc != 0)
  {
  #ifdef PRINT_DEBUG_ERROR
    Serial.printf("# HT16D35B IIC Error! \n");
  #endif
    device_detected = 0;
    return (rc);
  }else if(device_detected == 0){
    Setup();
  }

  rc = write(HT16D35B_DISPLAY_RAM, data, size);

  if (rc != 0)
  {
  #ifdef PRINT_DEBUG_ERROR
    Serial.printf("### LED IIC Error! ###%d#### \n", rc);
  #endif
  }
  
  return (rc);
}
