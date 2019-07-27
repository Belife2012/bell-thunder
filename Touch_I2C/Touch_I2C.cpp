#include <Arduino.h>
#include <Wire.h>
#include <Touch_I2C.h>

/* 
 * 构造函数
 * 
 * @parameters: 触碰模块的 IIC 地址
 * @return: 
 */
TOUCH_I2C::TOUCH_I2C(int slave_address) : 
  SENSOR_IIC(slave_address)
{}

/* 
 * 获取触碰状态，0为未按下状态，1为按下状态，其他的过程值 可以通过连续获取状态做判断触碰过程
 * 
 * @parameters: 
 *      0 未按下状态
 *      1 按下状态
 *      2 如果有发生过 “按下后释放”，第一次获取的状态为触碰（2）
 * @return: 
 *      0 获取数据正常
 *      非0 获取数据出错
 */
byte TOUCH_I2C::Get_Status(byte *readValue, unsigned char channel)
{
  unsigned char ret;

  ret = read(TOUCH_ADDRESS_VALUE, readValue, 1, channel);
  if(*readValue == 0){
    if(Check_Touch_Event(TOUCH_EVENT_TOUCH, channel) == true){
      *readValue = 2;
    }
  }

  return ret;
}

/* 
 * 复位工作模式：自动模式，未按下亮绿灯，按下亮红灯
 * 
 * @parameters: 
 * @return: 
 */
byte TOUCH_I2C::Reset_Mode(unsigned char channel)
{
  unsigned char ctl_data, ret;

  ctl_data = 0;

  ret = write(TOUCH_ADDRESS_CONTROL, &ctl_data, 1, channel);

  return ret;
}

/* 
 * 检查触碰按键的相关按键事件是否有发生
 * 
 * @parameters: 
 * @return: true是有发生过，false是未发生过
 */
bool TOUCH_I2C::Check_Touch_Event(enum_touch_event check_event, unsigned char channel)
{
  byte ret;
  byte readValue = 0;

  if(check_event == TOUCH_EVENT_RELEASE){
    ret = read(TOUCH_ADDRESS_RELEASE, &readValue, 1, channel);
  }else if(check_event == TOUCH_EVENT_PRESS){
    ret = read(TOUCH_ADDRESS_PRESS, &readValue, 1, channel);
  }else if(check_event == TOUCH_EVENT_TOUCH){
    ret = read(TOUCH_ADDRESS_TOUCH, &readValue, 1, channel);
  }

  if(ret != 0){
    return false;
  }else{
    return readValue;
  }
}

/* 
 * 设置触碰模块的LED灯颜色，范围 0-255
 * 
 * @parameters: 全部传入0 值时，即为关闭LED灯
 *      RedValue LED亮度的红色分量，范围 0-255
 *      GreenValue LED亮度的绿色分量，范围 0-255
 *      BlueValue LED亮度的蓝色分量，范围 0-255
 * @return: 
 *      0 写数据正常
 *      非0 写数据出错
 */
byte TOUCH_I2C::Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue, unsigned char channel)
{
  unsigned char rgb[3], ret;

  rgb[0] = RedValue;
  rgb[1] = GreenValue;
  rgb[2] = BlueValue;

  ret = write(TOUCH_ADDRESS_RGB, rgb, 3, channel);

  return ret;
}

int TOUCH_I2C::Thunder_Get_Touch_Data(uint8_t sensorChannel)
{
    byte statusValue1;
    Get_Status( &statusValue1, sensorChannel);
    return statusValue1;
}

void TOUCH_I2C::Thunder_Set_Touch_Value(uint8_t sensorChannel, byte RedValue, byte GreenValue, byte BlueValue)
{
  Set_LED_RGBvalue(RedValue, GreenValue, BlueValue, sensorChannel);
}

bool TOUCH_I2C::Thunder_Get_Touch_Status(uint8_t sensorChannel, int status) 
{
  if(status == 0) {
    return Check_Touch_Event(TOUCH_EVENT_RELEASE, sensorChannel);
  } else if(status == 1) {
    return Check_Touch_Event(TOUCH_EVENT_PRESS, sensorChannel);
  } else if(status == 2) {
    return Check_Touch_Event(TOUCH_EVENT_TOUCH, sensorChannel);
  }
}
