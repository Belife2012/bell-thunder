#include "Touch_I2C.h"

/* 
 * 获取触碰状态，0为未按下状态，1为按下状态，其他的过程值 可以通过连续获取状态再做判断
 * 
 * @parameters: 
 * @return: 
 *      0 未按下状态
 *      1 按下状态
 */
byte TOUCH_I2C::Get_Status(void)
{

}

/* 
 * 设置触碰模块的LED灯颜色，范围 0-255
 * 
 * @parameters: 全部传入0 值时，即为关闭LED灯
 *      RedValue LED亮度的红色分量，范围 0-255
 *      GreenValue LED亮度的绿色分量，范围 0-255
 *      BlueValue LED亮度的蓝色分量，范围 0-255
 * @return: 
 */
void TOUCH_I2C::Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue)
{

}

