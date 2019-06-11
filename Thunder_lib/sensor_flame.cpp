#include "sensor_flame.h"

/**
 * @brief 获取传感器的红外光源相对角度，可以通过返回数值判断火焰位置的
 *  偏离方向和偏离程度
 * 
 * @return int8_t 红外光源相对角度，取值范围 -45 ~ 45
 */
int8_t SENSOR_FLAME::Get_Flame_Angle()
{
    unsigned char read_angle;
    if(0 == read(FLAME_REG_ANGLE, &read_angle, 1)){
        angle = read_angle;
    }else {
        angle = 0;
    }

    return angle;
}

/**
 * @brief 获取传感器的红外光相对强度，可以通过返回数值判断火焰相对强度
 * 
 * @return unsigned char 红外光相对强度，取值范围0~100
 */
unsigned char SENSOR_FLAME::Get_Flame_Intensity()
{
    unsigned char read_intensity;
    if(0 == read(FLAME_REG_INTENSITY, &read_intensity, 1)){
        intensity = read_intensity;
    }else {
        intensity = 0;
    }

    return intensity;
}

/**
 * @brief 判断有没有火苗，如果测量到红外强度大于CHECK_FLAME_INTENSITY，
 *  则判断为有火苗，否则判断为没有火苗；可以通过改变CHECK_FLAME_INTENSITY
 *  修改灵敏度（如果CHECK_FLAME_INTENSITY设置太小，太阳光就会判断为有火苗）
 * 
 * @return true 有火苗
 * @return false 没有火苗
 */
bool SENSOR_FLAME::Check_Flame()
{
    if( CHECK_FLAME_INTENSITY < Get_Flame_Intensity() ){
        return true;
    }else {
        return false;
    }
}
