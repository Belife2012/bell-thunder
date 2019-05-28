#include "sensor_flame.h"

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

bool SENSOR_FLAME::Check_Flame()
{
    if( 50 < Get_Flame_Intensity() ){
        return true;
    }else {
        return false;
    }
}
