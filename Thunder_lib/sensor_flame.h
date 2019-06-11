#ifndef _SENSOR_FLAME_
#define _SENSOR_FLAME_

#include "Sensor_IIC.h"

#define FLAME_IIC_ADDR      (0x02)

#define FLAME_REG_ANGLE         (0x01)
#define FLAME_REG_INTENSITY     (0x02)

#define CHECK_FLAME_INTENSITY   (30)

class SENSOR_FLAME : public SENSOR_IIC
{
public:
    SENSOR_FLAME(int slave_address) : SENSOR_IIC(slave_address) {};

    int8_t Get_Flame_Angle();
    unsigned char Get_Flame_Intensity();
    bool Check_Flame();
private:
    int8_t angle;
    unsigned char intensity;
};

#endif
