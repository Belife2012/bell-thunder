#ifndef _SENSOR_FLAME_
#define _SENSOR_FLAME_

#include "iic_thunder.h"

#define FLAME_IIC_ADDR      (0x02)

#define FLAME_REG_ANGLE         (0x01)
#define FLAME_REG_INTENSITY     (0x02)

#define CHECK_FLAME_INTENSITY   (50) // 检测强度大于此为有火苗

class SENSOR_FLAME : public SENSOR_IIC
{
public:
    SENSOR_FLAME(int slave_address) : SENSOR_IIC(slave_address) {};

    /*--------------Thunder IDE APIs: -------------*/
    int8_t Get_Flame_Angle(unsigned char channel=0);
    unsigned char Get_Flame_Intensity(unsigned char channel=0);
    bool Check_Flame(unsigned char channel=0);
private:
};

#endif
