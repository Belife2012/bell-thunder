#include "iic_thunder.h"

#define FAN_IIC_ADDR    (0x03)

#define FAN_REG_SPEED   (0x10)

class MOTOR_FAN : public SENSOR_IIC
{
private:
    
public:
    MOTOR_FAN(int slave_address) : SENSOR_IIC(slave_address) {};
    signed char Get_Fan_Speed(unsigned char channel=0);

    /*--------------Thunder IDE APIs: -------------*/
    void Set_Fan_Speed(signed char speed, unsigned char channel=0);
};
