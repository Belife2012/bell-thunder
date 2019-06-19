#include "Sensor_IIC.h"

#define FAN_IIC_ADDR    (0x03)

#define FAN_REG_SPEED   (0x10)

class MOTOR_FAN : public SENSOR_IIC
{
private:
    unsigned char fan_speed;
public:
    MOTOR_FAN(int slave_address) : SENSOR_IIC(slave_address) {};

    void Set_Fan_Speed(unsigned char speed);
    unsigned char Get_Fan_Speed(void);
};