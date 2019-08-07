#ifndef __SENSOR_HUMAN_H__
#define __SENSOR_HUMAN_H__
#include "iic_thunder.h"

#define HUMAN_IIC_ADDR              (0x0A)
#define HUMAN_IIC_REG_STATUS_ADDR   (0x01)

class SENSOR_HUMAN : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_HUMAN(int slave_address) : SENSOR_IIC(slave_address) { };

    unsigned char GetStatus(uint8_t sensorChannel=0);
};

#endif
