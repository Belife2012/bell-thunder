#ifndef __SENSOR_TEMP_H__
#define __SENSOR_TEMP_H__
#include "Sensor_IIC.h"

#define TEMP_IIC_ADDR         (0x07)
#define TEMP_IIC_REG_LOWD_ADDR      (0x01)
#define TEMP_IIC_REG_HIGHD_ADDR     (0x02)

class SENSOR_TEMP : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_TEMP(int slave_address) : SENSOR_IIC(slave_address) { };

    float GetTemperature(uint8_t sensorChannel=0);
};

#endif
