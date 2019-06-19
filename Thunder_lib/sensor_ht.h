#ifndef __SENSOR_HT_H__
#define __SENSOR_HT_H__
#include "Sensor_IIC.h"

#define HT_IIC_ADDR         (0x05)
#define HT_IIC_REG_ADDR     (0x01)

class SENSOR_HT : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_HT(int slave_address);

    float ReadHumidity(uint8_t sensorChannel);
    float ReadTemperature(uint8_t sensorChannel);
};

#endif
