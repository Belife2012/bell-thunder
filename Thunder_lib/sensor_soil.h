#ifndef __SENSOR_SOIL_H__
#define __SENSOR_SOIL_H__
#include "Sensor_IIC.h"

#define SOIL_IIC_ADDR               (0x08)
#define SOIL_IIC_REG_HUM_ADDR       (0x01)
#define SOIL_IIC_REG_SETMIN_ADDR    (0x10)
#define SOIL_IIC_REG_SETMAX_ADDR    (0x20)

class SENSOR_SOIL : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_SOIL(int slave_address):SENSOR_IIC(slave_address) {};

    unsigned char GetHumidity(unsigned char sensorChannel=0);
    void SetDetectRange(unsigned char max_value = 100, unsigned char min_value = 0, unsigned char sensorChannel=0);
};

#endif
