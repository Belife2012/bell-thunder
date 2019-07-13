#ifndef __SENSOR_GAS_H__
#define __SENSOR_GAS_H__
#include "Sensor_IIC.h"

#define GAS_IIC_ADDR            (0x06)
#define GAS_IIC_REG_RATIO_ADDR  (0x01)
#define GAS_IIC_REG_SETMIN_ADDR  (0x10)
#define GAS_IIC_REG_SETMAX_ADDR  (0x20)

class SENSOR_GAS : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_GAS(int slave_address):SENSOR_IIC(slave_address) {};

    unsigned char GetToxicgasRatio(unsigned char sensorChannel=0);
    void SetDetectRange(unsigned char max_value = 100, unsigned char min_value = 0, unsigned char sensorChannel=0);
};

#endif
