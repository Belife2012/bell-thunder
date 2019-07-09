#include "sensor_gas.h"

unsigned char SENSOR_GAS::GetToxicgasRatio(unsigned char sensorChannel)
{
    unsigned char read_data = 0;
    unsigned char ret,times;

    for(times=0; times<5; times++){
        ret = read(GAS_IIC_REG_RATIO_ADDR, &read_data, 1, sensorChannel);
        if(ret == 0) break;
    }

    return read_data;
}

void SENSOR_GAS::SetDetectRange(unsigned char sensorChannel, unsigned char max_value, unsigned char min_value)
{
    unsigned char ret,times;

    for(times=0; times<5; times++){
        ret = write(GAS_IIC_REG_SETMAX_ADDR, &max_value, 1, sensorChannel);
        if(ret != 0) continue;
        ret = write(GAS_IIC_REG_SETMIN_ADDR, &min_value, 1, sensorChannel);
        if(ret != 0) continue;

        break;
    }

    return;
}