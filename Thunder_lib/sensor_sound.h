#ifndef __SENSOR_SOUND_H__
#define __SENSOR_SOUND_H__
#include "Sensor_IIC.h"

#define SOUND_IIC_ADDR              (0x09)
#define SOUND_IIC_REG_DB_ADDR       (0x01)
#define SOUND_IIC_REG_SETMIN_ADDR   (0x10)
#define SOUND_IIC_REG_SETMAX_ADDR   (0x20)

class SENSOR_SOUND : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_SOUND(int slave_address):SENSOR_IIC(slave_address) {};

    unsigned char GetSoundDB(unsigned char sensorChannel);
    void SetDetectRange(unsigned char sensorChannel, unsigned char max_value = 100, unsigned char min_value = 0);
};

#endif
