#include "sensor_sound.h"

unsigned char SENSOR_SOUND::GetSoundDB(unsigned char sensorChannel)
{
    unsigned char read_data = 0;
    unsigned char ret,times;

    for(times=0; times<5; times++){
        ret = read(SOUND_IIC_REG_DB_ADDR, &read_data, 1, sensorChannel);
        if(ret == 0) break;
    }
    if(ret != 0) Serial.println("sound IIC error");

    return read_data;
}

void SENSOR_SOUND::SetDetectRange(unsigned char sensorChannel, unsigned char max_value, unsigned char min_value)
{
    unsigned char ret,times;

    for(times=0; times<5; times++){
        ret = write(SOUND_IIC_REG_SETMAX_ADDR, &max_value, 1, sensorChannel);
        if(ret != 0) continue;
        ret = write(SOUND_IIC_REG_SETMIN_ADDR, &min_value, 1, sensorChannel);
        if(ret != 0) continue;

        break;
    }

    return;
}