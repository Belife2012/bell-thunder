#include "sensor_human.h"

unsigned char SENSOR_HUMAN::GetStatus(uint8_t sensorChannel)
{
    uint8_t _data = 0;
    uint8_t ret,times;

    for(times=0; times < 5; times++) {
        ret = read(HUMAN_IIC_REG_STATUS_ADDR, &_data, 1, sensorChannel);
        if(ret != 0) continue;

        break;
    }
    _data = (_data==1) ? 0 : 1;

    // Serial.printf("GetTemperature: %d data: %d %d\n", ret, high_data, low_data);

    return _data;
}