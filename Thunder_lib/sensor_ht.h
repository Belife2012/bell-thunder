#ifndef __SENSOR_HT_H__
#define __SENSOR_HT_H__
#include "iic_thunder.h"

#define HT_IIC_ADDR         (0x05)
#define HT_IIC_REG_HUM_ADDR      (0x01)
#define HT_IIC_REG_TEMP_ADDR     (0x02)

class SENSOR_HT : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_HT(int slave_address);

    /*--------------Thunder IDE APIs: -------------*/
    float GetHumidity(uint8_t sensorChannel=0);
    float GetTemperature(uint8_t sensorChannel=0);
};

#endif
