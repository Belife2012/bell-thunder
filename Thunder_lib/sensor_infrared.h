#ifndef __SENSOR_INFRARED_H__
#define __SENSOR_INFRARED_H__
#include "Sensor_IIC.h"

#define INFRARED_IIC_ADDR                   (0x04)
#define INFRARED_IIC_REG_SYSCHANNEL         (0x10)
#define INFRARED_IIC_REG_SYSMODE            (0x20)
#define INFRARED_IIC_REG_DISTANCE           (0x01)
#define INFRARED_IIC_REG_BEACON             (0x02)
#define INFRARED_IIC_REG_REMOTE             (0x03)

typedef enum {
    SYS_MODE_REMOTE,
    SYS_MODE_BEACON,
    SYS_MODE_DISTANCE
} SysMode_TypeEnum;

class SENSOR_INFRARED : public SENSOR_IIC
{
private:
    /* data */
public:
    SENSOR_INFRARED(int slave_address) : SENSOR_IIC(slave_address) {};

    void SetSysMode(uint8_t sensorChannel, unsigned char sys_mode);
    void SetSysChannel(uint8_t sensorChannel, unsigned char sys_channel);

    unsigned char GetDistance(uint8_t sensorChannel);
    unsigned char GetBeaconDist(uint8_t sensorChannel);
    unsigned char GetBeaconDire(uint8_t sensorChannel);
    unsigned char GetRemoteInfo(uint8_t sensorChannel);
    unsigned char Check_Out_IRdata(uint16_t data);
};

#endif
