#include "sensor_human.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 获取人体红外变化，如果有人体红外变化，返回1， 否则返回0
 * 
 * @param sensorChannel: 传感器接口编号
 * @return unsigned char : 有人体红外变化，返回1， 否则返回0
 */
unsigned char SENSOR_HUMAN::GetStatus(uint8_t sensorChannel)
{
    uint8_t _data = 1;
    uint8_t ret;

    ret = read(HUMAN_IIC_REG_STATUS_ADDR, &_data, 1, sensorChannel);
    CHECK_IIC_RETURN(ret);

    _data = (_data==1) ? 0 : 1;
    
    return _data;
}