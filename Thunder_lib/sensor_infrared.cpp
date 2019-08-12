#include "sensor_infrared.h"

uint8_t SENSOR_INFRARED::Check_Out_IRdata(uint16_t data)
{
    uint8_t result = 0x0f;

    if( 0x0000 == (data & 0x0f00) ) // 信标模式
    {
        result ^= (uint8_t)(data >> 4);
        result ^= (uint8_t)(data >> 8);
        result ^= (uint8_t)(data >> 12);

        result <<= 1;
        if( (result & 0x0f) != ((uint8_t)data & 0x0f) ){
            return 0;
        }
    }else {
        result ^= (uint8_t)(data >> 4);
        result ^= (uint8_t)(data >> 8);
        result ^= (uint8_t)(data >> 12);

        if( (result & 0x0f) != ((uint8_t)data & 0x0f) ){
            return 0;
        }
    }

    return 1;
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 设置红外传感器的工作频道
 * 
 * @param sys_channel:工作频道（1~4）
 * @param sensorChannel:传感器接口编号
 */
void SENSOR_INFRARED::SetSysChannel(unsigned char sys_channel, uint8_t sensorChannel)
{
    CHECK_RANGE(sys_channel, 1, 4);
    sys_channel -= 1;

    write(INFRARED_IIC_REG_SYSCHANNEL, &sys_channel, 1, sensorChannel);
}

/**
 * @brief: 设置红外传感器的工作模式
 * 
 * @param sys_mode: 工作模式 0/1/2
 * @param sensorChannel: 传感器接口编号
 */
void SENSOR_INFRARED::SetSysMode(unsigned char sys_mode, uint8_t sensorChannel)
{
    CHECK_RANGE(sys_mode, 0 ,2);
    write(INFRARED_IIC_REG_SYSMODE, &sys_mode, 1, sensorChannel);
}

/**
 * @brief: 获取红外传感器检测到的障碍物距离
 * 
 * @param sensorChannel:传感器接口编号
 * @return unsigned char :障碍物距离（相对距离）
 */
unsigned char SENSOR_INFRARED::GetDistance(uint8_t sensorChannel)
{
    uint8_t read_data[2] = {0x0,0x0};
    uint8_t ret;

    ret = read(INFRARED_IIC_REG_DISTANCE, read_data, 2, sensorChannel);
    CHECK_IIC_RETURN(ret);
    
    return read_data[0];
}

/**
 * @brief: 获取检测到的信标方向
 * 
 * @param sensorChannel:传感器接口编号
 * @return unsigned char : 信标方向
 */
unsigned char SENSOR_INFRARED::GetBeaconDire(uint8_t sensorChannel)
{
    uint8_t read_data[2] = {0x0,0x0};
    uint8_t ret_value = 0;
    uint8_t ret;

    ret = read(INFRARED_IIC_REG_BEACON, read_data, 2, sensorChannel);
    CHECK_IIC_RETURN(ret);

	ret_value = read_data[0];

    return ret_value;
}

/**
 * @brief: 获取检测到的信标距离
 * 
 * @param sensorChannel: 传感器接口编号
 * @return unsigned char : 信标距离
 */
unsigned char SENSOR_INFRARED::GetBeaconDist(uint8_t sensorChannel)
{
    uint8_t read_data[2] = {0x0,0x0};
    uint8_t ret_value = 0;
    uint8_t ret;

    ret = read(INFRARED_IIC_REG_BEACON, read_data, 2, sensorChannel);
    CHECK_IIC_RETURN(ret);

	ret_value = read_data[1];

    return ret_value;
}

/**
 * @brief: 获取检测到的信标遥控数据
 * 
 * @param sensorChannel: 传感器接口编号
 * @return unsigned char : 遥控数据
 */
unsigned char SENSOR_INFRARED::GetRemoteInfo(uint8_t sensorChannel)
{
    uint8_t read_data[2] = {0x0,0x0};
    uint16_t remote_data;
    uint8_t ret_value = 0;
    uint8_t ret;

    ret = read(INFRARED_IIC_REG_REMOTE, read_data, 2, sensorChannel);
    CHECK_IIC_RETURN(ret);

    remote_data = read_data[1];
    remote_data <<= 8;
    remote_data |= read_data[0];

    // 校验遥控器数据是否正确
    ret_value = Check_Out_IRdata(remote_data);
    if(ret_value == 0) {
        return 0;
    }

	ret_value = (read_data[0]) >> 4;
    switch(ret_value) {
        case 0x01: ret_value = 1; break;
        case 0x02: ret_value = 2; break;
        case 0x04: ret_value = 3; break;
        case 0x08: ret_value = 4; break;
        case 0x03: ret_value = 5; break;
        case 0x05: ret_value = 6; break;
        case 0x09: ret_value = 7; break;
        case 0x06: ret_value = 8; break;
        case 0x0A: ret_value = 9; break;
        case 0x0C: ret_value = 10; break;
        case 0x07: ret_value = 11; break;
        case 0x0B: ret_value = 12; break;
        case 0x0D: ret_value = 13; break;
        case 0x0E: ret_value = 14; break;
        case 0x0F: ret_value = 15; break;
        default: ret_value = 0; break;
    }

    if(read_data[1] & 0x40) {
        ret_value = 16;
    }

    return ret_value;
}