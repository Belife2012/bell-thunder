#include "sensor_gas.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 获取传感器检测到的气体浓度
 * 
 * @param sensorChannel: 传感器接口编号
 * @return unsigned char : 气体浓度（0~100）
 */
unsigned char SENSOR_GAS::GetToxicgasRatio(unsigned char sensorChannel)
{
    unsigned char read_data = 0;
    unsigned char ret;

    ret = read(GAS_IIC_REG_RATIO_ADDR, &read_data, 1, sensorChannel);
    CHECK_IIC_RETURN(ret);

    return read_data;
}

/**
 * @brief: 设置传感器检测的最大值和最小值
 * 
 * @param max_value 最大值，取默认最大值的 max_value% 为最大值
 * @param min_value 最小值，取默认最大值的 min_value% 为最小值
 * @param sensorChannel: 传感器接口编号
 */
void SENSOR_GAS::SetDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
    unsigned char ret;

    ret = write(GAS_IIC_REG_SETMAX_ADDR, &max_value, 1, sensorChannel);
    CHECK_IIC_RETURN(ret);
    ret = write(GAS_IIC_REG_SETMIN_ADDR, &min_value, 1, sensorChannel);
    CHECK_IIC_RETURN(ret);

    return;
}