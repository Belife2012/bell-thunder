#include "sensor_soil.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief 获取土壤湿度相对值
 * 
 * @param sensorChannel 
 * @return unsigned char 土壤湿度相对值（0~100）
 */
unsigned char SENSOR_SOIL::GetHumidity(unsigned char sensorChannel)
{
    unsigned char read_data = 0;
    unsigned char ret;

    ret = read(SOIL_IIC_REG_HUM_ADDR, &read_data, 1, sensorChannel);
    if(ret != 0) {
        log_e("read humidity of soil error");
        return 0;
    }

    return read_data;
}

/**
 * @brief 设置土壤湿度检测的最大值和最小值
 * 
 * @param max_value 最大值，取默认最大值的 max_value% 为最大值
 * @param min_value 最小值，取默认最大值的 min_value% 为最小值
 * @param sensorChannel 
 */
void SENSOR_SOIL::SetDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
    unsigned char ret;

    ret = write(SOIL_IIC_REG_SETMAX_ADDR, &max_value, 1, sensorChannel);
    if(ret != 0) {
        log_e("set soid max error");
        return;
    }
    ret = write(SOIL_IIC_REG_SETMIN_ADDR, &min_value, 1, sensorChannel);
    if(ret != 0) {
        log_e("set soil min error");
        return;
    }

    return;
}