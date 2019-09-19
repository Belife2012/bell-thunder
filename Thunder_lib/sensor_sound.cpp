#include "sensor_sound.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief 获取声音的相对强度值
 * 
 * @param sensorChannel 
 * @return unsigned char 声音的相对强度值（0~100）
 */
unsigned char SENSOR_SOUND::GetSoundDB(unsigned char sensorChannel)
{
    unsigned char read_data = 0;
    unsigned char ret;

    ret = read(SOUND_IIC_REG_DB_ADDR, &read_data, 1, sensorChannel);
    if(ret != 0) {
        log_e("read sound error");
        return 0;
    }

    return read_data;
}

/**
 * @brief 设置声音强度检测的最大值和最小值
 * 
 * @param max_value 最大值，取默认最大值的 max_value% 为最大值
 * @param min_value 最小值，取默认最大值的 min_value% 为最小值
 * @param sensorChannel 
 */
void SENSOR_SOUND::SetDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
    unsigned char ret;

    ret = write(SOUND_IIC_REG_SETMAX_ADDR, &max_value, 1, sensorChannel);
    if(ret != 0) {
        log_e("set sound max error");
        return;
    }
    ret = write(SOUND_IIC_REG_SETMIN_ADDR, &min_value, 1, sensorChannel);
    if(ret != 0) {
        log_e("set sound min error");
        return;
    }

    return;
}