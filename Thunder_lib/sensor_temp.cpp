#include "sensor_temp.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief 获取温度
 * 
 * @param sensorChannel 
 * @return float 温度值
 */
float SENSOR_TEMP::GetTemperature(uint8_t sensorChannel)
{
    uint8_t low_data = 0, high_data = 0;
    uint16_t read_value = 0;
    float ret_value;
    uint8_t ret;

    ret = read(TEMP_IIC_REG_LOWD_ADDR, &low_data, 1, sensorChannel);
    if(ret != 0) {
        log_e("read temp error");
        return 0;
    }
    ret = read(TEMP_IIC_REG_HIGHD_ADDR, &high_data, 1, sensorChannel);
    if(ret != 0) {
        log_e("read temp error");
        return 0;
    }

    // Serial.printf("GetTemperature: %d data: %d %d\n", ret, high_data, low_data);

	read_value = (read_value | high_data) << 8;
	read_value = (read_value | low_data);
    
	ret_value = (float)read_value / 16;

    return ret_value;
}