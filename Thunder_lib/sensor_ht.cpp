#include "sensor_ht.h"

SENSOR_HT::SENSOR_HT(int slave_address) : SENSOR_IIC(slave_address)
{ }

float SENSOR_HT::GetHumidity(uint8_t sensorChannel)
{
    uint8_t read_data[3] = {0x0,0x0,0x0};
    uint32_t read_value = 0;
    float ret_value;
    uint8_t ret,times;

    for(times=0; times < 5; times++) {
        ret = read(HT_IIC_REG_HUM_ADDR, read_data, 3, sensorChannel);
        if(ret == 0) break;
    }
    
	read_value = (read_value | read_data[0]) << 8;
	read_value = (read_value | read_data[1]) << 8;
	read_value = (read_value | read_data[2]);
	read_value = read_value >> 4;
    
    // if(read_data[0] == 0){
    // Serial.printf("%2x ", read_data[0]);
    // Serial.printf("%2x ", read_data[1]);
    // Serial.printf("%2x ", read_data[2]);
    // }
	ret_value = (double)read_value / 0xfffff * 100;

    return ret_value;
}

float SENSOR_HT::GetTemperature(uint8_t sensorChannel)
{
    uint8_t read_data[3] = {0x0,0x0,0x0};
    uint32_t read_value = 0;
    float ret_value;
    uint8_t ret,times;

    for(times=0; times < 5; times++) {
        ret = read(HT_IIC_REG_TEMP_ADDR, read_data, 3, sensorChannel);
        if(ret == 0) break;
    }

	read_value = (read_value | read_data[0]) << 8;
	read_value = (read_value | read_data[1]) << 8;
	read_value = (read_value | read_data[2]);
	read_value = read_value & 0xfffff;
    
    // if(read_data[0] == 0){
    // Serial.printf("%2x ", read_data[0]);
    // Serial.printf("%2x ", read_data[1]);
    // Serial.printf("%2x ", read_data[2]);
    // }
	ret_value = (double)read_value / 0xfffff * 200 - 50;

    return ret_value;
}
