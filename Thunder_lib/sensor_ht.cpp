#include "sensor_ht.h"

SENSOR_HT::SENSOR_HT(int slave_address) : SENSOR_IIC(slave_address)
{ }

float SENSOR_HT::ReadHumidity(uint8_t sensorChannel)
{
    uint8_t read_data[5] = {0xff,0xff,0xff,0xff,0xff};
    uint32_t read_value = 0;
    float ret_value;
    uint8_t ret;

    ret = read(HT_IIC_REG_ADDR, read_data, 2);
    
	read_value = (read_value | read_data[0]) << 8;
	read_value = (read_value | read_data[1]) << 8;
	read_value = (read_value | read_data[2]);
	read_value = read_value >> 4;
    
    Serial.printf(" %d ", ret);
    Serial.printf(" %d ", read_value);
	ret_value = (double)read_value / 0xfffff * 100;

    return ret_value;
}

float SENSOR_HT::ReadTemperature(uint8_t sensorChannel)
{
    uint8_t read_data[5];
    uint32_t read_value = 0;
    float ret_value;
    uint8_t ret;

    ret = read(HT_IIC_REG_ADDR, read_data, 1);

	read_value = (read_value | read_data[2]) << 8;
	read_value = (read_value | read_data[3]) << 8;
	read_value = (read_value | read_data[4]);
	read_value = read_value & 0xfffff;
    
    Serial.printf(" %d ", ret);
    Serial.printf(" %d ", read_value);
	ret_value = (double)read_value / 0xfffff * 200 - 50;

    return ret_value;
}
