#include <Arduino.h>
#include <Wire.h>
#include "sensor_colorlight.h"

SENSOR_COLORLIGHT::SENSOR_COLORLIGHT(int slave_address) : SENSOR_IIC(slave_address)
{
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

int SENSOR_COLORLIGHT::Get_Result(unsigned char result_index, unsigned char channel)
{
    byte getValue=0, bakCode=0;

    switch (result_index)
    {
    case RESULT_ENV:
        bakCode = read(IIC_REG_ADDR_ENV, &getValue, 1, channel);
        break;
    case RESULT_REFLECT_R:
        bakCode = read(IIC_REG_ADDR_REFLECT_R, &getValue, 1, channel);
        break;
    case RESULT_REFLECT_G:
        bakCode = read(IIC_REG_ADDR_REFLECT_G, &getValue, 1, channel);
        break;
    case RESULT_REFLECT_B:
        bakCode = read(IIC_REG_ADDR_REFLECT_B, &getValue, 1, channel);
        break;
    case RESULT_COLOR:
        bakCode = read(IIC_REG_ADDR_COLOR, &getValue, 1, channel);
        break;

    default:
        break;
    }

    if (bakCode != 0)
    {
        return 0;
    }

    return getValue;
}

/**
 * @brief 设置光线检测的最大值和最小值
 * 
 * @param max_value 最大值
 * @param min_value 最小值
 * @param sensorChannel 
 */
void SENSOR_COLORLIGHT::SetDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
    CHECK_RANGE(max_value, 0, 100);
    CHECK_RANGE(min_value, 0, 100);

    dark_value[sensorChannel] = min_value;
    bright_value[sensorChannel] = max_value;
}

/**
 * @brief: 复位光电传感器的设置
 * 
 * @param channel:传感器接口编号 
 */
void SENSOR_COLORLIGHT::Reset(unsigned char channel)
{
    dark_value[channel] = 0;
    bright_value[channel] = 100;
}

byte SENSOR_COLORLIGHT::Set_Operate_Mode(byte optMode, unsigned char channel)
{
    byte ret;
    CHECK_RANGE(optMode, 0, MODE_COLOR);
    ret = write(IIC_REG_ADDR_SETTING, &optMode, 1, channel);

    return ret;
}

byte SENSOR_COLORLIGHT::Set_Operate_Modes(byte optMode, byte optData, unsigned char channel)
{
    byte ret;

    if (optMode > MODE_COLOR)
    {
        optMode = MODE_COLOR;
    }

    optMode |= (optData << 4);
    ret = write(IIC_REG_ADDR_SETTING, &optMode, 1, channel);

    return ret;
}
