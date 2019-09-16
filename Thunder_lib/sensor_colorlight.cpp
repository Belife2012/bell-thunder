#include <Arduino.h>
#include <Wire.h>
#include "sensor_colorlight.h"

SENSOR_COLORLIGHT::SENSOR_COLORLIGHT(int slave_address) : SENSOR_IIC(slave_address)
{
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 获取颜色模式的数据HSV的数据和RGB的数据
 * 
 * @param data_index: 数据的编号，参考SENSOR_COLORLIGHT::enum_Data_Index
 * @param channel: 传感器接口编号
 * @return int :返回数据结果
 */
int SENSOR_COLORLIGHT::Get_Data(unsigned char data_index, unsigned char channel)
{
    int getValue=0;
    byte bakCode=0;
    byte read_data[2];

    switch (data_index)
    {
    case DATA_HSV_H:
        bakCode = read(CLINE_IIC_REG_HSV_H_L, &read_data[0], 1, channel);
        bakCode |= read(CLINE_IIC_REG_HSV_H_H, &read_data[1], 1, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        break;
    case DATA_HSV_S:
        bakCode = read(CLINE_IIC_REG_HSV_S, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case DATA_HSV_V:
        bakCode = read(CLINE_IIC_REG_HSV_V_L, &read_data[0], 1, channel);
        bakCode |= read(CLINE_IIC_REG_HSV_V_H, &read_data[1], 1, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        break;
    case DATA_COLOR_R:
        bakCode = read(CLINE_IIC_REG_COLOR_R_L, &read_data[0], 1, channel);
        bakCode |= read(CLINE_IIC_REG_COLOR_R_H, &read_data[1], 1, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        break;
    case DATA_COLOR_G:
        bakCode = read(CLINE_IIC_REG_COLOR_G_L, &read_data[0], 1, channel);
        bakCode |= read(CLINE_IIC_REG_COLOR_G_H, &read_data[1], 1, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        break;
    case DATA_COLOR_B:
        bakCode = read(CLINE_IIC_REG_COLOR_B_L, &read_data[0], 1, channel);
        bakCode |= read(CLINE_IIC_REG_COLOR_B_H, &read_data[1], 1, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        break;

    default:
        break;
    }

    if (bakCode != 0)
    {
        return 0;
    }
    // Serial.printf(" 0x%02x 0x%02x ", read_data[1], read_data[0]);
    return getValue;
}

/**
 * @brief: 获取检测结果，可以获取环境光检测结果、反射光检测结果（红、绿、蓝分量），颜色检测结果
 * 
 * @param result_index: 0|环境光，1|反射光红分量，2|反射光绿分量，3|反射光蓝分量，
 *                      4|颜色（结果参考SENSOR_COLORLIGHT::enum_Color）
 * @param channel:传感器接口编号
 * @return int :返回的检测数据结果
 */
int SENSOR_COLORLIGHT::Get_Result(unsigned char result_index, unsigned char channel)
{
    byte getValue=0, bakCode=0;

    switch (result_index)
    {
    case RESULT_ENV:
        bakCode = read(CLINE_IIC_REG_ENV, &getValue, 1, channel);
        break;
    case RESULT_REFLECT_R:
        bakCode = read(CLINE_IIC_REG_REFLECT_R, &getValue, 1, channel);
        break;
    case RESULT_REFLECT_G:
        bakCode = read(CLINE_IIC_REG_REFLECT_G, &getValue, 1, channel);
        break;
    case RESULT_REFLECT_B:
        bakCode = read(CLINE_IIC_REG_REFLECT_B, &getValue, 1, channel);
        break;
    case RESULT_COLOR:
        bakCode = read(CLINE_IIC_REG_COLOR, &getValue, 1, channel);
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
 * @brief 设置反射模式检测的范围
 * 
 * @param max_value 最大值
 * @param min_value 最小值
 * @param sensorChannel 
 */
void SENSOR_COLORLIGHT::SetDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
    CHECK_RANGE(max_value, 0, 100);
    CHECK_RANGE(min_value, 0, 100);

    write(CLINE_IIC_REG_REFLECT_MAX, &max_value, 1, sensorChannel);
    write(CLINE_IIC_REG_REFLECT_MIN, &min_value, 1, sensorChannel);
}

/**
 * @brief: 复位反射模式的范围设置
 * 
 * @param channel:传感器接口编号 
 */
void SENSOR_COLORLIGHT::Reset(unsigned char channel)
{
    byte reg_data;
    reg_data = 100;
    write(CLINE_IIC_REG_REFLECT_MAX, &reg_data, 1, channel);
    reg_data = 0;
    write(CLINE_IIC_REG_REFLECT_MIN, &reg_data, 1, channel);
}

/**
 * @brief: 设置工作模式
 * 
 * @param optMode:设置工作模式，SENSOR_COLORLIGHT::enum_Mode，0|环境光模式，1|反射模式，2|颜色模式
 * @param channel:传感器接口编号
 * @return byte :返回0表示成功，否则失败
 */
byte SENSOR_COLORLIGHT::Set_Operate_Mode(byte optMode, unsigned char channel)
{
    byte ret;
    CHECK_RANGE(optMode, 0, MODE_COLOR);
    ret = write(CLINE_IIC_REG_SETTING, &optMode, 1, channel);

    return ret;
}

/**
 * @brief: 设置工作模式，包含设置反射模式的灯光
 * 
 * @param optMode:设置工作模式，SENSOR_COLORLIGHT::enum_Mode，0|环境光模式，1|反射模式，2|颜色模式
 * @param optData:设置反射模式的灯光，相应的bit=1表示灯亮，否则为灯灭，bit0|红，bit1|绿，bit2|蓝
 * @param channel:传感器接口编号
 * @return byte :返回0表示成功，否则失败
 */
byte SENSOR_COLORLIGHT::Set_Operate_Modes(byte optMode, byte optData, unsigned char channel)
{
    byte ret;

    if (optMode > MODE_COLOR)
    {
        optMode = MODE_COLOR;
    }

    optMode |= (optData << 4);
    ret = write(CLINE_IIC_REG_SETTING, &optMode, 1, channel);

    return ret;
}
