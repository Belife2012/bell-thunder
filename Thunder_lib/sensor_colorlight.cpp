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
 * @brief: 获取检测结果，可以获取环境光检测结果、反射光检测结果（红、绿、蓝分量），颜色检测结果；
 * 还可以获取颜色模式的数据: HSV数据和RGB数据
 * 
 * @param result_index: 
 * 0|环境光，1|反射红光分量，2|反射绿光分量，3|反射蓝光分量，4|颜色（颜色结果参考SENSOR_COLORLIGHT::enum_Color）
 * 10|H值，11|S值，12|V值，13|R值，14|G值，15|B值
 * @param channel:传感器接口编号
 * @return int :返回的检测数据结果
 */
int SENSOR_COLORLIGHT::Get_Result(unsigned char result_index, unsigned char channel)
{
    int getValue=0;
    byte bakCode=0;
    byte read_data[2];

    switch (result_index)
    {
    case RESULT_ENV:
        bakCode = read(CLINE_IIC_REG_ENV, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case RESULT_REFLECT_R:
        bakCode = read(CLINE_IIC_REG_REFLECT_R, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case RESULT_REFLECT_G:
        bakCode = read(CLINE_IIC_REG_REFLECT_G, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case RESULT_REFLECT_B:
        bakCode = read(CLINE_IIC_REG_REFLECT_B, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case RESULT_COLOR:
        bakCode = read(CLINE_IIC_REG_COLOR, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case DATA_HSV_H:
        bakCode |= read(CLINE_IIC_REG_HSV_H, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        break;
    case DATA_HSV_S:
        bakCode = read(CLINE_IIC_REG_HSV_S, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case DATA_HSV_V:
        bakCode |= read(CLINE_IIC_REG_HSV_V, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        CHECK_RANGE(getValue, 0, 1800);
        getValue = getValue * 100 / 1800;
        break;
    case DATA_COLOR_R:
        bakCode |= read(CLINE_IIC_REG_COLOR_R, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        CHECK_RANGE(getValue, 0, 1800);
        getValue = getValue * 100 / 1800;
        break;
    case DATA_COLOR_G:
        bakCode |= read(CLINE_IIC_REG_COLOR_G, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        CHECK_RANGE(getValue, 0, 1800);
        getValue = getValue * 100 / 1800;
        break;
    case DATA_COLOR_B:
        bakCode |= read(CLINE_IIC_REG_COLOR_B, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        CHECK_RANGE(getValue, 0, 1800);
        getValue = getValue * 100 / 1800;
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
 * @brief: 设置工作模式为反射模式，并设置反射模式的灯光
 * 
 * @param optData:设置反射模式的灯光，相应的bit=1表示灯亮，否则为灯灭，bit0|红，bit1|绿，bit2|蓝
 * 0x01|红亮，绿灭，蓝灭
 * 0x03|红亮，绿亮，蓝灭
 * 0x05|红亮，绿灭，蓝亮
 * 0x07|红亮，绿亮，蓝亮
 * 0x02|红灭，绿亮，蓝灭
 * 0x06|红灭，绿亮，蓝亮
 * 0x04|红灭，绿灭，蓝亮
 * @param channel:传感器接口编号
 * @return byte :返回0表示成功，否则失败
 */
byte SENSOR_COLORLIGHT::Set_Reflect_Led(byte optData, unsigned char channel)
{
    byte ret;
    byte optMode;

    optMode = MODE_REFLECT;
    optMode |= (optData << 4);
    ret = write(CLINE_IIC_REG_SETTING, &optMode, 1, channel);

    return ret;
}
