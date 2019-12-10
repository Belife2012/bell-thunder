#include <Arduino.h>
#include <Wire.h>
#include "sensor_colorlight.h"
#include "byte_convertion.h"

SENSOR_COLORLIGHT::SENSOR_COLORLIGHT(int slave_address) : SENSOR_IIC(slave_address)
{
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/
#define MAX_VALUE_RED       2100      
#define MAX_VALUE_GREEN     2300
#define MAX_VALUE_BLUE      2200
#define MAX_VALUE_CLEAR     2100
/**
 * @brief: 获取检测结果，可以获取环境光模式的检测结果、反射光模式的检测结果（红、绿、蓝分量），
 * 颜色模式的检测结果，还可以获取颜色模式的数据: HSV数据和RGB数据；
 * 
 * @param result_index: 
 * 0|环境光，1|反射红光分量，2|反射绿光分量，3|反射蓝光分量，
 * 4|颜色（颜色结果参考SENSOR_COLORLIGHT::enum_Color 0~8分别代表 无颜色/黑/白/红/黄/绿/青/蓝/紫）
 * 10|H值，11|S值，12|V值，13|R值，14|G值，15|B值
 * @param channel:传感器接口编号
 * @return int :返回的检测数据结果
 */
int SENSOR_COLORLIGHT::Get_Result(unsigned char result_index, unsigned char channel)
{
    int getValue = 0;
    byte bakCode = 0;
    byte read_data[2];

    switch (result_index)
    {
    case RESULT_ENV:
        bakCode = read(CLINE_IIC_REG_ENV, &read_data[0], 1, channel);
        FILTER_RANGE(getValue, read_data[0], 0, 100);
        break;
    case RESULT_REFLECT_R:
        bakCode = read(CLINE_IIC_REG_REFLECT_R, &read_data[0], 1, channel);
        FILTER_RANGE(getValue, read_data[0], 0, 100);
        break;
    case RESULT_REFLECT_G:
        bakCode = read(CLINE_IIC_REG_REFLECT_G, &read_data[0], 1, channel);
        FILTER_RANGE(getValue, read_data[0], 0, 100);
        break;
    case RESULT_REFLECT_B:
        bakCode = read(CLINE_IIC_REG_REFLECT_B, &read_data[0], 1, channel);
        FILTER_RANGE(getValue, read_data[0], 0, 100);
        break;
    case RESULT_COLOR:
        bakCode = read(CLINE_IIC_REG_COLOR, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case DATA_HSV_H:
        bakCode = read(CLINE_IIC_REG_HSV_H, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        // CHECK_RANGE(getValue, 0, 360);
        // getValue = getValue * 100 / 360;
        break;
    case DATA_HSV_S:
        bakCode = read(CLINE_IIC_REG_HSV_S, &read_data[0], 1, channel);
        getValue = read_data[0];
        break;
    case DATA_HSV_V:
        bakCode = read(CLINE_IIC_REG_HSV_V, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        CHECK_RANGE(getValue, 0, MAX_VALUE_CLEAR);
        getValue = getValue * 100 / MAX_VALUE_CLEAR;
        break;
    case DATA_COLOR_R:
        bakCode = read(CLINE_IIC_REG_COLOR_R, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        // Serial.printf("%d ", getValue);
        CHECK_RANGE(getValue, 0, MAX_VALUE_RED);
        getValue = getValue * 255 / MAX_VALUE_RED;
        break;
    case DATA_COLOR_G:
        bakCode = read(CLINE_IIC_REG_COLOR_G, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        // Serial.printf("%d ", getValue);
        CHECK_RANGE(getValue, 0, MAX_VALUE_GREEN);
        getValue = getValue * 255 / MAX_VALUE_GREEN;
        break;
    case DATA_COLOR_B:
        bakCode = read(CLINE_IIC_REG_COLOR_B, &read_data[0], 2, channel);
        getValue = read_data[1];
        getValue = (getValue << 8) + read_data[0];
        // Serial.printf("%d ", getValue);
        CHECK_RANGE(getValue, 0, MAX_VALUE_BLUE);
        getValue = getValue * 255 / MAX_VALUE_BLUE;
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
 * @brief: 校准传感器的最大值或最小值
 * 
 * @param mode: 0 设置值为反射光最大值，1 设置值为反射光最小值, 2 重置反射光设置值
 *              3 设置值为环境光最大值，4 设置值为环境光最小值, 5 重置环境光设置值
 * @param value: 新设置的数值（0~100）
 * @param sensorChannel: 传感器接口编号
 */
void SENSOR_COLORLIGHT::Set_Extremum(int mode, float value, uint8_t channel)
{
    unsigned char new_value;
    CHECK_RANGE(value, 0, 100);

    new_value = (unsigned char)value;
    if (mode == 0)
    {
        write(CLINE_IIC_REG_REFLECT_MAX, &new_value, 1, channel);
    }
    else if (mode == 1)
    {
        write(CLINE_IIC_REG_REFLECT_MIN, &new_value, 1, channel);
    }
    else if (mode == 2)
    {
        new_value = 100;
        write(CLINE_IIC_REG_REFLECT_MAX, &new_value, 1, channel);
        new_value = 0;
        write(CLINE_IIC_REG_REFLECT_MAX, &new_value, 1, channel);
    }else if (mode == 3)
    {
        write(CLINE_IIC_REG_ENV_MAX, &new_value, 1, channel);
    }
    else if (mode == 4)
    {
        write(CLINE_IIC_REG_ENV_MIN, &new_value, 1, channel);
    }
    else if (mode == 5)
    {
        new_value = 100;
        write(CLINE_IIC_REG_ENV_MAX, &new_value, 1, channel);
        new_value = 0;
        write(CLINE_IIC_REG_ENV_MAX, &new_value, 1, channel);
    }
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
 * @brief 设置反射光模式检测的范围
 * 
 * @param max_value 最大值
 * @param min_value 最小值
 * @param sensorChannel 
 */
void SENSOR_COLORLIGHT::SetReflectDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
    CHECK_RANGE(max_value, 0, 100);
    CHECK_RANGE(min_value, 0, 100);

    write(CLINE_IIC_REG_ENV_MAX, &max_value, 1, sensorChannel);
    write(CLINE_IIC_REG_ENV_MIN, &min_value, 1, sensorChannel);
}
/**
 * @brief 设置环境光模式检测的范围
 * 
 * @param max_value 最大值
 * @param min_value 最小值
 * @param sensorChannel 
 */
void SENSOR_COLORLIGHT::SetEnvDetectRange(unsigned char max_value, unsigned char min_value, unsigned char sensorChannel)
{
    CHECK_RANGE(max_value, 0, 100);
    CHECK_RANGE(min_value, 0, 100);

    write(CLINE_IIC_REG_REFLECT_MAX, &max_value, 1, sensorChannel);
    write(CLINE_IIC_REG_REFLECT_MIN, &min_value, 1, sensorChannel);
}

/**
 * @brief: 复位传感器的反射光最大最小值设置，环境光的最大最小值设置
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
    reg_data = 100;
    write(CLINE_IIC_REG_ENV_MAX, &reg_data, 1, channel);
    reg_data = 0;
    write(CLINE_IIC_REG_ENV_MAX, &reg_data, 1, channel);
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

void SENSOR_COLORLIGHT::Read_RGB_Scale(float *RGB_scale, unsigned char channel)
{
    byte read_buf[4];
    BYTE_CONVERTION _data;

    read(CLINE_IIC_REG_SCALE_R, read_buf, 4, channel);
    read_buf[3] = read_buf[2];
    read_buf[2] = read_buf[1];
    read_buf[1] = read_buf[0];
    read_buf[0] = 0;
    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &RGB_scale[0]);
    // Serial.printf("\niic: %d, %d, %d, %d, %f", read_buf[0],read_buf[1],read_buf[2],read_buf[3],RGB_scale[0]);

    read(CLINE_IIC_REG_SCALE_G, read_buf, 4, channel);
    read_buf[3] = read_buf[2];
    read_buf[2] = read_buf[1];
    read_buf[1] = read_buf[0];
    read_buf[0] = 0;
    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &RGB_scale[1]);
    // Serial.printf("\niic: %d, %d, %d, %d, %f", read_buf[0],read_buf[1],read_buf[2],read_buf[3],RGB_scale[1]);

    read(CLINE_IIC_REG_SCALE_B, read_buf, 4, channel);
    read_buf[3] = read_buf[2];
    read_buf[2] = read_buf[1];
    read_buf[1] = read_buf[0];
    read_buf[0] = 0;
    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &RGB_scale[2]);
    // Serial.printf("\niic: %d, %d, %d, %d, %f", read_buf[0],read_buf[1],read_buf[2],read_buf[3],RGB_scale[2]);

}

byte SENSOR_COLORLIGHT::Calibrate_Sensor(unsigned char channel)
{
    byte calibrate_flag, ret;

    float result_old[3];
    float result[3];

    Set_Operate_Mode(MODE_COLOR, channel);
    delay(100);
    Read_RGB_Scale(result_old, channel);

	Serial.println("calibrate color light");
    calibrate_flag = 0x01;
    ret = write(CLINE_IIC_REG_CALIBRATE, &calibrate_flag, 1, channel);
    if(ret != 0) 
    {
        Serial.println("Calibrate Fail");
        return ret;
    }
    delay(500);

    for(;;)
    {
        delay(100);
        Read_RGB_Scale(result, channel);
        if(result[0] != result_old[0] || result[1] != result_old[1] || result[2] != result_old[2]) {
            break;
        }
    }

    delay(100);
    Read_RGB_Scale(result, channel);
    if( (result[0] == 1.0 && result[1] == 1.0 && result[2] == 1.0)
      || result[0] > 1.4 || result[1] > 1.4 || result[2] > 1.4
      || result[0] < 0.6 || result[1] < 0.6 || result[2] < 0.6 ) {
          Serial.printf("Calibrate Error: %3.2f %3.2f %3.2f\n", result[0], result[1], result[2]);
          return 0xff;
    }
	Serial.printf("Calibrate Success: %3.2f %3.2f %3.2f\n", result[0], result[1], result[2]);
    
    return ret;
}