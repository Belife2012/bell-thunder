#include "iic_thunder.h"

#ifndef _COLORLIGHT_I2C_
#define _COLORLIGHT_I2C_

// 光电模块
#define COLORLIGHT_IIC_ADDR 0x0B //光电模块I2C器件地址

#define IIC_REG_ADDR_SETTING 0x01 // 设置工作模式和反射模式的灯光, bit0~3: 工作模式；bit4~7: 反射模式的RGB灯数据
#define IIC_REG_ADDR_ENV 0x02
#define IIC_REG_ADDR_REFLECT_R 0x03
#define IIC_REG_ADDR_REFLECT_G 0x04
#define IIC_REG_ADDR_REFLECT_B 0x05
#define IIC_REG_ADDR_COLOR 0x06
#define IIC_REG_ADDR_HSV_H_L 0x07
#define IIC_REG_ADDR_HSV_H_H 0x08
#define IIC_REG_ADDR_HSV_S 0x09
#define IIC_REG_ADDR_COLOR_R_L 0x0A
#define IIC_REG_ADDR_COLOR_R_H 0x0B
#define IIC_REG_ADDR_COLOR_G_L 0x0C
#define IIC_REG_ADDR_COLOR_G_H 0x0D
#define IIC_REG_ADDR_COLOR_B_L 0x0E
#define IIC_REG_ADDR_COLOR_B_H 0x0F

class SENSOR_COLORLIGHT : public SENSOR_IIC
{
private:
    float dark_value[6] = {0, 0, 0, 0, 0, 0};
    float bright_value[6] = {100, 100, 100, 100, 100, 100};

public:
    typedef enum
    {
        MODE_ENV,
        MODE_REFLECT,
        MODE_COLOR
    } enum_Mode;
    typedef enum
    {
        RESULT_ENV,
        RESULT_REFLECT_R,
        RESULT_REFLECT_G,
        RESULT_REFLECT_B,
        RESULT_COLOR
    } enum_Result_Index;
    typedef enum {
        COLOR_NON ,
        COLOR_BLACK ,
        COLOR_WHITE ,
        COLOR_RED ,
        COLOR_ORANGE ,
        COLOR_YELLOWISH ,
        COLOR_YELLOW ,
        COLOR_GRASS ,
        COLOR_GREEN ,
        COLOR_CYAN ,
        COLOR_SKY ,
        COLOR_BLUE ,
        COLOR_PURPLE ,
        COLOR_FUCHSIA ,
        COLOR_PINK ,
    } enum_Color;

    SENSOR_COLORLIGHT(int slave_address);

    /*--------------Thunder IDE APIs: -------------*/
    int Get_Result(unsigned char result_index, unsigned char channel = 0);
    byte Set_Operate_Mode(byte optMode, unsigned char channel = 0);
    byte Set_Operate_Modes(byte optMode, byte optData, unsigned char channel = 0);
    void SetDetectRange(unsigned char max_value = 100, unsigned char min_value = 0, unsigned char sensorChannel = 0);
    void Reset(unsigned char channel = 0);
};

#endif
