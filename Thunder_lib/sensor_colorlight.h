#include "iic_thunder.h"

#ifndef _COLORLIGHT_I2C_
#define _COLORLIGHT_I2C_

// 光电模块
#define COLORLIGHT_IIC_ADDR 0x0B //光电模块I2C器件地址

#define CLINE_IIC_REG_SETTING        0x01 // 设置工作模式和反射模式的灯光, bit0~3: 工作模式；bit4~7: 反射模式的RGB灯数据
#define CLINE_IIC_REG_ENV            0x02
#define CLINE_IIC_REG_REFLECT_R      0x03
#define CLINE_IIC_REG_REFLECT_G      0x04
#define CLINE_IIC_REG_REFLECT_B      0x05
#define CLINE_IIC_REG_COLOR          0x06
#define CLINE_IIC_REG_HSV_H          0x07
#define CLINE_IIC_REG_HSV_V          0x09
#define CLINE_IIC_REG_HSV_S          0x0B
#define CLINE_IIC_REG_COLOR_R        0x0C
#define CLINE_IIC_REG_COLOR_G        0x0E
#define CLINE_IIC_REG_COLOR_B        0x10
#define CLINE_IIC_REG_REFLECT_MAX    0x12
#define CLINE_IIC_REG_REFLECT_MIN    0x13

class SENSOR_COLORLIGHT : public SENSOR_IIC
{
private:

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
    typedef enum
    {
        DATA_HSV_H = 10,
        DATA_HSV_S,
        DATA_HSV_V,
        DATA_COLOR_R,
        DATA_COLOR_G,
        DATA_COLOR_B
    } enum_Data_Index;
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
    byte Set_Reflect_Led(byte optData, unsigned char channel = 0);
    void Set_Extremum(int mode, float value, uint8_t channel=0);
    
    void SetDetectRange(unsigned char max_value = 100, unsigned char min_value = 0, unsigned char sensorChannel = 0);
    void Reset(unsigned char channel = 0);
};

#endif
