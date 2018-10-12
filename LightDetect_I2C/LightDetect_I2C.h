#ifndef _LIGHTDETECT_I2C_
#define _LIGHTDETECT_I2C_

class LIGHTDETECT_I2C
{
public:
    void Set_Operate_Mode(byte optMode);
    unsigned short Get_Light_Value(void);
};

#endif
