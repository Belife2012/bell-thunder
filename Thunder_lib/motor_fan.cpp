#include "motor_fan.h"

signed char MOTOR_FAN::Get_Fan_Speed(unsigned char channel)
{
    unsigned char speed;

    if(0 != read(FAN_REG_SPEED, &speed, 1, channel)){
        speed = 0;
    }

    return speed;
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief: 设置风扇电机的功率
 * 
 * @param speed: 功率值-100~100，负数表示反转，数值的绝对值越大，电机功率越大
 * @param channel:风扇电机接口的编号：1/2/3/4/5(A)/6(B)
 */
void MOTOR_FAN::Set_Fan_Speed(signed char speed, unsigned char channel)
{
    signed char fan_speed;
    fan_speed = speed;

    CHECK_RANGE(fan_speed, -100, 100);
    write(FAN_REG_SPEED, (const unsigned char *)&fan_speed, 1, channel);
}

