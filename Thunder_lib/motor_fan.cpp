#include "motor_fan.h"

void MOTOR_FAN::Set_Fan_Speed(unsigned char speed)
{
    fan_speed = speed;

    write(FAN_REG_SPEED, &fan_speed, 1);
}

unsigned char MOTOR_FAN::Get_Fan_Speed(void)
{
    return fan_speed;
}


