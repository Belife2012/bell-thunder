#pragma once
#ifndef __SENSOR_ATTITUDE__
#define __SENSOR_ATTITUDE__

#include "iic_thunder.h"

#define ATTITUDE_IIC_ADDR   (0x68)
#define ATTITUDE_IIC_REG_ANGLE_X    (0x01)
#define ATTITUDE_IIC_REG_ANGLE_Y    (0x05)
#define ATTITUDE_IIC_REG_ANGLE_Z    (0x09)
#define ATTITUDE_IIC_REG_ANGLEV_X   (0x0D)
#define ATTITUDE_IIC_REG_ANGLEV_Y   (0x11)
#define ATTITUDE_IIC_REG_ANGLEV_Z   (0x15)
#define ATTITUDE_IIC_REG_ACCEL_X    (0x19)
#define ATTITUDE_IIC_REG_ACCEL_Y    (0x1D)
#define ATTITUDE_IIC_REG_ACCEL_Z    (0x21)
#define ATTITUDE_IIC_REG_PITCH      (0x25)
#define ATTITUDE_IIC_REG_ROLL       (0x29)
#define ATTITUDE_IIC_REG_YAW        (0x2D)
#define ATTITUDE_IIC_REG_SHAKE_SENS     (0x32)
#define ATTITUDE_IIC_REG_SHAKE_STATUS   (0x33)
#define ATTITUDE_IIC_REG_ANGLE_RESET_X    (0x34)
#define ATTITUDE_IIC_REG_ANGLE_RESET_Y    (0x35)
#define ATTITUDE_IIC_REG_ANGLE_RESET_Z    (0x36)

class SENSOR_ATTITUDE : public SENSOR_IIC
{
private:
    /* data */
public:
    typedef enum{
        AXIS_X = 0,
        AXIS_Y,
        AXIS_Z,
        AXIS
    } enum_Attitude_Axis;

    SENSOR_ATTITUDE(int slave_address):SENSOR_IIC(slave_address) {};

    bool CalibrateSensor();
    void ShowAttitude(unsigned char channel=0);
    float GetAttitude(int axis, unsigned char channel=0);
    float GetAttitudeAngle(int axis, unsigned char channel=0);
    void ClearAttitudeAngle(int axis, unsigned char channel=0);
    float GetAttitudeAngleV(int axis, unsigned char channel=0);
    float GetAttitudeAccel(int axis, unsigned char channel=0);
    void SetShakeSens(unsigned char  new_value, unsigned char channel=0);
    bool CheckShakeStatus(unsigned char channel=0);
};

#endif
