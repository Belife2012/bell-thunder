#pragma once
#ifndef __SENSOR_ATTITUDE__
#define __SENSOR_ATTITUDE__

#include "bell_thunder.h"
#include "MPU9250.h"

#define STABLE_COEFFICIENT_WAVE     (0.4) //稳定（相对静止）时波动系数在0.2左右
#define EARTH_GRAVITY_ACCEL         (9.8)

typedef enum{
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
} enum_Attitude_Axis;

class ATTITUDE
{
private:
    /* data */
    MPU9250 sensor_dev;
    double x_angle {0.f};
    double y_angle {0.f};
    double z_angle {0.f};
    
    float angular_v[3] {0.f,0.f,0.f};
    float accel_ms2[3] {0.f,0.f,0.f};
    float coefficient_wave[3] {0.f,0.f,0.f}; // 一段时间内的波动系数
    float shake_coefficient {STABLE_COEFFICIENT_WAVE}; // 震荡灵敏度值
    float average_gyro[3] {0.f,0.f,0.f}; // 相对静止时一段时间内的陀螺仪数据平均值
    float average_accel[3] {0.f,0.f,0.f};
    
    void Save_CalibrateData();
    void Load_CalibrateData();
    static void Daemon_Attitude_Sensor(void *pvParameters);
public:
    ATTITUDE(/* args */);
    ~ATTITUDE();

    void Init_Sensor();
    void Open_Sensor();
    bool Calibrate_Sensor();
    void ShowAttitude();
    float GetAttitudeStatus(int axis);
    double GetAttitudeAngle(int axis);
    void ClearAttitudeAngle(int axis);
    float GetAttitudeAngleV(int axis);

    float GetAttitudeAccel(int axis);

    void CalcCoefficientWave(int axis, float new_data);
    float GetCoefficientWave(int axis);

    void SetShakeThreshold(float new_value);
    bool CheckShakeStatus(void);
};

extern ATTITUDE Attitude_Sensor;

#endif
