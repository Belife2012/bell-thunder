#include "Disk_Manager.h"
#include "common.h"
#include "Sensor_Attitude.h"

ATTITUDE::ATTITUDE(/* args */)
{
}

ATTITUDE::~ATTITUDE()
{
}

void ATTITUDE::Init_Sensor()
{
    sensor_dev.setup();
    Load_CalibrateData();
}

void ATTITUDE::Open_Sensor()
{
    xTaskCreatePinnedToCore(ATTITUDE::Daemon_Attitude_Sensor, "attitudeSensor",
                    3072, NULL, 3, NULL, 1);
}

/**
 * @brief 在执行函数Calibrate_Sensor时，首先要静止放置3秒，
 * 然后分别绕着X、Y、Z轴旋转器件至少一圈
 * 
 * @return true 
 * @return false 
 */
bool ATTITUDE::Calibrate_Sensor()
{
    Speaker.Play_Song(95);// 开始校准
    delay(1500);
    sensor_dev.calibrateAccelGyro();
    Speaker.Play_Song(86);// 开始校准磁力计，需要旋转设备
    sensor_dev.calibrateMag();
    Speaker.Play_Song(97);// 校准完成
    Save_CalibrateData();
    sensor_dev.printCalibration();

    return true;
}

void ATTITUDE::ShowAttitude()
{
    Serial.printf("%f ", sensor_dev.getPitch());
    Serial.printf("%f ", sensor_dev.getRoll());
    Serial.printf("%f\n", sensor_dev.getYaw());
}

/**
 * @brief 获取姿态传感器的各个方向的旋转角度，
 * 角度值未清零时，角度值会进行累加
 * 注意：测量某一轴的旋转角度时，尽量保持其他两轴不动，否则测量误差很大
 * 
 * @param axis 
 * @return float 
 */
double ATTITUDE::GetAttitudeAngle(enum_Attitude_Axis axis)
{
    switch(axis){
        case X_AXIS:
            return x_angle;
            break;
        case Y_AXIS:
            return y_angle;
            break;
        case Z_AXIS:
            return z_angle;
            break;
        default:
            return 0.f;
    }
}

/**
 * @brief 清零旋转角度的记录，从0 开始记录旋转角度；
 * 
 * @param axis 可以分别清零x, y, z 轴，也可以全部一起清零
 */
void ATTITUDE::ClearAttitudeAngle(enum_Attitude_Axis axis)
{
    switch(axis){
        case X_AXIS:
            x_angle = 0.f;
            break;
        case Y_AXIS:
            y_angle = 0.f;
            break;
        case Z_AXIS:
            z_angle = 0.f;
            break;
        case ALL_AXIS:
            x_angle = 0.f;
            y_angle = 0.f;
            z_angle = 0.f;
            break;
        default: break;
    }
    
    return ;
}

/**
 * @brief 获取姿态传感器的角速度
 *    ^  Y轴正向
 *    |
 *    |
 * ------->  X轴正向 
 *    |
 *    |
 * 
 * @param axis X_AXIS：X轴，绕X轴的角速度，看向X轴正方向，顺时针绕着转为负值，反之为正值
 * Y_AXIS：Y轴，绕Y轴的角速度，看向Y轴正方向，顺时针绕着转为负值，反之为正值
 * Z_AXIS：Z轴(正向为下)，绕Z轴的角速度，看向Z轴正方向，顺时针绕着转为负值，反之为正值
 * @return float 角速度值
 */
float ATTITUDE::GetAttitudeAngleV(enum_Attitude_Axis axis)
{
    if(axis < 3){
        return angular_v[axis];
    }else{
        return 0.f;
    }
}

float ATTITUDE::GetAttitudeAccel(enum_Attitude_Axis axis)
{
    if(axis < 3){
        return accel_ms2[axis];
    }else{
        return 0.f;
    }
}

/**
 * @brief 获取姿态传感器的姿态，需要校准后才能使用正确
 *    ^  Y轴正向
 *    |
 *    |
 * ------->  X轴正向 
 *    |
 *    |
 * 
 * @param axis X_AXIS：绕X轴旋转，俯仰角度，测量范围 -180° ~ 180°
 * Y_AXIS：绕Y轴旋转，翻滚角度，测量范围 -90° ~ 90°
 * Z_AXIS：绕Z轴旋转，方向角度(偏航角)，相对于北方向，正北方向为0°，测量范围 0° ~ 360°，水平放置时，测量的方向才能准确
 * 
 * @return float 返回相对角度值
 */
float ATTITUDE::GetAttitudeStatus(enum_Attitude_Axis axis)
{
    float ret;
    switch(axis){
        case X_AXIS:
            ret = sensor_dev.getRoll();
            break;
        case Y_AXIS:
            ret = sensor_dev.getPitch();
            break;
        case Z_AXIS:
            ret = sensor_dev.getYaw();
            ret = (ret < -90.0f) ? (450.0f + ret) : (ret + 90.0f);
            break;
        default:
            ret = 0.f;
    }

    return ret;
}

float ATTITUDE::GetCoefficientWave(enum_Attitude_Axis axis)
{
    return coefficient_wave[axis];
}
/**
 * @brief 计算轴向的波动系数，并计算了静止状态时的平均值作为陀螺仪的基准
 * 
 * @param axis 轴向
 * @param new_data 每次更新陀螺仪数据时会产生一个新的值，传入进行计算
 */
void ATTITUDE::CalcCoefficientWave(enum_Attitude_Axis axis, float new_data)
{
    static uint8_t count_index[3];
    static float max_value[3], min_value[3], sum_all[3], accel_sum[3];

    if(count_index[axis] >= 20)
    {
        coefficient_wave[axis] = max_value[axis] - min_value[axis];
        // 如果波动系数小于相对静止是的波动系数
        if(coefficient_wave[axis] < STABLE_COEFFICIENT_WAVE){
            average_gyro[axis] = sum_all[axis] / count_index[axis];
            average_accel[axis] = accel_sum[axis] / count_index[axis];
        }

        count_index[axis] = 1;
        sum_all[axis] = new_data;
        accel_sum[axis] = accel_ms2[axis];
        max_value[axis] = new_data;
        min_value[axis] = new_data;
        return ;
    }

    count_index[axis]++;
    sum_all[axis] += new_data;
    accel_sum[axis] += accel_ms2[axis];
    if(max_value[axis] < new_data){
        max_value[axis] = new_data;
    }
    if(min_value[axis] > new_data){
        min_value[axis] = new_data;
    }
    return ;
}

/**
 * @brief 灵敏度越高，震荡的波动系数阈值就越小
 * 
 * @param new_value 参数设置范围：0~100，参数越高就越灵敏，能感受到更小幅度的震动;
 */
void ATTITUDE::SetShakeThreshold(float new_value)
{
    if(new_value > 100.f){
        new_value = 100.f;
    }else if(new_value < STABLE_COEFFICIENT_WAVE){
        new_value = STABLE_COEFFICIENT_WAVE;
    }

    shake_coefficient = 100.f - new_value + STABLE_COEFFICIENT_WAVE;
}

/**
 * @brief 获取震动状态，在震动灵敏度感应范围内如果检测到震动，返回true，否则返回false
 * 
 * @return true 
 * @return false 
 */
bool ATTITUDE::CheckShakeStatus() 
{
    float max_wave = 0.f;

    max_wave = coefficient_wave[X_AXIS] > max_wave ? coefficient_wave[X_AXIS] : max_wave;
    max_wave = coefficient_wave[Y_AXIS] > max_wave ? coefficient_wave[Y_AXIS] : max_wave;
    max_wave = coefficient_wave[Z_AXIS] > max_wave ? coefficient_wave[Z_AXIS] : max_wave;

    if(max_wave > shake_coefficient){
        return true;
    }else{
        return false;
    }
}

// private Func

void ATTITUDE::Save_CalibrateData()
{
    float calibrate_datas[12];
    calibrate_datas[0] = sensor_dev.getAccBias(0); 
    calibrate_datas[1] = sensor_dev.getAccBias(1); 
    calibrate_datas[2] = sensor_dev.getAccBias(2); 
    calibrate_datas[3] = sensor_dev.getGyroBias(0);
    calibrate_datas[4] = sensor_dev.getGyroBias(1);
    calibrate_datas[5] = sensor_dev.getGyroBias(2);
    calibrate_datas[6] = sensor_dev.getMagBias(0); 
    calibrate_datas[7] = sensor_dev.getMagBias(1); 
    calibrate_datas[8] = sensor_dev.getMagBias(2);  
    calibrate_datas[9] = sensor_dev.getMagScale(0);
    calibrate_datas[10] = sensor_dev.getMagScale(1);
    calibrate_datas[11] = sensor_dev.getMagScale(2);

    bool ack;
    ack = Disk_Manager.Write_Attitude_Calibrate(calibrate_datas);
    if(ack == false){
        Serial.println("Write Calibrate Fail");
        return ;
    }
}
void ATTITUDE::Load_CalibrateData()
{
    float calibrate_datas[12];
    bool ack;

    ack = Disk_Manager.Read_Attitude_Calibrate(calibrate_datas);
    if(ack == false){
        Serial.println("No Calibrate");
        return ;
    }

    sensor_dev.setAccBias(0, calibrate_datas[0]); 
    sensor_dev.setAccBias(1, calibrate_datas[1]); 
    sensor_dev.setAccBias(2, calibrate_datas[2]); 
    sensor_dev.setGyroBias(0, calibrate_datas[3]);
    sensor_dev.setGyroBias(1, calibrate_datas[4]);
    sensor_dev.setGyroBias(2, calibrate_datas[5]);
    sensor_dev.setMagBias(0, calibrate_datas[6]); 
    sensor_dev.setMagBias(1, calibrate_datas[7]); 
    sensor_dev.setMagBias(2, calibrate_datas[8]); 
    sensor_dev.setMagScale(0, calibrate_datas[9]);
    sensor_dev.setMagScale(1, calibrate_datas[10]);
    sensor_dev.setMagScale(2, calibrate_datas[11]);

    sensor_dev.printCalibration();
}

// global
ATTITUDE Attitude_Sensor;

void ATTITUDE::Daemon_Attitude_Sensor(void *pvParameters)
{
    static unsigned long time_record = millis();
    for(;;){
        Attitude_Sensor.sensor_dev.update();

        Attitude_Sensor.angular_v[X_AXIS] = Attitude_Sensor.sensor_dev.getGyro(X_AXIS);
        Attitude_Sensor.angular_v[Y_AXIS] = Attitude_Sensor.sensor_dev.getGyro(Y_AXIS);
        Attitude_Sensor.angular_v[Z_AXIS] = Attitude_Sensor.sensor_dev.getGyro(Z_AXIS);
        // Attitude_Sensor.angular_v[X_AXIS] = Data_Filter(Attitude_Sensor.angular_v[X_AXIS], Attitude_Sensor.gyro_cache[X_AXIS], 6, 1);
        // Attitude_Sensor.angular_v[Y_AXIS] = Data_Filter(Attitude_Sensor.angular_v[Y_AXIS], Attitude_Sensor.gyro_cache[Y_AXIS], 6, 1);
        // Attitude_Sensor.angular_v[Z_AXIS] = Data_Filter(Attitude_Sensor.angular_v[Z_AXIS], Attitude_Sensor.gyro_cache[Z_AXIS], 6, 1);
        Attitude_Sensor.accel_ms2[X_AXIS] = Attitude_Sensor.sensor_dev.getAcc(X_AXIS) * EARTH_GRAVITY_ACCEL;
        Attitude_Sensor.accel_ms2[Y_AXIS] = Attitude_Sensor.sensor_dev.getAcc(Y_AXIS) * EARTH_GRAVITY_ACCEL;
        Attitude_Sensor.accel_ms2[Z_AXIS] = Attitude_Sensor.sensor_dev.getAcc(Z_AXIS) * EARTH_GRAVITY_ACCEL;

        // 获取静止基准值
        Attitude_Sensor.CalcCoefficientWave(X_AXIS, Attitude_Sensor.angular_v[X_AXIS]);
        Attitude_Sensor.CalcCoefficientWave(Y_AXIS, Attitude_Sensor.angular_v[Y_AXIS]);
        Attitude_Sensor.CalcCoefficientWave(Z_AXIS, Attitude_Sensor.angular_v[Z_AXIS]);

        float interval;
        interval = millis() - time_record;
        interval = interval/1000.f + 0.0004;
        time_record = millis();

        Attitude_Sensor.x_angle += ( (Attitude_Sensor.angular_v[X_AXIS] - Attitude_Sensor.average_gyro[X_AXIS]) * interval );
        Attitude_Sensor.y_angle += ( (Attitude_Sensor.angular_v[Y_AXIS] - Attitude_Sensor.average_gyro[Y_AXIS]) * interval );
        Attitude_Sensor.z_angle += ( (Attitude_Sensor.angular_v[Z_AXIS] - Attitude_Sensor.average_gyro[Z_AXIS]) * interval );

        vTaskDelay(pdMS_TO_TICKS(16));
    }
}
