#include "sensor_attitude.h"
#include "byte_convertion.h"
/**
 * @brief 在执行函数Calibrate_Sensor时，首先要静止放置3秒，
 * 然后分别绕着X、Y、Z轴旋转器件至少一圈
 * 
 * @return true 
 * @return false 
 */
bool SENSOR_ATTITUDE::CalibrateSensor()
{
}

void SENSOR_ATTITUDE::ShowAttitude(unsigned char channel)
{
    float result;
    byte read_buf[4];
    BYTE_CONVERTION _data;

    read(ATTITUDE_IIC_REG_PITCH, read_buf, 4, channel);
    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &result);
    Serial.printf("pitch: %6.2f ", result);

    read(ATTITUDE_IIC_REG_ROLL, read_buf, 4, channel);
    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &result);
    Serial.printf("roll: %6.2f ", result);
    
    read(ATTITUDE_IIC_REG_YAW, read_buf, 4, channel);
    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &result);
    Serial.printf("yaw: %6.2f \n", result);
}

/**
 * @brief 获取姿态传感器的各个方向的旋转角度，
 * 角度值未清零时，角度值会进行累加
 * 注意：测量某一轴的旋转角度时，尽量保持其他两轴不动，否则测量误差很大
 *      测量结果会产生叠加误差，所以一般不会用于测量长时间的旋转角度
 * 
 * @param axis 
 * @return float 
 */
float SENSOR_ATTITUDE::GetAttitudeAngle(int axis, unsigned char channel)
{
    float result;
    byte read_buf[4];
    BYTE_CONVERTION _data;

    switch(axis){
        case AXIS_X:
            read(ATTITUDE_IIC_REG_ANGLE_X, read_buf, 4, channel);
            break;
        case AXIS_Y:
            read(ATTITUDE_IIC_REG_ANGLE_Y, read_buf, 4, channel);
            break;
        case AXIS_Z:
            read(ATTITUDE_IIC_REG_ANGLE_Z, read_buf, 4, channel);
            break;
        default:
            result = 0.f;
            return result;
    }

    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &result);
    return result;
}

/**
 * @brief 清零旋转角度的记录，从0 开始记录旋转角度；
 * 
 * @param axis 可以分别清零x, y, z 轴，ALL_AXIS可以全部一起清零
 */
void SENSOR_ATTITUDE::ClearAttitudeAngle(int axis, unsigned char channel)
{
    byte write_buf;

    write_buf = 0x01;
    switch(axis){
        case AXIS_X:
            write(ATTITUDE_IIC_REG_ANGLE_RESET_X, &write_buf, 1, channel);
            break;
        case AXIS_Y:
            write(ATTITUDE_IIC_REG_ANGLE_RESET_Y, &write_buf, 1, channel);
            break;
        case AXIS_Z:
            write(ATTITUDE_IIC_REG_ANGLE_RESET_Z, &write_buf, 1, channel);
            break;
        default:
            break;
    }
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
float SENSOR_ATTITUDE::GetAttitudeAngleV(int axis, unsigned char channel)
{
    float result;
    byte read_buf[4];
    BYTE_CONVERTION _data;

    switch(axis){
        case AXIS_X:
            read(ATTITUDE_IIC_REG_ANGLEV_X, read_buf, 4, channel);
            break;
        case AXIS_Y:
            read(ATTITUDE_IIC_REG_ANGLEV_Y, read_buf, 4, channel);
            break;
        case AXIS_Z:
            read(ATTITUDE_IIC_REG_ANGLEV_Z, read_buf, 4, channel);
            break;
        default:
            result = 0.f;
            return result;
    }

    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &result);
    return result;
}

/**
 * @brief 获取姿态传感器的各个方向(X Y Z)的加速度（m/s2）
 * 这里所说的加速度是包含了重力加速度，其处于静止状态时会有一个固定的重力加速度
 * （也就是9.8 m/s2，所以有时候也叫重力传感器），如果器件不处于严格的水平状态，
 * 各轴向的重力加速度分量会因轴向不同而发生改变。如果器件外加一个运动加速度或者
 * 旋转（器件会感应到旋转产生的向心力），器件感应到的就不仅仅是重力加速度，还有
 * 运动加速度和旋转向心力，所以要用于某个方面的检测时，要控制好其他变量，或者
 * 做好数据融合算法。
 * 
 * @param axis 
 * @return float 
 */
float SENSOR_ATTITUDE::GetAttitudeAccel(int axis, unsigned char channel)
{
    float result;
    byte read_buf[4];
    BYTE_CONVERTION _data;

    switch(axis){
        case AXIS_X:
            read(ATTITUDE_IIC_REG_ACCEL_X, read_buf, 4, channel);
            break;
        case AXIS_Y:
            read(ATTITUDE_IIC_REG_ACCEL_Y, read_buf, 4, channel);
            break;
        case AXIS_Z:
            read(ATTITUDE_IIC_REG_ACCEL_Z, read_buf, 4, channel);
            break;
        default:
            result = 0.f;
            return result;
    }

    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &result);
    return result;
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
float SENSOR_ATTITUDE::GetAttitude(int axis, unsigned char channel)
{
    float result;
    byte read_buf[4];
    BYTE_CONVERTION _data;

    switch(axis){
        case AXIS_X:
            read(ATTITUDE_IIC_REG_PITCH, read_buf, 4, channel);
            break;
        case AXIS_Y:
            read(ATTITUDE_IIC_REG_ROLL, read_buf, 4, channel);
            break;
        case AXIS_Z:
            read(ATTITUDE_IIC_REG_YAW, read_buf, 4, channel);
            break;
        default:
            result = 0.f;
            return result;
    }

    byte_convertion_init(&_data, read_buf, 4, 0);
    byte_convertion_read_float(&_data, &result);
    return result;
}

/**
 * @brief 灵敏度越高，震荡的波动系数阈值就越小
 * 
 * @param new_value 参数设置范围：0~100，参数越高就越灵敏，能感受到更小幅度的震动;
 */
void SENSOR_ATTITUDE::SetShakeSens(unsigned char new_value, unsigned char channel)
{
    write(ATTITUDE_IIC_REG_SHAKE_SENS, &new_value, 1, channel);
}

/**
 * @brief 获取震动状态，在震动灵敏度感应范围内如果检测到震动，返回true，否则返回false
 * 
 * @return true 检测到震动
 * @return false 没有检测到震动
 */
bool SENSOR_ATTITUDE::CheckShakeStatus(unsigned char channel) 
{
    byte read_buf;

    read(ATTITUDE_IIC_REG_YAW, &read_buf, 1, channel);

    return (read_buf==0) ? false : true;
}
