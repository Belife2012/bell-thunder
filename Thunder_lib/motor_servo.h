#ifndef __MOTOR_SERVO_H__
#define __MOTOR_SERVO_H__

class MOTOR_SERVO
{
private:
    /* data */
public:
    void Setup_Servo(void);
    void Servo_Turn(int servo, float angle);
    void Servo_Percent_Setting(int servo_index, float max_value, float min_value, float zero_value, int direction);
    void Servo_Turn_Percent(int servo, float percent);
};

#endif
