#if 1
#include <bell_thunder.h>

#define PRINT_DEBUG_INFO
#define PRINT_DEBUG_ERROR

uint8_t LED_Counter = 1;

////////////////////////////////////////////// 巡线 ///////////////////////////////////////////////////////////
void Test_Line_Tracing()
{
    Bell_Thunder.Enable_En_Motor(); // En_Motor_Flag = 1;
    Rx_Data[1] = 1;
    Bell_Thunder.Line_Tracing();
}

////////////////////////////////////////////// 负载测试 ///////////////////////////////////////////////////////////
//LED_Counter = 1;
void Demo_3()
{
    ///////////////// 前进 /////////////////
    Motor_Thunder.Motor_Move(1, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
    Motor_Thunder.Motor_Move(2, 255, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);

    ///////////////// 舵机 /////////////////
    Motor_Servo.Servo_Turn(1, 120); //臂 A
    Motor_Servo.Servo_Turn(2, 120); //爪 B

    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);

    ///////////////// 舵机 /////////////////
    Motor_Servo.Servo_Turn(1, 65);  //臂 A
    Motor_Servo.Servo_Turn(2, 120); //爪 B

    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);

    ///////////////// 后退 /////////////////
    Motor_Thunder.Motor_Move(1, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
    Motor_Thunder.Motor_Move(2, 255, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);

    ///////////////// 舵机 /////////////////
    Motor_Servo.Servo_Turn(1, 90);  //臂 A
    Motor_Servo.Servo_Turn(2, 180); //爪 B

    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
    Display_Screen.Play_Thunder_Picture(LED_Counter); //单色点阵图案
    LED_Counter++;
    if (LED_Counter > 100)
        LED_Counter = 1;
    delay(200);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////// 超声波构型 ///////////////////////////////////////////////////////////
void Demo_US()
{
    //超声波数据
    float US_Data_cm = 0;
    US_Data_cm = Sensor_Ultrasonic.Get_Distance();
    Serial.printf("SSSSSSSSSS US_Data_cm : %.1f [cm]SSSSSSSSSS\n", US_Data_cm);

    if (US_Data_cm != 0)
    {
        if (US_Data_cm < 20)
        {
            //暂停
            Motor_Thunder.Motor_Move(1, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            Motor_Thunder.Motor_Move(2, 0, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            delay(50);
            //后退
            //        Motor_Thunder.Motor_Move(1, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            //        Motor_Thunder.Motor_Move(2, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

            Motor_Thunder.Motor_Move(1, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            Motor_Thunder.Motor_Move(2, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            delay(1000);
            //左转
            //        Motor_Thunder.Motor_Move(1, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            //        Motor_Thunder.Motor_Move(2, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

            Motor_Thunder.Motor_Move(1, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            Motor_Thunder.Motor_Move(2, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            delay(500);
        }
        else
        { //前进
            //        Motor_Thunder.Motor_Move(1, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            //        Motor_Thunder.Motor_Move(2, 150, 2); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向

            Motor_Thunder.Motor_Move(1, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
            Motor_Thunder.Motor_Move(2, 150, 1); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
        }
    }

    delay(50);
}

/******************************** Auto test process ************************************/
void Auto_Test_Motor()
{
    static uint8_t test_status = 0;

    if (test_status == 1)
    {
        Motor_Thunder.Set_Target(1, 8); //50ms 最大40  //编码器计数值
        Motor_Thunder.Set_Target(2, 8);
        //Serial.println("*** set motor: 10 ***");
    }
    else if (test_status == 2)
    {
        Motor_Thunder.Set_Target(1, 100); //50ms 最大40  //编码器计数值
        Motor_Thunder.Set_Target(2, 100);
        //Serial.println("*** set motor: 5 ***");
    }
    else if (test_status == 3)
    {
        Motor_Thunder.Set_Target(1, 0); //50ms 最大40  //编码器计数值
        Motor_Thunder.Set_Target(2, 0);
        //Serial.println("*** set motor: 0 ***");
    }
    else if (test_status == 4)
    {
        Motor_Thunder.Set_Target(1, -100); //50ms 最大40  //编码器计数值
        Motor_Thunder.Set_Target(2, -100);
        //Serial.println("*** set motor: 20 ***");
    }
    else if (test_status == 5)
    {
        Motor_Thunder.Set_Target(1, -8); //50ms 最大40  //编码器计数值
        Motor_Thunder.Set_Target(2, -8);
        //Serial.println("*** set motor: -8 ***");
    }
    else
    {
        test_status = 0;
    }
    test_status++;

    delay(3000);
}

void Auto_Test_Led()
{
    static uint8_t test_status = 3;

    Display_Screen.Play_Animation(test_status);

    test_status++;
    if (test_status > 15)
    {
        test_status = 3;
    }
}

void Auto_Test_US()
{
    //超声波数据
    float US_Data_cm = 0;

    US_Data_cm = Sensor_Ultrasonic.Get_Distance();
    if (US_Data_cm < 1.0)
    {
#ifdef PRINT_DEBUG_ERROR
        Serial.printf("### US_Data_cm : %.1f [cm]###############################\n", US_Data_cm);
#endif
    }
    else
    {
#ifdef PRINT_DEBUG_INFO
        Serial.printf("* US_Data_cm : %.1f [cm]*\n", US_Data_cm);
#endif
    }
}

void Auto_Test_Speaker()
{
    static uint8_t test_status = 169;

    int busy_flag = Speaker_Thunder.WT588_Busy_Check(); // 0  播放中  1 停止
    if (busy_flag == 0)
    {
        return; // 正在有音乐播放中，不进行新的播放
    }
    else if (busy_flag == 1)
    {
    }

    Speaker_Thunder.Set_Sound_Volume(100);
    Speaker_Thunder.Play_Song(test_status);

    // test_status++;
    // if(test_status > 174){
    //   test_status = 100;
    // }
}

/**
 * 
 */
void Auto_Test_Color()
{
    //颜色识别数据
    unsigned short RGBC[4] = {0};
    float HSV[3] = {0};
    //  uint8_t Colour_Num = 0;

    if (Sensor_Color.Get_RGBC_Data(RGBC) != 0)
    {
        RGBC[0] = 0;
        RGBC[1] = 0;
        RGBC[2] = 0;
        RGBC[3] = 0;
#ifdef PRINT_DEBUG_ERROR
        Serial.printf("### RGBC no data ###############################\n");
#endif
    }
    else
    {
        Sensor_Color.RGBtoHSV(RGBC, HSV); // 计算HSV

#ifdef PRINT_DEBUG_INFO
        Serial.printf("* R: %d ~G: %d ~B: %d ~C: %d ~H: %f *\n", RGBC[0], RGBC[1], RGBC[2], RGBC[3], HSV[0]); //(RED)(GREEN)(BLUE)(CLEAR)
#endif
    }
}

void Auto_Test_LED_Color()
{
    static byte ledDataIndex = 0;

    byte LEDs_DataResult[36];
    byte colorData[36] = {0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8,
                          0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8, 0xC8, 0x00, 0xC8};

    for (byte i = 0; i < sizeof(colorData); i++)
    {
        LEDs_DataResult[i] = (byte)((((float)colorData[i]) / 2) * (cos(ledDataIndex * 2 * PI / 200 - PI) + 1));
    }

    LED_Color.Set_LEDs_Data(0x01, LEDs_DataResult, sizeof(LEDs_DataResult) / 2);
    LED_Color.Set_LEDs_Data(0x07, LEDs_DataResult + 18, sizeof(LEDs_DataResult) / 2);
    // delay(10);
    LED_Color.LED_Updata();

    if (ledDataIndex++ > 200)
        ledDataIndex = 0;
}

void Auto_Test_Touch()
{
    static byte ledIndex = 0;
    byte statusValue, errorCode;

    switch (ledIndex)
    {
    case 0:
        Sensor_Touch.Set_LED_RGBvalue(10, 0, 0);
        // Serial.println("red LED");
        break;
    case 1:
        Sensor_Touch.Set_LED_RGBvalue(0, 10, 0);
        // Serial.println("green LED");
        break;
    case 2:
        Sensor_Touch.Set_LED_RGBvalue(0, 0, 10);
        // Serial.println("blue LED");
        break;
    case 5:
        Sensor_Touch.Reset_Mode();
    default:
        break;
    }

    // errorCode = Sensor_Touch.Get_Status(&statusValue);
    // if(errorCode != 0){
    //   Serial.println("### Touch read Error #################################");
    // }else Serial.printf("* Touch Status: %d \n", statusValue);

    if (Sensor_Touch.Check_Event(TOUCH_EVENT_RELEASE) == true)
    {
        Serial.println("Touch Release");
    }
    if (Sensor_Touch.Check_Event(TOUCH_EVENT_PRESS) == true)
    {
        Serial.println("Touch Press");
    }
    if (Sensor_Touch.Check_Event(TOUCH_EVENT_TOUCH) == true)
    {
        Serial.println("Touch Happened");
    }

    if (++ledIndex > 10)
        ledIndex = 0;
    // else if(ledIndex > 5) Sensor_Touch.Reset_Mode();
}

void Auto_Test_Light()
{
    static byte lightIndex = 0;
    float lightValue;
    byte errorCode;

    // if(lightIndex < 10){
    //     lightIndex++;
    //     Sensor_Light.Set_Led_Brightness(lightIndex*10);
    // }
    // else{
    //     lightIndex = 0;
    //     Sensor_Light.Set_Led_Brightness(0);
    // }
    // delay(30);

    Sensor_Light.Set_Led_Brightness(0);
    delay(500);
    Serial.println("=======50ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(50);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(50);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======40ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(40);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(40);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======30ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(30);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(30);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======20ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(20);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(20);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======15ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(15);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(15);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======10ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(10);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(10);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======5ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(5);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(5);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======4ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(4);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(4);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======3ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(3);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(3);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======2ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(2);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(2);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    delay(500);
    Serial.println("=======1ms========");
    Sensor_Light.Set_Led_Brightness(100);
    delay(1);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
    Sensor_Light.Set_Led_Brightness(0);
    delay(1);
    lightValue = Sensor_Light.Get_Light_Value(0);
    Serial.printf("* light Value: %.2f \n", lightValue);
}

void Auto_Get_Motor_Speed_Value()
{
    int16_t leftSpeed, rightSpeed;

    leftSpeed = Motor_Thunder.Get_Speed_simple(1);
    rightSpeed = Motor_Thunder.Get_Speed_simple(2);

    Serial.printf("%d\t%d\n", leftSpeed, rightSpeed);
}

/* 
 * 测试电机编码器的数值与旋转角度的关系
 * 
 * @parameters: 
 * @return: 
 */
void Auto_Test_Encoder_RotateValue()
{
    int32_t Pre_encoderValue;
    int32_t Cur_encoderValue;
    int8_t flag = 1;

    // 关闭电机PID控制
    Bell_Thunder.Disable_En_Motor();
    while (1)
    {
        Pre_encoderValue = Motor_Thunder.Get_RotateValue(1);
        Motor_Thunder.Set_Motor_Power(1, 40 * flag);
        for (;;)
        {
            Cur_encoderValue = Motor_Thunder.Get_RotateValue(1);
            if (abs(Cur_encoderValue - Pre_encoderValue) > 216)
            {
                Motor_Thunder.Set_Motor_Power(1, 0);
                break;
            }
        }
        delay(1000);
        Cur_encoderValue = Motor_Thunder.Get_RotateValue(1);
        Serial.printf("rotate: %d\n", abs(Cur_encoderValue - Pre_encoderValue));

        Pre_encoderValue = Motor_Thunder.Get_RotateValue(2);
        Motor_Thunder.Set_Motor_Power(2, 40 * flag);
        for (;;)
        {
            Cur_encoderValue = Motor_Thunder.Get_RotateValue(2);
            if (abs(Cur_encoderValue - Pre_encoderValue) > 216)
            {
                Motor_Thunder.Set_Motor_Power(2, 0);
                break;
            }
        }
        delay(1000);
        Cur_encoderValue = Motor_Thunder.Get_RotateValue(2);
        Serial.printf("rotate: %d\n\n", abs(Cur_encoderValue - Pre_encoderValue));

        if (flag == 1)
        {
            flag = -1;
        }
        else
        {
            flag = 1;
        }
    }
}

void Auto_Test_Flame_Sensor(byte portIn)
{
    Serial.printf("Angle: %3d", Sensor_Flame.Get_Flame_Angle(portIn));
    Serial.printf(" Intens: %3d\n", Sensor_Flame.Get_Flame_Intensity(portIn));

    // delay(100);
}

void Auto_Test_Fan_Motor()
{
    // delay(1000);
    for(int8_t i = 0; i < 100; i++)
    {
        Motor_Fan.Set_Fan_Speed(i,1);
    //   delay(100);
        Serial.print("set:");Serial.println(i);
        Serial.print("get:");Serial.println(Motor_Fan.Get_Fan_Speed(1));
        delay(200);
    }

    for (int8_t i = 0; i < 100; i++)
    {
        Motor_Fan.Set_Fan_Speed( (100 - i),1);
    //   delay(100);
        Serial.print("set:");Serial.println(100 - i);
        Serial.print("get:");Serial.println(Motor_Fan.Get_Fan_Speed(1));
        delay(200);
    }

    for (int8_t i = -100; i < 0; i++)
    {
        Motor_Fan.Set_Fan_Speed((-i-100),1);
    //   delay(100);
        Serial.print("set:");Serial.println(-i-100);
        Serial.print("get:");Serial.println(Motor_Fan.Get_Fan_Speed(1));
        delay(200);
    }

    for (int8_t i = -100; i < 0; i++)
    {
        Motor_Fan.Set_Fan_Speed(i,1);
    //   delay(100);
        Serial.print("set:");Serial.println(i);
        Serial.print("get:");Serial.println(Motor_Fan.Get_Fan_Speed(1));
        delay(200);
    }  
}

void Auto_Toxicgas_Sensor()
{
    Serial.println("normal test");
    for(uint8_t i = 0; i < 20; i++)
    {
        Serial.println(Sensor_Gas.GetToxicgasRatio(3));
        delay(1000);
    }

    Serial.println("mode test");
    Sensor_Gas.SetDetectRange(60,30,3);
    for(uint8_t i = 0; i < 20; i++)
    {
        Serial.println(Sensor_Gas.GetToxicgasRatio(3));
        delay(1000);
    }

    Serial.println("reset test");
    Sensor_Gas.SetDetectRange(100,0,3);
    for(uint8_t i = 0; i < 20; i++)
    {
        Serial.println(Sensor_Gas.GetToxicgasRatio(3));
        delay(1000);
    }
}

void Auto_Infrared_Sensor()
{
    static uint8_t modeFlag = 0;

	// 红外接收模块初始化
    switch(modeFlag){
        case 0:
            Serial.println("initial SYS_MODE_DISTANCE");
            Sensor_Infrared.SetSysMode(SENSOR_INFRARED::SYS_MODE_DISTANCE, SENSOR_IIC::PORT_1);
            modeFlag = 1;
            break;
        case 1:
            Serial.println("initial SYS_MODE_REMOTE");
            Sensor_Infrared.SetSysMode(SENSOR_INFRARED::SYS_MODE_DISTANCE, SENSOR_IIC::PORT_1);
            modeFlag = 2;
            break;
        case 2:
            Serial.println("initial SYS_MODE_BEACON");
            Sensor_Infrared.SetSysMode(SENSOR_INFRARED::SYS_MODE_DISTANCE, SENSOR_IIC::PORT_1);
            modeFlag = 0;
            break;
        default:
            Serial.println("initial ERROR");
            break;
    }
    
    for(int i=0; i<50; i++){
        Serial.printf("Remote:%3d ", Sensor_Infrared.GetRemoteInfo(SENSOR_IIC::PORT_1));
        Serial.printf("BeaconD:%3d ", Sensor_Infrared.GetBeaconDist(SENSOR_IIC::PORT_1));
        Serial.printf("BeaconA:%3d ", Sensor_Infrared.GetBeaconDire(SENSOR_IIC::PORT_1));
        Serial.printf("Distance:%3d\n", Sensor_Infrared.GetDistance(SENSOR_IIC::PORT_1));
        delay(100);
    }
}

void Auto_Test_ColorLight(bool env_flag, bool color_flag, bool reflect_flag)
{
    static uint32_t timer = 0, work_mode = 0;

    uint8_t channel = 1;
    int result;

    if (timer + 5000 > millis() && reflect_flag)
    {
        if (work_mode != 1)
        {
            // Sensor_ColorLight.SetDetectRange(50, 0, channel);
            Sensor_ColorLight.Set_Operate_Mode(SENSOR_COLORLIGHT::MODE_REFLECT, channel);
            // Sensor_ColorLight.Set_Reflect_Led(0x07, channel);
            work_mode = 1;
        }
        Serial.printf("Reflect: %d", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_REFLECT_R, channel));
        Serial.printf(" %d", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_REFLECT_G, channel));
        Serial.printf(" %d\n", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_REFLECT_B, channel));
    }
    else if (timer + 10000 > millis() && color_flag)
    {
        if (work_mode != 2)
        {
            Sensor_ColorLight.Set_Operate_Mode(SENSOR_COLORLIGHT::MODE_COLOR, channel);
            work_mode = 2;
        }
        result = Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, channel);
        Serial.printf("Color:%2d ", result);
        switch (result)
        {
        case SENSOR_COLORLIGHT::COLOR_NON:
            Serial.printf("NON       ");
            break;
        case SENSOR_COLORLIGHT::COLOR_RED:
            Serial.printf("RED       ");
            break;
        case SENSOR_COLORLIGHT::COLOR_YELLOW:
            Serial.printf("YELLOW    ");
            break;
        case SENSOR_COLORLIGHT::COLOR_GREEN:
            Serial.printf("GREEN     ");
            break;
        case SENSOR_COLORLIGHT::COLOR_CYAN:
            Serial.printf("CYAN      ");
            break;
        case SENSOR_COLORLIGHT::COLOR_BLUE:
            Serial.printf("BLUE      ");
            break;
        case SENSOR_COLORLIGHT::COLOR_PURPLE:
            Serial.printf("PURPLE    ");
            break;
        case SENSOR_COLORLIGHT::COLOR_BLACK:
            Serial.printf("BLACK     ");
            break;
        case SENSOR_COLORLIGHT::COLOR_WHITE:
            Serial.printf("WHITE     ");
            break;

        default:
            Serial.printf("EEEError  ");
            break;
        }
        Serial.printf("data: %3d", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::DATA_HSV_H, channel));
        Serial.printf(" %3d", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::DATA_HSV_S, channel));
        Serial.printf(" %3d", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::DATA_HSV_V, channel));
        Serial.printf(" %3d", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::DATA_COLOR_R, channel));
        Serial.printf(" %3d", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::DATA_COLOR_G, channel));
        Serial.printf(" %3d\n", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::DATA_COLOR_B, channel));
    }
    else if(env_flag)
    {
        if (work_mode != 3)
        {
            Sensor_ColorLight.Set_Operate_Mode(SENSOR_COLORLIGHT::MODE_ENV, channel);
            Sensor_ColorLight.Set_Extremum(3, 20, channel);
            Sensor_ColorLight.Set_Extremum(4, 0, channel);
            work_mode = 3;
        }
        Serial.printf("env: %d\n", Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_ENV, channel));
    }

    if (timer + 15000 < millis())
    {
        timer = millis();
    }
    delay(10);
}

// #define REMOTER_CONTROL_SERVO_TEST
void Auto_Test_Servo()
{
    /* 测试舵机 */
    float servo_status = 0;
    delay(2000);
    Motor_Servo.Servo_Turn_Percent(1, servo_status);
    Motor_Servo.Servo_Turn_Percent(2, servo_status);
    delay(1000);
    for (;;)
    {
#ifdef REMOTER_CONTROL_SERVO_TEST
        for (;;)
        {
            if (Ble_Remoter.Get_Key_Value(KEY_UP))
            {
                servo_status += 0.1;
                break;
            }
            else if (Ble_Remoter.Get_Key_Value(KEY_DOWN))
            {
                servo_status -= 0.1;
                break;
            }
        }
#else
        servo_status -= 0.1;
#endif
        Motor_Servo.Servo_Turn_Percent(1, servo_status);
        Motor_Servo.Servo_Turn_Percent(2, servo_status);
        Serial.printf("servo: %f\n", servo_status);
        delay(10);
        if (servo_status <= -100)
            break;
    }

    delay(2000);
    servo_status = 0;
    Motor_Servo.Servo_Turn_Percent(1, servo_status);
    Motor_Servo.Servo_Turn_Percent(2, servo_status);
    delay(1000);
    for (;;)
    {
#ifdef REMOTER_CONTROL_SERVO_TEST
        for (;;)
        {
            if (Ble_Remoter.Get_Key_Value(KEY_UP))
            {
                servo_status += 0.1;
                break;
            }
            else if (Ble_Remoter.Get_Key_Value(KEY_DOWN))
            {
                servo_status -= 0.1;
                break;
            }
        }
#else
        servo_status += 0.1;
#endif
        Motor_Servo.Servo_Turn_Percent(1, servo_status);
        Motor_Servo.Servo_Turn_Percent(2, servo_status);
        Serial.printf("servo: %f\n", servo_status);
        delay(10);
        if (servo_status >= 100)
            break;
    }
}

void Auto_Test_Line()
{
    uint8_t IR_Data[2];
    Bell_Thunder.Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑

    Serial.printf("* IR_Data[0]:%d  IR_Data[1]:%d *\n", IR_Data[0], IR_Data[1]);
}


void test_read_Us()
{
    float US_Data_cm = 0;

    US_Data_cm = Sensor_Ultrasonic.Get_Distance();
    if (US_Data_cm < 1.0)
    {
        // Serial.printf("### US_Data_cm : %.1f [cm]###############1################\n", US_Data_cm);
    }
    else
    {
        // Serial.printf("*** US_Data_cm : %.1f [cm]***\n", US_Data_cm);
    }
}

// 测试超声波 无延时的读取数据
void test_Us_QuickRead()
{
    while (1)
    {
        Display_Screen.Play_Thunder_Picture(6); //单色点阵图案
        delay(100);
        Display_Screen.Play_Thunder_Picture(7); //单色点阵图案
        delay(10);
        test_read_Us();
        delay(100);
        Display_Screen.Play_Thunder_Picture(8); //单色点阵图案
        test_read_Us();
        delay(100);
        Display_Screen.Play_Thunder_Picture(9); //单色点阵图案
        test_read_Us();
        delay(100);
        Display_Screen.Play_Thunder_Picture(10); //单色点阵图案
        test_read_Us();
        delay(100);
        Display_Screen.Play_Thunder_Picture(11); //单色点阵图案
        test_read_Us();
        delay(100);
        Display_Screen.Play_Thunder_Picture(12); //单色点阵图案
        test_read_Us();
        delay(100);
        Display_Screen.Play_Thunder_Picture(13); //单色点阵图案
        test_read_Us();
        delay(100);
    }
}

void Auto_Test_AllSensors()
{
    Auto_Test_Color();
    Auto_Test_US();
    Auto_Test_Touch();
    Auto_Test_Light();

    Auto_Test_Line();
    Auto_Test_Speaker();

    Serial.println("");
}


void Uart_Echo()
{
    while(!Serial.available()){
    }
    Serial.write(Serial.read());
}
/**************************** 功能函数 ****************************/

void Wait_Command(byte byte_counter)
{
    int command_param_int[2];

    // uint32_t beginWaitTime;
    // beginWaitTime = millis();
    while (Serial.available() < byte_counter)
    {
        // current_time = millis();
        // if(current_time > beginWaitTime + 5000){
        //   break;
        // }
    }
    for (byte i = 0; i < 2; i++)
    {
        command_param_int[i] = Serial.parseInt();
    }

    // parse command
    // Sensor_Color.write(0x41, (byte *)(&command_param_int[0]), 1);
    // Sensor_Color.write(0x42, (byte *)(&command_param_int[1]), 1);
}

void Serial_Ble_Print_Speed()
{
    char speed_info[17];
    memset(speed_info, ' ', sizeof(speed_info));
    // sprintf(speed_info, "INFO: %3d ,%3d \n", Motor_Thunder.Get_Speed_simple(1), Motor_Thunder.Get_Speed_simple(2));
    // sprintf(speed_info, "%7d,%7d\n", Motor_Thunder.Get_RotateValue(1), Motor_Thunder.Get_RotateValue(2));
    int walk_diff;
    walk_diff = Motor_Thunder.Get_RotateValue(1) - Motor_Thunder.Get_RotateValue(2);
    assert(walk_diff == 0);
    sprintf(speed_info, "DiffL-R:%7d\n", walk_diff);

    BLE_ThunderGo.Tx_BLE((uint8_t *)speed_info, sizeof(speed_info) - 1);
}

#endif