#include <bell_thunder.h>

// #define PRODUCE_COLOR
// #define PRODUCE_LIGHT
// #define PRODUCE_BLE_JOY
// #define PRODUCE_COLORLINE
// #define PRODUCE_TOUCH

void Wait_Keypress()
{
    while(1) {
        if(Bell_Thunder.Check_Function_Button_Event(KEY_CLICK_ONE)) {
            return;
        }
    }
}

#ifdef PRODUCE_COLOR
/* 
 * 颜色传感器的治具程序
 * 
 * @parameters: 
 * @return: 
 */
void Produce_Color_Sensor()
{
    unsigned short RGBC[4] = {0};

    // initial LED pins
    pinMode(2, OUTPUT);
    pinMode(0, OUTPUT);
    pinMode(12, OUTPUT);

    while (1)
    {
        if (Sensor_Color.Get_RGBC_Data(RGBC) != 0)
        {
            RGBC[0] = 0;
            RGBC[1] = 0;
            RGBC[2] = 0;
            RGBC[3] = 0;
            Serial.printf("### RGBC Get Fail ####################\n");
        }
        else
        {
            switch (Sensor_Color.Colour_Recognition(RGBC))
            {
            case SENSOR_COLOR::CARD_BLACK:
                Serial.println("*** black card");
                digitalWrite(2, 0);
                digitalWrite(0, 1);
                digitalWrite(12, 1);
                break;
            case SENSOR_COLOR::CARD_WHITE:
                Serial.println("*** white card");
                digitalWrite(2, 1);
                digitalWrite(0, 0);
                digitalWrite(12, 1);
                break;
            case SENSOR_COLOR::CARD_RED:
                Serial.println("*** red card");
                digitalWrite(2, 1);
                digitalWrite(0, 1);
                digitalWrite(12, 1);
                break;
            case SENSOR_COLOR::CARD_BROWN:
                Serial.println("*** brown card");
                digitalWrite(2, 1);
                digitalWrite(0, 1);
                digitalWrite(12, 1);
                break;
            case SENSOR_COLOR::CARD_YELLOW:
                Serial.println("*** yellow card");
                digitalWrite(2, 1);
                digitalWrite(0, 1);
                digitalWrite(12, 0);
                break;
            case SENSOR_COLOR::CARD_GREEN:
                Serial.println("*** green card");
                digitalWrite(2, 1);
                digitalWrite(0, 1);
                digitalWrite(12, 1);
                break;
            case SENSOR_COLOR::CARD_BLUE:
                Serial.println("*** blue card");
                digitalWrite(2, 1);
                digitalWrite(0, 1);
                digitalWrite(12, 1);
                break;
            case SENSOR_COLOR::CARD_NO:
                Serial.println("*** no card");
                digitalWrite(2, 1);
                digitalWrite(0, 1);
                digitalWrite(12, 1);
                break;
            default:
                Serial.println("### bad return!");
                digitalWrite(2, 1);
                digitalWrite(0, 1);
                digitalWrite(12, 1);
                break;
            }
        }
        delay(100);
    }
}
#endif 

#ifdef PRODUCE_LIGHT
void Produce_Light_Sensor()
{
    bool result;
    float lightValue;
    byte errorCode;

    while (1)
    {
        errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
        if (errorCode == 0)
        {
            Serial.println("Light Sensor Connecting...");
            break;
        }

        delay(100);
    }

    Sensor_Light.Set_Led_Brightness(0);
    delay(30);
    errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
    if (errorCode != 0)
        return;
    if (0 < lightValue && lightValue < 20)
    {
        result = true;
    }
    else
    {
        result = false;
        Sensor_Light.Set_Led_Brightness(0);

        // 等待拔掉模块
        while (1)
        {
            errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
            if (errorCode != 0)
            {
                return;
            }
            delay(100);
        }
    }

    delay(300);

    Sensor_Light.Set_Led_Brightness(80);
    delay(30);
    errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
    if (errorCode != 0)
        return;
    if (lightValue < 20)
    {
        result = false;
        Sensor_Light.Set_Led_Brightness(0);

        // 等待拔掉模块
        while (1)
        {
            errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
            if (errorCode != 0)
            {
                return;
            }
            delay(100);
        }
    }
    else
    {
        result = true;
    }

    delay(700);

    if (result == true)
    {
        Sensor_Light.Set_Led_Brightness(5);
        // 等待拔掉模块
        while (1)
        {
            errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
            if (errorCode != 0)
            {
                return;
            }
            delay(100);
        }
    }
}
#endif

#ifdef PRODUCE_BLE_JOY
// 蓝牙手柄的按键测试
void test_BLE_joystick()
{
    if (BLE_Remoter.Check_Key(KEY_UP) == 1)
    {
        Display_Screen.Play_LED_String("UP");
    }
    else if (BLE_Remoter.Check_Key(KEY_DOWN) == 1)
    {
        Display_Screen.Play_LED_String("DW");
    }
    else if (BLE_Remoter.Check_Key(KEY_LEFT) == 1)
    {
        Display_Screen.Play_LED_String("LF");
    }
    else if (BLE_Remoter.Check_Key(KEY_RIGHT) == 1)
    {
        Display_Screen.Play_LED_String("RT");
    }
    else if (BLE_Remoter.Check_Key(KEY_SELECT) == 1)
    {
        Display_Screen.Play_LED_String("I");
    }
    else if (BLE_Remoter.Check_Key(KEY_BACK) == 1)
    {
        Display_Screen.Play_LED_String("II");
    }
    else if (BLE_Remoter.Check_Key(KEY_A) == 1)
    {
        Display_Screen.Play_LED_String("A");
    }
    else if (BLE_Remoter.Check_Key(KEY_B) == 1)
    {
        Display_Screen.Play_LED_String("B");
    }
    else if (BLE_Remoter.Check_Key(KEY_X) == 1)
    {
        Display_Screen.Play_LED_String("C");
    }
    else if (BLE_Remoter.Check_Key(KEY_Y) == 1)
    {
        Display_Screen.Play_LED_String("D");
    }
    else if (BLE_Remoter.Check_Key(KEY_L1) == 1)
    {
        Display_Screen.Play_LED_String("L1");
    }
    else if (BLE_Remoter.Check_Key(KEY_R1) == 1)
    {
        Display_Screen.Play_LED_String("R1");
    }
    else if (BLE_Remoter.Check_Key(KEY_L2) == 1)
    {
        Display_Screen.Play_LED_String("L2");
    }
    else if (BLE_Remoter.Check_Key(KEY_R2) == 1)
    {
        Display_Screen.Play_LED_String("R2");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X) - 100) < 0.00001))
    {
        Display_Screen.Play_LED_String("LFR");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X) - -100) < 0.00001))
    {
        Display_Screen.Play_LED_String("LFL");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y) - 100) < 0.00001))
    {
        Display_Screen.Play_LED_String("LFU");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y) - -100) < 0.00001))
    {
        Display_Screen.Play_LED_String("LFD");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_X) - 100) < 0.00001))
    {
        Display_Screen.Play_LED_String("RTR");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_X) - -100) < 0.00001))
    {
        Display_Screen.Play_LED_String("RTL");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_Y) - 100) < 0.00001))
    {
        Display_Screen.Play_LED_String("RTU");
    }
    else if ((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_Y) - -100) < 0.00001))
    {
        Display_Screen.Play_LED_String("RTD");
    }
    else
    {
        Display_Screen.Play_LED_String("XXX");
    }
}
#endif

#ifdef  PRODUCE_COLORLINE
void Produce_ColorLine_Sensor()
{
    Display_Screen.Play_LED_String("...");
    Wait_Keypress();
    if(0 != Sensor_ColorLight.Calibrate_Sensor(SENSOR_IIC::PORT_1)) {
        Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_ENGLISH_OHNO);
        Display_Screen.Play_LED_String("NO");
        return;
    }
    delay(100);

    Sensor_ColorLight.Set_Operate_Mode(SENSOR_COLORLIGHT::MODE_COLOR, SENSOR_IIC::PORT_1);
    delay(100);

    int check_amount;
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_WHITE == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("1");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_RED == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("2");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_BLACK == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("3");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_GREEN == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("4");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_YELLOW == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("5");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_BLUE == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("6");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_NON == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("7");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_MUSIC_E5);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_WHITE == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 10){
            Display_Screen.Play_LED_String("OK");
            Speaker_Thunder.Play_Song(SPEAKER_THUNDER::SOUND_ENGLISH_GOOD);
            break;
        }
        if(Bell_Thunder.Get_Function_Button_Status()) {
            return;
        }
        delay(10);
    }
    check_amount = 0;
    while(1){
        if(SENSOR_COLORLIGHT::COLOR_NON == Sensor_ColorLight.Get_Result(SENSOR_COLORLIGHT::RESULT_COLOR, SENSOR_IIC::PORT_1)) {
            check_amount++;
        }else {
            check_amount = 0;
        }
        if(check_amount > 20){
            break;
        }
        delay(10);
    }
}
#endif


#ifdef PRODUCE_TOUCH
void Produce_Touch_Sensor()
{
    byte statusValue, errorCode;

    while (1)
    {
        errorCode = Sensor_Touch.Reset_Mode();
        if (errorCode == 0)
            break;

        delay(100);
    }
    Serial.println("Begin Test...");

    delay(200);
    while (1)
    {
        errorCode = Sensor_Touch.Get_Status(&statusValue);
        if (errorCode != 0)
            return;

        if (statusValue == 1)
        {
            delay(50);
            if (statusValue == 1)
                break;
        }
    }

    delay(200);
    while (1)
    {
        errorCode = Sensor_Touch.Get_Status(&statusValue);
        if (errorCode != 0)
            return;

        if (statusValue == 0)
        {
            delay(50);
            if (statusValue == 0)
                break;
        }
    }

    Sensor_Touch.Set_LED_RGBvalue(0, 0, 255);
    Serial.println("blue LED");

    delay(200);
    while (1)
    {
        errorCode = Sensor_Touch.Get_Status(&statusValue);
        if (errorCode != 0)
            return;

        if (statusValue == 1)
        {
            delay(50);
            if (statusValue == 1)
                break;
        }
    }

    Sensor_Touch.Set_LED_RGBvalue(255, 255, 255);
    Serial.println("white LED");

    delay(200);
    while (1)
    {
        errorCode = Sensor_Touch.Get_Status(&statusValue);
        if (errorCode != 0)
            return;

        if (statusValue == 0)
        {
            delay(50);
            if (statusValue == 0)
                break;
        }
    }
}

#endif