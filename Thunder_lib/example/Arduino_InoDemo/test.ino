#if 0
void test_read_Us(){
    float US_Data_cm = 0;

    US_Data_cm = Sensor_Ultrasonic.Get_US_cm();
    if (US_Data_cm < 1.0){
        // Serial.printf("### US_Data_cm : %.1f [cm]###############1################\n", US_Data_cm);
    }else{
        // Serial.printf("*** US_Data_cm : %.1f [cm]***\n", US_Data_cm);
    }
}

// 测试超声波 无延时的读取数据
void test_Us_QuickRead()
{
  while (1)
  {
    Display_Screen.Play_LED_HT16F35B_Show(6); //单色点阵图案
    delay(100);
    Display_Screen.Play_LED_HT16F35B_Show(7); //单色点阵图案
    delay(10);
    test_read_Us();
    delay(100);
    Display_Screen.Play_LED_HT16F35B_Show(8); //单色点阵图案
    test_read_Us();
    delay(100);
    Display_Screen.Play_LED_HT16F35B_Show(9); //单色点阵图案
    test_read_Us();
    delay(100);
    Display_Screen.Play_LED_HT16F35B_Show(10); //单色点阵图案
    test_read_Us();
    delay(100);
    Display_Screen.Play_LED_HT16F35B_Show(11); //单色点阵图案
    test_read_Us();
    delay(100);
    Display_Screen.Play_LED_HT16F35B_Show(12); //单色点阵图案
    test_read_Us();
    delay(100);
    Display_Screen.Play_LED_HT16F35B_Show(13); //单色点阵图案
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

    Test_Line_Sensor();
    Auto_Test_Speaker();

    Serial.println("");
}

void Test_IIC_BUS_Busy()
{
    byte colorData_1[18] = {0x16, 0x13, 0x9B, 0xB6, 0xB4, 0xF5, 0x84, 0x81, 0xEF, 0x5A, 0x56, 0xEB, 0x2C, 0x27, 0xE4, 0x1D, 0x18, 0xCD};
    byte colorData_2[18] = {0x16, 0x13, 0x9B, 0xB6, 0xB4, 0xF5, 0x84, 0x81, 0xEF, 0x5A, 0x56, 0xEB, 0x2C, 0x27, 0xE4, 0x1D, 0x18, 0xCD};
    LED_Color.Set_LEDs_Data(0x01, colorData_1, sizeof(colorData_1));
    LED_Color.Set_LEDs_Data(0x07, colorData_2, sizeof(colorData_2));
    LED_Color.LED_Updata();
    
    uint8_t LED_BUFF[29] =  
    {
        0x00, //地址
        0x20, 0x21, 0x22, 0x24, 0x38, 0x00, 0x3F, 0x20, 0x20, 0x20, 0x3F, 0x00, 0x10, 0x20, 0x3F, 0x00, 0x00, 0x00,
        0xC4, 0x44, 0x40, 0xC4, 0x44, 0xC0, 0x44, 0xC4, 0x40,
        0x00  //不用的
    };
    Display_Screen.LED_Show(LED_BUFF, sizeof(LED_BUFF));

    delay(5);
}

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

  while(1){
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
      switch( Sensor_Color.Colour_Recognition(RGBC) ){
        case BLACK_CARD:
          Serial.println("*** black card");
          digitalWrite(2,0);
          digitalWrite(0,1);
          digitalWrite(12,1);
          break;
        case WHITE_CARD:
          Serial.println("*** white card");
          digitalWrite(2,1);
          digitalWrite(0,0);
          digitalWrite(12,1);
          break;
        case RED_CARD:
          Serial.println("*** red card");
          digitalWrite(2,1);
          digitalWrite(0,1);
          digitalWrite(12,1);
          break;
        case BROWN_CARD:
          Serial.println("*** brown card");
          digitalWrite(2,1);
          digitalWrite(0,1);
          digitalWrite(12,1);
          break;
        case YELLOW_CARD:
          Serial.println("*** yellow card");
          digitalWrite(2,1);
          digitalWrite(0,1);
          digitalWrite(12,0);
          break;
        case GREEN_CARD:
          Serial.println("*** green card");
          digitalWrite(2,1);
          digitalWrite(0,1);
          digitalWrite(12,1);
          break;
        case BLUE_CARD:
          Serial.println("*** blue card");
          digitalWrite(2,1);
          digitalWrite(0,1);
          digitalWrite(12,1);
          break;
        case NO_CARD:
          Serial.println("*** no card");
          digitalWrite(2,1);
          digitalWrite(0,1);
          digitalWrite(12,1);
          break;
        default:
          Serial.println("### bad return!");
          digitalWrite(2,1);
          digitalWrite(0,1);
          digitalWrite(12,1);
          break;
      }
    }
    delay(100);
  }
}

void Produce_Touch_Sensor()
{
  byte statusValue, errorCode;

  while(1){
    errorCode = Sensor_Touch.Reset_Mode();
    if(errorCode == 0) break;
    
    delay(100);
  }
  Serial.println("Begin Test...");

  delay(200);
  while(1){
    errorCode = Sensor_Touch.Get_Status(&statusValue);
    if(errorCode != 0) return;

    if(statusValue == 1){
      delay(50);
      if(statusValue == 1) break;
    }
  }
  
  delay(200);
  while(1){
    errorCode = Sensor_Touch.Get_Status(&statusValue);
    if(errorCode != 0) return;

    if(statusValue == 0){
      delay(50);
      if(statusValue == 0) break;
    }
  }

  Sensor_Touch.Set_LED_RGBvalue(0, 0, 255);
  Serial.println("blue LED");

  delay(200);
  while(1){
    errorCode = Sensor_Touch.Get_Status(&statusValue);
    if(errorCode != 0) return;

    if(statusValue == 1){
      delay(50);
      if(statusValue == 1) break;
    }
  }

  Sensor_Touch.Set_LED_RGBvalue(255, 255, 255);
  Serial.println("white LED");
  
  delay(200);
  while(1){
    errorCode = Sensor_Touch.Get_Status(&statusValue);
    if(errorCode != 0) return;
  
    if(statusValue == 0){
      delay(50);
      if(statusValue == 0) break;
    }
  }

}

void Produce_Light_Sensor()
{
  bool result;
  float lightValue;
  byte errorCode;
  
  while(1){
    errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
    if(errorCode == 0){
      Serial.println("Light Sensor Connecting...");
      break;
    }

    delay(100);
  }

  Sensor_Light.Set_Led_Brightness(0);
  delay(30);
  errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
  if(errorCode != 0) return;
  if( 0 < lightValue && lightValue < 20 ){
    result = true;
  }else{
    result = false;
    Sensor_Light.Set_Led_Brightness(0);
    
    // 等待拔掉模块
    while(1){
      errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
      if(errorCode != 0){
        return;
      }
      delay(100);
    }
  }

  delay(300);
  
  Sensor_Light.Set_Led_Brightness(80);
  delay(30);
  errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
  if(errorCode != 0) return;
  if( lightValue < 20 ){
    result = false;
    Sensor_Light.Set_Led_Brightness(0);
    
    // 等待拔掉模块
    while(1){
      errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
      if(errorCode != 0){
        return;
      }
      delay(100);
    }
  }else{
    result = true;
  }

  delay(700);

  if(result == true){
    Sensor_Light.Set_Led_Brightness(5);
    // 等待拔掉模块
    while(1){
      errorCode = Sensor_Light.Get_Light_Value_original(&lightValue);
      if(errorCode != 0){
        return;
      }
      delay(100);
    }
  }
}

// 蓝牙手柄的按键测试
void test_BLE_joystich()
{
  if(BLE_Remoter.Get_Key_Value(KEY_UP) == 1) {
      Display_Screen.Play_LED_String("UP");
  } else if(BLE_Remoter.Get_Key_Value(KEY_DOWN) == 1) {
      Display_Screen.Play_LED_String("DW");
  } else if(BLE_Remoter.Get_Key_Value(KEY_LEFT) == 1) {
      Display_Screen.Play_LED_String("LF");
  } else if(BLE_Remoter.Get_Key_Value(KEY_RIGHT) == 1) {
      Display_Screen.Play_LED_String("RT");
  } else if(BLE_Remoter.Get_Key_Value(KEY_SELECT) == 1) {
      Display_Screen.Play_LED_String("I");
  } else if(BLE_Remoter.Get_Key_Value(KEY_BACK) == 1) {
      Display_Screen.Play_LED_String("II");
  } else if(BLE_Remoter.Get_Key_Value(KEY_A) == 1) {
      Display_Screen.Play_LED_String("A");
  } else if(BLE_Remoter.Get_Key_Value(KEY_B) == 1) {
      Display_Screen.Play_LED_String("B");
  } else if(BLE_Remoter.Get_Key_Value(KEY_X) == 1) {
      Display_Screen.Play_LED_String("C");
  } else if(BLE_Remoter.Get_Key_Value(KEY_Y) == 1) {
      Display_Screen.Play_LED_String("D");
  } else if(BLE_Remoter.Get_Key_Value(KEY_L1) == 1) {
      Display_Screen.Play_LED_String("L1");
  } else if(BLE_Remoter.Get_Key_Value(KEY_R1) == 1) {
      Display_Screen.Play_LED_String("R1");
  } else if(BLE_Remoter.Get_Key_Value(KEY_L2) == 1) {
      Display_Screen.Play_LED_String("L2");
  } else if(BLE_Remoter.Get_Key_Value(KEY_R2) == 1) {
      Display_Screen.Play_LED_String("R2");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X) - 100) < 0.00001)) {
      Display_Screen.Play_LED_String("LFR");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_X) - -100) < 0.00001)) {
      Display_Screen.Play_LED_String("LFL");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y) - 100) < 0.00001)) {
      Display_Screen.Play_LED_String("LFU");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_L_Y) - -100) < 0.00001)) {
      Display_Screen.Play_LED_String("LFD");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_X) - 100) < 0.00001)) {
      Display_Screen.Play_LED_String("RTR");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_X) - -100) < 0.00001)) {
      Display_Screen.Play_LED_String("RTL");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_Y) - 100) < 0.00001)) {
      Display_Screen.Play_LED_String("RTU");
  } else if((abs(BLE_Remoter.Get_Control_Value(KEY_ROCKER_R_Y) - -100) < 0.00001)) {
      Display_Screen.Play_LED_String("RTD");
  } else {
      Display_Screen.Play_LED_String("XXX");
  }
}

#endif