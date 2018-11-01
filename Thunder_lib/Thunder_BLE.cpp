/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 蓝牙库文件
 * 
 *   创建日期： 20180606
 *   作者：     宋博伟
 *   邮箱：     songbw123@163.com
 *
 *   版本：     v0.2
 *   修改日期   20180721
 *   修改：     宋博伟
 *   邮箱：     songbw123@163.com
 *   修改内容： 
 * 
 *   
 * 
 * 功能列表：
 *  1.  uint64_t Get_ID(void);                        // 获取并返回芯片ID
 *  2.  void Setup_EEPROM(void);                      // 配置EEPROM
 *  3.  void Read_BLE_Name (int addr);                // 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID
 *  4.  void Write_BLE_Name (int addr);               // 写入自定义蓝牙名称
 *  5.  void Write_ROM (int addr, int val);           // 写ROM
 *  6.  void Reset_ROM (void);                        // 重置ROM
 *  7.  void Setup_BLE(void);                         // 配置BLE
 *  8.  void Tx_BLE(uint8_t Tx_Data[], int byte_num); // 发送蓝牙数据
 * 
 ************************************************/

#include <Thunder_BLE.h>

// 蓝牙连接/断开的回调函数
class MyServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
      deviceConnected = true;
      Serial.printf("SSS___ 蓝牙连接成功 ___SSS\n");
      // Thunder_Motor.Setup_PID_Timer();
    };

    void onDisconnect(BLEServer* pServer) 
    {
      deviceConnected = false;
      Serial.printf("SSS___ 断开蓝牙连接 ___SSS\n");
      Thunder.Stop_All();
      Speaker.Play_Song(45);   // 断开声音
    }
};

// 接收蓝牙指令的回调函数
uint8_t BLE_Name_Data[30] = {0};
uint8_t BLE_Name_Length = 0;
class MyCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
      std::string rxValue = pCharacteristic->getValue();  // 接收到的存入rxValue 最多20个8位

      if (rxValue.length() > 0) 
      {
        Serial.printf("SSS___ 收到蓝牙指令: %x ___SSS\n",rxValue[0]);
        
        if(rxValue[0] == 0xA1)  // 蓝牙命名指令数据
        {
          uint8_t SUM = 0;
          Rx_Data[0] = rxValue[0];
          for (int i = 1; i < rxValue.length(); i++)
          {
            BLE_Name_Data[i-1] = rxValue[i];
            SUM += rxValue[i - 1];
          }
          BLE_Name_Length = rxValue.length() - 2;

          if(SUM != rxValue[rxValue.length()-1])
          {
            Rx_Data[0] = 0;
            Serial.printf("\nSSS___ SUM error 0xA1 ___SSS\n");
            Serial.printf("SSS___ SUM: %x ___SSS\n",SUM);
            Serial.printf("SSS___ rxValue[rxValue.length()-1]: %x ___SSS\n",rxValue[rxValue.length()-1]);
          }
        }
        else if(rxValue[0] == 0xC2)   // 刷新左侧彩色灯
        {
          uint8_t SUM = rxValue[0];
          Rx_Data[0] = rxValue[0];
          for (int i = 1; i < 19; i++)
          {
            Thunder.I2C_LED_BUFF1[i-1] = rxValue[i];
            SUM += rxValue[i];
          }

          if(SUM != rxValue[19])
          {
            Rx_Data[0] = 0;
            Serial.printf("\nSSS___ SUM error 0xC2 ___SSS\n");
            Serial.printf("SSS___ SUM: %x ___SSS\n",SUM);
            Serial.printf("SSS___ rxValue[rxValue.length()-1]: %x ___SSS\n",rxValue[rxValue.length()-1]);
          }
        }
        else if(rxValue[0] == 0xC3)   // 刷新右侧彩色灯
        {
          uint8_t SUM = rxValue[0];
          Rx_Data[0] = rxValue[0];
          for (int i = 1; i < 19; i++)
          {
            Thunder.I2C_LED_BUFF2[i-1] = rxValue[i];
            SUM += rxValue[i];
          }

          if(SUM != rxValue[19])
          {
            Rx_Data[0] = 0;
            Serial.printf("\nSSS___ SUM error 0xC3 ___SSS\n");
            Serial.printf("SSS___ SUM: %x ___SSS\n",SUM);
            Serial.printf("SSS___ rxValue[rxValue.length()-1]: %x ___SSS\n",rxValue[rxValue.length()-1]);
          }
        }
        else if(rxValue[0] == 0xD3)   // 单色点阵灯一次性刷新前半部分灯
        {
          uint8_t SUM = rxValue[0];
          Rx_Data[0] = rxValue[0];
          for (int i = 1; i < 15; i++)
          {
            Thunder.LED_BUFF_Dot[i] = rxValue[i];
            SUM += rxValue[i];
          }

          if(SUM != rxValue[15])
          {
            Rx_Data[0] = 0;
            Serial.printf("\nSSS___ SUM error 0xD3 ___SSS\n");
            Serial.printf("SSS___ SUM: %x ___SSS\n",SUM);
            Serial.printf("SSS___ rxValue[rxValue.length()-1]: %x ___SSS\n",rxValue[rxValue.length()-1]);
          }
        }
        else if(rxValue[0] == 0xD4)   // 单色点阵灯一次性刷新后半部分灯
        {
          uint8_t SUM = rxValue[0];
          Rx_Data[0] = rxValue[0];
          for (int i = 1; i < 15; i++)
          {
            Thunder.LED_BUFF_Dot[i+14] = rxValue[i];
            SUM += rxValue[i];
          }

          if(SUM != rxValue[15])
          {
            Rx_Data[0] = 0;
            Serial.printf("\nSSS___ SUM error 0xD4 ___SSS\n");
            Serial.printf("SSS___ SUM: %x ___SSS\n",SUM);
            Serial.printf("SSS___ rxValue[rxValue.length()-1]: %x ___SSS\n",rxValue[rxValue.length()-1]);
          }
        }
        else  // 一般指令数据
        {
          for (int i = 0; i < rxValue.length(); i++)
          {
            Rx_Data[i] = rxValue[i];
          }
          if(Rx_Data[5] != (uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]))
          {
            Serial.printf("SSS__ SUM error __ Rx_Data[5]: %x __ sum: %x __SSS\n",Rx_Data[5],(uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]));
            Thunder.Reset_Rx_Data();
          }
        }
      }
    }
};

// 获取并返回芯片ID
uint64_t THUNDER_BLE::Get_ID(void)
{
  SEP32_ID = ESP.getEfuseMac();   // 获取ESP32芯片ID
  
  Serial.printf(" 设备 ID : %04X\n",(uint16_t)(SEP32_ID>>32)); // 打印芯片ID
  // Serial.printf("%08X\n",(uint32_t)SEP32_ID);
  
  return SEP32_ID;
}

// 配置EEPROM
void THUNDER_BLE::Setup_EEPROM(void)
{
  Serial.println("开始配置 EEPROM...");

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("初始化 EEPROM 失败！"); 
    delay(100000);
  }
  
  Serial.print(" EEPROM_SIZE : ");
  Serial.println(EEPROM_SIZE);
  
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    Serial.print(" ");
    Serial.print(byte(EEPROM.read(i)));
  }
  Serial.println();

  Serial.println(" 配置EEPROM完成");
}

// 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID
void THUNDER_BLE::Read_BLE_Name (int addr)
{
  if(EEPROM.read(addr) != 0xFF)
  {
    BLE_Named = 1;
  }
  else
  {
    SEP32_ID = Get_ID();  // 获取芯片ID
  }
}

// 写入自定义蓝牙名称
void THUNDER_BLE::Write_BLE_Name (int addr)
{
  Reset_ROM();  // 清空定义出来的ROM

  for(int i=addr; *(BLE_Name_Data+i) != '\0'; i++)
  {
    Write_ROM (i, *(BLE_Name_Data+i)); // 参数1：地址；参数2：数据
  }
  Write_ROM (BLE_Name_Length, '\0');   // 参数1：地址；参数2：数据
}

// 写ROM
void THUNDER_BLE::Write_ROM (int addr, int val)
{  
  // Serial.println("Start Write_ROM...");
  
  EEPROM.write(addr, val);
  EEPROM.commit();

  // Serial.println("Finish Write_ROM");
  
  // Serial.print(" bytes read from ROM . Values is: ");
  // Serial.print(byte(EEPROM.read(addr)));
  // Serial.println();   
}

// 重置ROM
void THUNDER_BLE::Reset_ROM ()
{
  // int val = 255;

  Serial.println("开始重置ROM...");

  for(int i = 0; i < EEPROM_SIZE; i++)
  {
    EEPROM.write(i, 255);
  }
  EEPROM.commit();

  Serial.println(" 重置ROM完成");
}

// 配置BLE
void THUNDER_BLE::Setup_BLE()
{
  Serial.printf("开始配置蓝牙...\n");

  Read_BLE_Name(ADD_BLE_NAME);  // 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID

  if(BLE_Named == 1)
  {
    Serial.printf(" 设备蓝牙已命名\n");

    char buf[32] = "";

    for(int i=0; byte(EEPROM.read(i)) != '\0'; i++)
    {
      Serial.printf("SSS___ byte(EEPROM.read(%d) : %d ___SSS\n",i,byte(EEPROM.read(i)));

      buf[i] = byte(EEPROM.read(i));
    }
    BLEDevice::init(buf);  // 创建BLE设备并命名 最多26个字母可以，一个汉字对应3个字母
  }
  else
  {
    Serial.printf(" 设备蓝牙未命名\n");

    char buf[5];
    sprintf(buf, "%X", (uint16_t)(SEP32_ID>>32));   // 取芯片ID后4位
    User_BLE_Name += buf;  // 结尾添加芯片ID后4位
    BLEDevice::init(User_BLE_Name);  // 创建BLE设备并命名 最多26个字母可以，一个汉字对应3个字母
  }

  BLEServer *pServer = BLEDevice::createServer(); // 创建BLE服务器
  pServer->setCallbacks(new MyServerCallbacks()); // 如果连接上蓝牙，deviceConnected置1

  BLEService *pService = pServer->createService(SERVICE_UUID);  // 创建BLE服务
  
  // 改蓝牙广播功率  最小： ESP_PWR_LVL_N14 ， 最大： ESP_PWR_LVL_P7
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P7);      
  // 改蓝牙默认功率  最小： ESP_PWR_LVL_N14 ， 最大： ESP_PWR_LVL_P7
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P7);  

  // 增加广播内容
  pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setPartialServices(BLEUUID(SERVICE_UUID));     // 广播UUIDs
  oAdvertisementData.setPartialServices(BLEUUID(ADVERTISING_UUID)); // 广播UUIDs

  ////////////////////////////////////////////////////////////////////////////
  std::string strServiceData = "";
  
  strServiceData += (char)02;     // Len
  strServiceData += (char)0x01;   // Type
  strServiceData += (char)0x06;
  oAdvertisementData.addData(strServiceData);

  // pAdvertising->setAdvertisementData(oAdvertisementData);
  /////////////////////////////////////////////////////////////////////////////

  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->start();


  // 创建BLE特征
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR );
  // pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  // BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE_NR);   //PROPERTY_READ PROPERTY_WRITE  PROPERTY_NOTIFY PROPERTY_BROADCAST  PROPERTY_INDICATE PROPERTY_WRITE_NR
  // BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks()); // 接收到数据后执行的内容

  pService->start();  // 开启服务

  pServer->getAdvertising()->start(); // 开始广播

  Serial.printf(" 蓝牙配置完成\n");
}

// 发送蓝牙数据
// 参数1-->要发送的地址
// 参数2-->发送的字节数
void THUNDER_BLE::Tx_BLE(uint8_t Tx_Data[], int byte_num)
{
  if (deviceConnected) 
  {
    Serial.printf("\nSSS___ 发送蓝牙指令: %x ___SSS\n", Tx_Data[0]);

    //发送
    pCharacteristic->setValue(&Tx_Data[0], byte_num); 
    pCharacteristic->notify();
  }
  else
  {
    Serial.printf("蓝牙未连接...\n");
  }
}
