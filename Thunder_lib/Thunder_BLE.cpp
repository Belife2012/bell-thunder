#include "esp_spi_flash.h"
#include "esp_task_wdt.h"
#include "Thunder_lib.h"
#include "Thunder_BLE.h"
#include "function_macro.h"
#include "Disk_Manager.h"

uint8_t BLE_Name_Data[BLE_NAME_SIZE] = {0x00};
QueueHandle_t THUNDER_BLE::Queue_Semaphore_BLE;
enum_Ble_Status THUNDER_BLE::ble_connect_type = BLE_NOT_OPEN;

// 蓝牙连接/断开的回调函数
class MyServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
      deviceConnected = true;
      Serial.printf("* BLE server Connected\n");
      
      pServer->getAdvertising()->stop();
    };

    void onDisconnect(BLEServer* pServer)
    {
      deviceConnected = false;
      Serial.printf("# BLE server DisConnect\n");
      Thunder.Stop_All();
      Speaker.Play_Song(45);   // 断开声音

      pServer->getAdvertising()->start();
    }
};

void Analyze_BLE_Data(std::string &recv_data)
{
  #ifdef DEBUG_BLE_COMMAND
  Serial.printf("\n*>%x:%x %x %x %x*\n",recv_data[0],
                recv_data[1],recv_data[2],recv_data[3],recv_data[4]);
  #endif
  
  uint8_t SUM = 0;
  int i;
  switch(recv_data[0]){
    case UART_GENERAL_BLE_NAME:  // 蓝牙命名指令数据
    {
      Rx_Data[0] = recv_data[0];
      for (i = 1; i < recv_data.length() - 1; i++)
      {
        if(i < BLE_NAME_SIZE){
          BLE_Name_Data[i-1] = recv_data[i];
          BLE_Name_Data[i] = '\0';
        }
        SUM += recv_data[i - 1];
      }
      SUM += recv_data[i - 1];

      if(SUM != recv_data[recv_data.length()-1])
      {
        Rx_Data[0] = 0;
        Serial.printf(" # %x SUM error: %x #\n", recv_data[0], SUM);
        Thunder.Reset_Rx_Data();
      }
      break;
    }
    case UART_GENERAL_LEFT_RGBLED:   // 刷新左侧彩色灯
    {
      SUM = recv_data[0];
      Rx_Data[0] = recv_data[0];
      for (i = 1; i < 19; i++)
      {
        Thunder.Color_LED_BUFF1[i-1] = recv_data[i];
        SUM += recv_data[i];
      }

      if(SUM != recv_data[19])
      {
        Rx_Data[0] = 0;
        Serial.printf(" # %x SUM error: %x #\n", recv_data[0], SUM);
        Thunder.Reset_Rx_Data();
      }
      break;
    }
    case UART_GENERAL_RIGHT_RGBLED:   // 刷新右侧彩色灯
    {
      SUM = recv_data[0];
      Rx_Data[0] = recv_data[0];
      for (i = 1; i < 19; i++)
      {
        Thunder.Color_LED_BUFF2[i-1] = recv_data[i];
        SUM += recv_data[i];
      }

      if(SUM != recv_data[19])
      {
        Rx_Data[0] = 0;
        Serial.printf(" # %x SUM error: %x #\n", recv_data[0], SUM);
        Thunder.Reset_Rx_Data();
      }
      break;
    }
    case UART_GENERAL_DEBUG_PRE_LED:   // 单色点阵灯一次性刷新前半部分灯
    {
      SUM = recv_data[0];
      Rx_Data[0] = recv_data[0];
      for (i = 1; i < 15; i++)
      {
        Thunder.LED_BUFF_Dot[i] = recv_data[i];
        SUM += recv_data[i];
      }

      if(SUM != recv_data[15])
      {
        Rx_Data[0] = 0;
        Serial.printf(" # %x SUM error: %x #\n", recv_data[0], SUM);
        Thunder.Reset_Rx_Data();
      }
      break;
    }
    case UART_GENERAL_DEBUG_SUF_LED:   // 单色点阵灯一次性刷新后半部分灯
    {
      SUM = recv_data[0];
      Rx_Data[0] = recv_data[0];
      for (i = 1; i < 15; i++)
      {
        Thunder.LED_BUFF_Dot[i+14] = recv_data[i];
        SUM += recv_data[i];
      }

      if(SUM != recv_data[15])
      {
        Rx_Data[0] = 0;
        Serial.printf(" # %x SUM error: %x #\n", recv_data[0], SUM);
        Thunder.Reset_Rx_Data();
      }
      break;
    }
    case UART_CALL_SPECIAL_FUNCTION:
    {
      int last_index;

      Rx_Data[COMMUNICATION_DATA_LENGTH_MAX] = recv_data.length();
      last_index = Rx_Data[COMMUNICATION_DATA_LENGTH_MAX] - 1;
      if( Rx_Data[COMMUNICATION_DATA_LENGTH_MAX] <=  COMMUNICATION_DATA_LENGTH_MAX){
        for (i = 0; i < last_index; i++)
        {
          Rx_Data[i] = recv_data[i];
          SUM += Rx_Data[i];
        }
        if(SUM != recv_data[last_index]){
          Serial.printf("* F1 SUM: %x *\n", SUM);
          Thunder.Reset_Rx_Data();
        }else{
          Rx_Data[last_index] = recv_data[last_index];
        }
      }
      break;
    }
    default:  // 一般指令数据
    {
      for (i = 0; i < recv_data.length(); i++)
      {
        Rx_Data[i] = recv_data[i];

        if(i >= (sizeof(Rx_Data) - 1)) break;
      }
      if(Rx_Data[5] != (uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]))
      {
        Serial.printf(" # SUM error: Rx_Data[5]=%x calSum=%x #\n",Rx_Data[5],(uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]));
        Thunder.Reset_Rx_Data();
      }

      break;
    }
  }
}

// 接收蓝牙指令的回调函数
class MyCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
      std::string rxValue = pCharacteristic->getValue();  // 接收到的存入rxValue 最多20个8位
      if (rxValue.length() > 0 && !ble_command_busy){
        ble_command_busy = true;
        Analyze_BLE_Data(rxValue);

        // 分析BLE数据后，如果指令Rx_Data[0] 不为 0 ，则give BLE信号
        if(Rx_Data[0] != 0){
          THUNDER_BLE::Give_Semaphore_BLE(BLE_SERVER_SEMAPHORE_RX);
        }
      }
    }
};

// 配置BLE
void THUNDER_BLE::Setup_BLE()
{
  Serial.printf("\nstart BLE ...\n");

  Get_BLE_Name();  // 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID

  if(BLE_Named == 1)
  {
    Serial.printf("* Dev BLE named: ");

    int i;
    char buf[BLE_NAME_SIZE] = "";
    for(i=0; i < BLE_NAME_SIZE; i++)
    {
      if( BLE_Name_Data[i] == '\0' || i == (BLE_NAME_SIZE - 1) ){
        buf[i] = '\0';
        break;
      }

      // 如果出现控制字符，则意味着EEPROM里面的信息不准确，BLE命名为 "Thunder_"+ID
      if(isControl(BLE_Name_Data[i])){
        buf[0] = 'T';buf[1] = 'h';buf[2] = 'u';buf[3] = 'n';buf[4] = 'd';buf[5] = 'e';
        buf[6] = 'r';buf[7] = '_';
        Get_ID();
        sprintf(&buf[8], "%X", (uint16_t)(SEP32_ID>>32));
        buf[12] = '\0';
        break;
      }

      buf[i] = BLE_Name_Data[i];
    }
    // 如果第一个字符串是字符串结束符
    if(buf[0] == '\0'){
      buf[0] = 'T';buf[1] = 'h';buf[2] = 'u';buf[3] = 'n';buf[4] = 'd';buf[5] = 'e';
      buf[6] = 'r';buf[7] = '_';
      Get_ID();
      sprintf(&buf[8], "%X", (uint16_t)(SEP32_ID>>32));
      buf[12] = '\0';
    }
    Serial.printf("%s\n", buf);

    BLEDevice::init(buf);  // 创建BLE设备并命名 最多26个字母可以，一个汉字对应3个字母
  }
  else
  {
    char buf[5];
    sprintf(buf, "%X", (uint16_t)(SEP32_ID>>32));   // 取芯片ID后4位
    User_BLE_Name += buf;  // 结尾添加芯片ID后4位
    
    Serial.printf("* default BLE name: %s\n", User_BLE_Name.c_str());
    BLEDevice::init(User_BLE_Name);  // 创建BLE设备并命名 最多26个字母可以，一个汉字对应3个字母
  }

}

void THUNDER_BLE::Start_Advertisement()
{
  if(pServer == NULL)
  {
    pServer = BLEDevice::createServer(); // 创建BLE服务器
    pServer->setCallbacks(new MyServerCallbacks()); // 如果连接上蓝牙，deviceConnected置1
      
    // 改蓝牙广播功率  最小： ESP_PWR_LVL_N14 ， 最大： ESP_PWR_LVL_P7
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P7);      
    // 改蓝牙默认功率  最小： ESP_PWR_LVL_N14 ， 最大： ESP_PWR_LVL_P7
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P7);  

    Setup_Ble_Security();

    // 增加广播内容
    pAdvertising = pServer->getAdvertising();
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    oAdvertisementData.setPartialServices(BLEUUID(SERVICE_UUID));     // 广播UUIDs
    oAdvertisementData.setPartialServices(BLEUUID(ADVERTISING_UUID)); // 广播UUIDs

    ////////////////////////////////////////////////////////////////////////////
    std::string strServiceData = "";
    
    /* 广播包单元的格式：长度+类型+数据 */
    strServiceData += (char)02;     // Len
    strServiceData += (char)0x01;   // Type
    strServiceData += (char)0x06;
    oAdvertisementData.addData(strServiceData);

    // pAdvertising->setAdvertisementData(oAdvertisementData);
    /////////////////////////////////////////////////////////////////////////////

    Serial.println("AdvertisementData:");
    for(int i=0; i<oAdvertisementData.getPayload().length(); i++){
      Serial.printf( "0x%02x ", (int)( (oAdvertisementData.getPayload().c_str())[i] ) );
    }
    Serial.println();

    pAdvertising->setAdvertisementData(oAdvertisementData);
    // pAdvertising->start();

  }
  if(pService == NULL)
  {
    pService = pServer->createService(SERVICE_UUID);  // 创建BLE服务
    // 创建BLE特征
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, 
                    BLECharacteristic::PROPERTY_NOTIFY 
                    | BLECharacteristic::PROPERTY_READ 
                    | BLECharacteristic::PROPERTY_WRITE 
                    | BLECharacteristic::PROPERTY_WRITE_NR );
    // pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->addDescriptor(new BLE2902());
    // BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE_NR);   //PROPERTY_READ PROPERTY_WRITE  PROPERTY_NOTIFY PROPERTY_BROADCAST  PROPERTY_INDICATE PROPERTY_WRITE_NR
    // BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE);
    pCharacteristic->setCallbacks(new MyCallbacks()); // 接收到数据后执行的内容

    pService->start();  // 开启服务
  }
  
  pServer->getAdvertising()->start(); // 开始广播

  Serial.printf("Open BLE Server\n");
}

void THUNDER_BLE::Delete_Ble_Server_Service()
{
  if(pServer == NULL || pService == NULL){
    return;
  }

  pServer->getAdvertising()->stop();
  // pServer->removeService(pService);
  // pService = NULL;
}

void THUNDER_BLE::Setup_Ble_Security()
{
  BLESecurity ble_security;

  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

  //set static passkey
  uint32_t passkey = 123456;
  esp_ble_gap_set_security_param(ESP_BLE_SM_PASSKEY, &passkey, sizeof(uint32_t));
  ble_security.setAuthenticationMode(ESP_LE_AUTH_BOND);
  ble_security.setCapability(ESP_IO_CAP_NONE);
  ble_security.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  ble_security.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  ble_security.setKeySize();

}

// 发送蓝牙数据
// 参数1-->要发送的地址
// 参数2-->发送的字节数
void THUNDER_BLE::Tx_BLE(uint8_t Tx_Data[], int byte_num)
{
  if (deviceConnected) 
  {
#ifdef DEBUG_BLE_COMMAND
    Serial.printf("*<%x*\n", Tx_Data[0]);
#endif

    //发送
    pCharacteristic->setValue(&Tx_Data[0], byte_num); 
    pCharacteristic->notify();
  }
  else
  {
#ifdef DEBUG_BLE_COMMAND
    Serial.printf("# BLE DisConnected...\n");
#endif
  }

}

// 获取并返回芯片ID
uint64_t THUNDER_BLE::Get_ID(void)
{
  SEP32_ID = ESP.getEfuseMac();   // 获取ESP32芯片ID
  
  Serial.printf(" Dev ID : %04X\n",(uint16_t)(SEP32_ID>>32)); // 打印芯片ID
  // Serial.printf("%08X\n",(uint32_t)SEP32_ID);
  
  return SEP32_ID;
}

// 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID
void THUNDER_BLE::Get_BLE_Name ()
{
  Disk_Manager.Read_Ble_Name(BLE_Name_Data);

  if(BLE_Name_Data[0] != 0xFF)
  {
    BLE_Named = 1;
  }
  else
  {
    SEP32_ID = Get_ID();  // 获取芯片ID
  }
}

#define RTC_CNTL_OPTIONS0_REG     0x3ff48000
#define RTC_CNTL_SW_SYS_RST       0x80000000
// 写入自定义蓝牙名称
void THUNDER_BLE::Set_BLE_Name ()
{
  if( true == Disk_Manager.Write_Ble_Name(BLE_Name_Data) ){
    Serial.print("\nBLE w name: "); Serial.printf("%s\n", BLE_Name_Data);
    Serial.println("BLE rename Success!");
  }
  // 断开蓝牙连接，使重命名有效
  Serial.print("Device reset...");
  delay(50);
  
  *((UBaseType_t *)RTC_CNTL_OPTIONS0_REG) |= RTC_CNTL_SW_SYS_RST;
}

void THUNDER_BLE::CreateQueueBLE()
{
  Queue_Semaphore_BLE = xQueueCreate(1, sizeof(int));
  if (Queue_Semaphore_BLE == NULL)
  {
    while (1)
    {
      Serial.println("Semaphore_BLE create fail");
    }
  }
}

/* 
 * 处理BLE command前需要take xSemaphore_BLE
 * 
 * @parameters:
 * @return
 */
int THUNDER_BLE::Take_Semaphore_BLE()
{
  int recv;

  do
  {
  } while ( xQueueReceive(Queue_Semaphore_BLE, &recv, portMAX_DELAY) != pdTRUE );

  return recv;
}

/* 
 * BLECharacteristicCallbacks onWrite函数 give xSemaphore_BLE
 * 
 * @parameters:
 * @return
 */
void THUNDER_BLE::Give_Semaphore_BLE(int ble_mesg_type)
{
  xQueueSend( Queue_Semaphore_BLE, ( void * )&ble_mesg_type, ( TickType_t ) 0 );
}


void THUNDER_BLE::SetBleConnectType(enum_Ble_Status new_status)
{
  ble_connect_type = new_status;
}
enum_Ble_Status THUNDER_BLE::GetBleConnectType()
{
  return ble_connect_type;
}