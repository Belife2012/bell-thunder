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

#include "esp_spi_flash.h"
#include "esp_task_wdt.h"
#include <Thunder_BLE.h>
#include <Thunder_lib.h>
#include <Task_Mesg.h>
#include "print_macro.h"

uint8_t BLE_Name_Data[BLE_NAME_SIZE] = {0x00};

// 蓝牙连接/断开的回调函数
class MyServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
      deviceConnected = true;
      Serial.printf("* BLE Connected: %d *\n", pServer->getConnectedCount());
      // Thunder_Motor.Setup_PID_Timer();
    };

    void onDisconnect(BLEServer* pServer) 
    {
      deviceConnected = false;
      Serial.printf("# BLE DisConnect: %d #\n", pServer->getConnectedCount());
      Thunder.Stop_All();
      Speaker.Play_Song(45);   // 断开声音
    }
};

void Analyze_BLE_Data(std::string &recv_data)
{
  #ifdef DEBUG_BLE_COMMAND
  Serial.printf("\n*>%x*\n",recv_data[0]);
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
        Thunder.I2C_LED_BUFF1[i-1] = recv_data[i];
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
        Thunder.I2C_LED_BUFF2[i-1] = recv_data[i];
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
          Task_Mesg.Give_Semaphore_BLE();
        }
      }
    }
};

// 配置BLE
void THUNDER_BLE::Setup_BLE()
{
  Serial.printf("\nstart Init BLE...\n");

  Read_BLE_Name(ADD_BLE_NAME);  // 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID

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

      // 如果出现控制字符，则意味着EEPROM里面的信息不准确，BLE命名为“BELL”
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
    Serial.printf("* Dev BLE No name\n");

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
    Serial.printf( " 0x%02x", (int)( (oAdvertisementData.getPayload().c_str())[i] ) );
  }
  Serial.println();

  pAdvertising->setAdvertisementData(oAdvertisementData);
  // pAdvertising->start();


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

  pServer->getAdvertising()->start(); // 开始广播

  Serial.printf("BLE Init end\n");
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
    Serial.printf("# BLE DisConnected...\n");
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

// 配置EEPROM
uint32_t flash_size;
void THUNDER_BLE::Setup_EEPROM(void)
{
  Serial.println("Init EEPROM...");
#if 0
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("Init EEPROM fail!"); 
    delay(100000);
  }
  
  Serial.print("* EEPROM_SIZE : ");
  Serial.println(EEPROM_SIZE);
  
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    Serial.print(" ");
    Serial.print(byte(EEPROM.read(i)));
  }
#else
  flash_size = spi_flash_get_chip_size();
  Serial.printf("Flash size: 0x%x\n", flash_size);

#endif
  Serial.println("\nInit EEPROM end");
}

// 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID
void THUNDER_BLE::Read_BLE_Name (uint32_t addr)
{
  Serial.println("\nEEPROM:");
  Read_ROM(ADD_BLE_NAME, BLE_Name_Data, BLE_NAME_SIZE);
  for (int i = 0; i < BLE_NAME_SIZE; i++)
  {
    Serial.printf("0x%2x ", BLE_Name_Data[i]);
  }
  Serial.println();

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
void THUNDER_BLE::Write_BLE_Name (uint32_t addr)
{
  if( ESP_OK != Reset_ROM() ){  // 清空定义出来的ROM
    Serial.println("\nFlash erase fail.");
    return;
  }
  Serial.print("\nBLE w name: "); Serial.printf("%s\n", BLE_Name_Data);

  if( ESP_OK != Write_ROM(ADD_BLE_NAME, BLE_Name_Data, BLE_NAME_SIZE) ){
    Serial.println("\nFlash W fail.");
    return;
  }

  Serial.println("BLE rename Success!");
  // 断开蓝牙连接，使重命名有效
  Serial.print("Device reset...");
  delay(50);
  // for(uint32_t i = 0; i < 6; i++){
  //   Serial.print(".");
  //   delay(300);
  // }
  // 复位系统，重启设备
  *((UBaseType_t *)RTC_CNTL_OPTIONS0_REG) |= RTC_CNTL_SW_SYS_RST;
}

// 写ROM
uint32_t THUNDER_BLE::Write_ROM (uint32_t addr, void *src_value, uint8_t size)
{
#if 0
  // Serial.println("Start Write_ROM...");
  
  EEPROM.write(addr, val);
  // EEPROM.commit();

  // Serial.println("Finish Write_ROM");
#else
  if(addr + size > 0x1000){
    return ESP_FAIL;
  }

  uint32_t backCode;
  
  Thunder_Motor.Disable_PID_Timer();
  backCode = spi_flash_write(flash_size - 0x1000 + addr, src_value, size);
  Thunder_Motor.Enable_PID_Timer();

  return backCode;
#endif
}

// 写ROM
uint32_t THUNDER_BLE::Read_ROM (uint32_t addr, void *dest_value, uint8_t size)
{
  if(addr + size > 0x1000){
    return ESP_FAIL;
  }

  uint32_t backCode;

  Thunder_Motor.Disable_PID_Timer();
  backCode = spi_flash_read(flash_size - 0x1000 + addr, dest_value, size);
  Thunder_Motor.Enable_PID_Timer();
  
  return backCode;
}

// 重置ROM
uint32_t THUNDER_BLE::Reset_ROM ()
{
#if 0
  for(int i = 0; i < EEPROM_SIZE; i++)
  {
    EEPROM.write(i, 0xFF);
  }
  // EEPROM.commit();
#else

  uint32_t backCode;

  Thunder_Motor.Disable_PID_Timer();
  backCode = spi_flash_erase_range(flash_size - 0x1000, 0x1000);
  Thunder_Motor.Enable_PID_Timer();
  
  return backCode;
#endif
}
