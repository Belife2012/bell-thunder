#include <Arduino.h>
#include "BLEDevice.h"
#include "ble_client.h"
#include <bell_thunder.h>
#include <disk_manager.h>
#include "function_macro.h"

// #define RECORD_SERVER_MAC

#define SERVER_IS_NRF_UART      0
#define SERVER_IS_THUNDER_GO    1
#define SERVER_IS_REMOTER       2
#define SERVER_STYLE            (SERVER_IS_REMOTER)

#if (SERVER_STYLE == SERVER_IS_NRF_UART)
  // The remote device we wish to connect to.
  static BLEUUID deviceUUID("00000306-0000-1000-8000-00805f9b34fb");
  // The remote service we wish to connect to.
  static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  // The characteristic of the remote service we are interested in.
  static BLEUUID wCharUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
  static BLEUUID rCharUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
#elif (SERVER_STYLE == SERVER_IS_THUNDER_GO)
  static BLEUUID deviceUUID("33e5ae8b-e58d-9ae4-bc9f-931c69b5d350");
  static BLEUUID serviceUUID("33e5ae8b-e58d-9ae4-bc9f-931c69b5d350");
  static BLEUUID wCharUUID("33e5ae8b-e58d-9ae4-bc9f-931c69b5d354");
  static BLEUUID rCharUUID("33e5ae8b-e58d-9ae4-bc9f-931c69b5d354");
#elif (SERVER_STYLE == SERVER_IS_REMOTER)
  static BLEUUID deviceUUID("00008850-0000-1000-8000-00805f9b34fb");
  static BLEUUID serviceUUID("00008850-0000-1000-8000-00805f9b34fb");
  static BLEUUID wCharUUID("0000885f-0000-1000-8000-00805f9b34fb");
  static BLEUUID rCharUUID("0000885a-0000-1000-8000-00805f9b34fb");

#endif

static uint8_t storedServerAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static BLEAdvertisedDevice* myServerDevice;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicW;
static BLEClient*  pClient;
static BLEScan* pBLEScan;
static uint8_t modeCommand[4] = {0x01, 0xaa, 0x55, 0x01};

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
static bool connectToServer();

class MyBLEClientCallbacks: public BLEClientCallbacks {
	void onConnect(BLEClient *_pClient){
    Serial.println("## Ble Client connected");
    // BLE_Client.Stop_Scan();
  }
	void onDisconnect(BLEClient *_pClient){
    Serial.println("## Ble Client Disconnected");
    connected = false;
    if(BLE_THUNDERGO::GetBleConnectType() == BLE_CLIENT_CONNECTED){
      BLE_THUNDERGO::SetBleConnectType(BLE_CLIENT_DISCONNECT);
    }
  }
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) 
  {
    // 统计扫描到设备的个数，超出一定数量就停止扫描，防止内存溢出
    // （重新开启扫描，会清空里面的std::vector<BLEAdvertisedDevice>）
    // if(scanDeviceCounts > 30) {
    //   scanDeviceCounts = 0;
    //   advertisedDevice.getScan()->stop();
    //   return;
    // }else {
    //   scanDeviceCounts++;
    // }

#ifdef DEBUG_BLE_CLIENT
    Serial.print("Founs Device : ");
    Serial.println(advertisedDevice.toString().c_str());
#endif

#if(SERVER_STYLE == SERVER_IS_REMOTER)
    // We have found a device, let us now see if it contains the right infomation
#ifdef RECORD_SERVER_MAC
    const uint8_t clearServerAddr[DISK_SIZE_BLE_SERVER_MAC] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if(memcmp(clearServerAddr, storedServerAddr, DISK_SIZE_BLE_SERVER_MAC) == 0){
      if (advertisedDevice.haveName() && advertisedDevice.getName() == std::string("bell_Controller")
          && advertisedDevice.haveRSSI() && (advertisedDevice.getRSSI() > -70) ) {
      // 
        uint8_t *advertisedPayload;
        advertisedPayload = advertisedDevice.getPayload();
        Serial.printf("AdvertiseData: %2x\n", advertisedPayload[advertisedPayload[0]+1+1]);
        advertisedDevice.getScan()->stop();

        if(pServerAddress != NULL) { delete pServerAddress;}
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        Serial.print("New device address: "); 
        Serial.println(pServerAddress->toString().c_str());

        memcpy(storedServerAddr, *(pServerAddress->getNative()), DISK_SIZE_BLE_SERVER_MAC);
        if(Disk_Manager.Write_Ble_Server_Mac(storedServerAddr) == false){
          Serial.println("Write_Ble_Server_Mac Fail");
        }

        BLE_THUNDERGO::Give_Semaphore_BLE(BLE_CLIENT_SEMAPHORE_CONN);
      } // Found our server
    }else{
      if(pServerAddress != NULL) { delete pServerAddress;}
      pServerAddress = new BLEAddress(storedServerAddr);
      if ( advertisedDevice.getAddress().equals(*pServerAddress) ) {
      // 
        advertisedDevice.getScan()->stop();

        Serial.print("Old device address: "); 
        Serial.println(pServerAddress->toString().c_str());

        BLE_THUNDERGO::Give_Semaphore_BLE(BLE_CLIENT_SEMAPHORE_CONN);
      } // Found our server
    }
#else
      if (advertisedDevice.haveName() && advertisedDevice.getName() == std::string("bell_Controller") )
      {
        uint8_t *advertisedPayload;
        advertisedPayload = advertisedDevice.getPayload();
        // 遥控器存有连接上的雷霆BLE address，在广播期间广播出来，
        // 所以雷霆可以通过广播信息判断手柄是否已经被绑定,否则需要去判断距离是否合适新建连接
        if(advertisedPayload[advertisedPayload[0]+1+1] == 0x17){
          for(int i=3; i < 3+6; i++){
            // 手柄已经被绑定，但是广播中的绑定地址 与 本设备address不相同，则不进行连接
            if(storedServerAddr[8-i] != advertisedPayload[advertisedPayload[0]+i]){
              return;
            }
          }
          Serial.println("Bonding Match");
        }else{
          Serial.println("No Bonding");
          if( advertisedDevice.haveRSSI() && (advertisedDevice.getRSSI() > -60) ){
            Serial.println("Distance Appropriate");
          }else{
            Serial.println("Distance Too far");
            return;
          }
        }

        BLE_THUNDERGO::SetBleConnectType(BLE_CLIENT_CONNECTED);
        advertisedDevice.getScan()->stop();
        Serial.println("Scanned myServerDevice");

        if(myServerDevice != NULL) {
          delete myServerDevice;
        }
        myServerDevice = new BLEAdvertisedDevice(advertisedDevice);

        BLE_THUNDERGO::Give_Semaphore_BLE(BLE_CLIENT_SEMAPHORE_CONN);
      } // Found our server
#endif
#else
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(deviceUUID))
      { } // onResult
#endif
  }

private:
  int scanDeviceCounts = 0;
}; // MyAdvertisedDeviceCallbacks

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,uint8_t* pData,size_t length,bool isNotify) {
#if (SERVER_STYLE == SERVER_IS_REMOTER)
  BLE_Remoter.Analyze_Raw_Data(pData, length);
#else
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
#endif
}

bool connectToServer()
{
#ifdef DEBUG_BLE_CLIENT
  Serial.print("Forming a connection to ");
  Serial.println(myServerDevice->getAddress().toString().c_str());
#endif

  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyBLEClientCallbacks());

  if(false == pClient->connect(myServerDevice->getAddress())){
    Serial.println(" Fail to connecte server");
    return false;
  }
  Serial.println(" - Connected to server");
  delay(1000);

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(rCharUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(rCharUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");
  
  // Read the value of the characteristic.
  if(pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if(pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  
  return true;
}

BLE_CLIENT::BLE_CLIENT(/* args */)
{
    // do{
    //   pClient = BLEDevice::createClient();
    // }while(pClient == NULL);
    // pClient->setClientCallbacks(new MyBLEClientCallbacks());
}

BLE_CLIENT::~BLE_CLIENT()
{
  if(pClient != NULL){
    pClient->disconnect(); //also free pClient
  }
  
}

void BLE_CLIENT::Setup_Ble_Client()
{
  #ifdef RECORD_SERVER_MAC
  if(Disk_Manager.Read_Ble_Server_Mac(storedServerAddr) == false){
    Serial.println("Read_Ble_Server_Mac Fail");
  }
  #else
  memcpy(storedServerAddr, BLEDevice::getAddress().getNative(), sizeof(storedServerAddr));
  #endif

  // BLEDevice::init("");
  Serial.printf("BLE Client Addr: %s\n", BLEDevice::getAddress().toString().c_str());

  if(pBLEScan == NULL){
    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device. 
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);

    // pBLEScan->start(30); // 会阻塞，所以不能放在主线里面
  }
} // End of setup.

// pBLEScan->start 会产生阻塞，所以只能在辅线程中调用
void BLE_CLIENT::Scan_Ble_Server()
{
  if(pBLEScan == NULL)
    return;

  const uint32_t scan_time = 10;
  // start the scan to run for seconds.
  #ifdef DEBUG_BLE_CLIENT
  Serial.printf("Start BLE Scan time: %ds\n", scan_time);
  #endif
  pBLEScan->start(scan_time);
}

void BLE_CLIENT::Stop_Scan()
{
  if(pBLEScan == NULL)
    return;

  pBLEScan->stop();
}

void BLE_CLIENT::Connect_Ble_Server()
{
  // If we have scanned for and found the desired BLE Server with which we wish to connect. 
  // Now we connect to it. Once we are connected we set the connected flag to be true.
  delay(100);
  if (connectToServer()) {
    Serial.println("connection OK");
    connected = true;
  } else {
    Serial.println("connection Fail");
  }
}

void BLE_CLIENT::Disconnect_Ble_Server()
{
  if(pClient == NULL)
    return;

  pClient->disconnect();
}

// This is the Arduino main loop function.
void BLE_CLIENT::Write() {

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
#if (SERVER_STYLE == SERVER_IS_NRF_UART)
    uint8_t sendValue[] = {'S', 'e', 'n', 'd', 'C', '\n'};
    Serial.println("Send Command...");
    pRemoteCharacteristic->writeValue(sendValue, sizeof(sendValue));

    // String newValue = "T: " + String(millis()/10) + "\n";
    // Serial.print("Setting new characteristic value :" + newValue);
    // pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
#elif (SERVER_STYLE == SERVER_IS_THUNDER_GO)
    uint8_t sendValue[] = {0xf1, 0x01, 0x64, 0x01, 0x00, 0x17, 0x01, 0x64, 0x01, 0x00, 0x17, 0x01, 0x64, 0x01, 0x00, 0x17, 0x00};
    for(int i=0; i<sizeof(sendValue)-1; i++){
      sendValue[sizeof(sendValue)-1] += sendValue[i];
    }
    Serial.println("Send Command...");
    pRemoteCharacteristic->writeValue(sendValue, sizeof(sendValue));
#elif (SERVER_STYLE == SERVER_IS_REMOTER)
    // Serial.println("Connecting...");
    delay(500);

#endif
  }
  
  delay(10); // Delay a second between loops.
} // End of loop
