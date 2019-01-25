#include <Arduino.h>
#include "BLEDevice.h"
#include "Ble_Client.h"
#include <Thunder_lib.h>
#include <Disk_Manager.h>

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
  static BLEUUID wCharUUID("0000885a-0000-1000-8000-00805f9b34fb");
  static BLEUUID rCharUUID("0000885a-0000-1000-8000-00805f9b34fb");

#endif

static uint8_t storedServerAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static BLEAddress *pServerAddress;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
#if (SERVER_STYLE == SERVER_IS_REMOTER)
  Ble_Remoter.Analyze_Raw_Data(pData, length);
#else
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
#endif
}

class MyBLEClientCallbacks: public BLEClientCallbacks {
	void onConnect(BLEClient *pClient){
    Serial.println("## Ble Client connected");
    Ble_Client.Stop_Scan();
  }
	void onDisconnect(BLEClient *pClient){
    Serial.println("## Ble Client Disconnected");
    Ble_Client.Disconnect_Ble_Server();
    if(Task_Mesg.ble_connect_type == 2){
      Task_Mesg.ble_connect_type = 0;
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
  void onResult(BLEAdvertisedDevice advertisedDevice) {
#ifdef DEBUG_BLE_CLIENT
    Serial.print("Founs Device: ");
    Serial.println(advertisedDevice.toString().c_str());
#endif

    const uint8_t clearServerAddr[DISK_SIZE_BLE_SERVER_MAC] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#if(SERVER_STYLE == SERVER_IS_REMOTER)
    // We have found a device, let us now see if it contains the right infomation
    if(memcmp(clearServerAddr, storedServerAddr, DISK_SIZE_BLE_SERVER_MAC) == 0){
      if (advertisedDevice.haveName() && advertisedDevice.getName() == std::string("bell_Controller")
          && advertisedDevice.haveRSSI() && (advertisedDevice.getRSSI() > -60) ) {
      // 
        advertisedDevice.getScan()->stop();

        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        Serial.print("New device address: "); 
        Serial.println(pServerAddress->toString().c_str());

        memcpy(storedServerAddr, *(pServerAddress->getNative()), DISK_SIZE_BLE_SERVER_MAC);
        if(Disk_Manager.Wirte_Ble_Server_Mac(storedServerAddr) == false){
          Serial.println("Wirte_Ble_Server_Mac Fail");
        }

        Task_Mesg.Give_Semaphore_BLE(2);
      } // Found our server
    }else{
      pServerAddress = new BLEAddress(storedServerAddr);
      if ( advertisedDevice.getAddress().equals(*pServerAddress) ) {
      // 
        advertisedDevice.getScan()->stop();

        Serial.print("Old device address: "); 
        Serial.println(pServerAddress->toString().c_str());

        Task_Mesg.Give_Semaphore_BLE(2);
      } // Found our server
    }
#else
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(deviceUUID)) {
#endif
  } // onResult
}; // MyAdvertisedDeviceCallbacks

BLE_CLIENT::BLE_CLIENT(/* args */)
{
    do{
      pClient = BLEDevice::createClient();
    }while(pClient == NULL);
    pClient->setClientCallbacks(new MyBLEClientCallbacks());

}

BLE_CLIENT::~BLE_CLIENT()
{
  if(pClient != NULL){
    delete pClient; 
  }
  
  if(pBLEScan != NULL){
    delete pBLEScan;
  }
}

void BLE_CLIENT::Setup_Ble_Client()
{
  if(Disk_Manager.Read_Ble_Server_Mac(storedServerAddr) == false){
    Serial.println("Read_Ble_Server_Mac Fail");
  }

  Serial.println("Starting BLE Client application...");
  // BLEDevice::init("");

  if(pBLEScan == NULL){
    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device. 
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
  }
} // End of setup.

void BLE_CLIENT::Scan_Ble_Server()
{
  if(pBLEScan == NULL)
    return;

  // start the scan to run for 30 seconds.
  pBLEScan->start(30);
}
void BLE_CLIENT::Stop_Scan()
{
  if(pBLEScan == NULL)
    return;

  pBLEScan->stop();
}

bool BLE_CLIENT::connectToServer(BLEAddress pAddress)
{
#ifdef DEBUG_BLE_CLIENT
  Serial.print("Forming a connection to ");
  Serial.println(pAddress.toString().c_str());

  Serial.println("Connecting...");
#endif

  if(false == pClient->connect(pAddress)){
    Serial.println(" Fail to connecte server");
    return false;
  }
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    return false;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(wCharUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(wCharUUID.toString().c_str());
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  std::string value = pRemoteCharacteristic->readValue();
  Serial.print("The characteristic value was: ");
  Serial.println(value.c_str());

  pRemoteCharacteristic->registerForNotify(notifyCallback);
}
void BLE_CLIENT::Connect_Ble_Server()
{
  // If we have scanned for and found the desired BLE Server with which we wish to connect. 
  // Now we connect to it. Once we are connected we set the connected flag to be true.
  if (connectToServer(*pServerAddress)) {
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