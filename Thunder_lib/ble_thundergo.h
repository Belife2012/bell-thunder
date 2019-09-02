#ifndef _THUNDER_BLE_H_
#define _THUNDER_BLE_H_

#include <Arduino.h>

// 内存读取
#include "EEPROM.h"

// 蓝牙
#include <BLEDevice.h>
#include <BLE2902.h>
#include <BLEBeacon.h>

#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_main.h>

// 雷霆库
#include <motor_thunder.h>
#include <display_thunder.h>

// 内存读取
#define EEPROM_SIZE 64
#define BLE_NAME_SIZE 32

// 蓝牙
#define SERVICE_UUID            "33E5AE8B-E58D-9AE4-BC9F-931C69B5D354"
#define CHARACTERISTIC_UUID_RX  "33E5AE8B-E58D-9AE4-BC9F-931C69B5D354"
#define CHARACTERISTIC_UUID_TX  "33E5AE8B-E58D-9AE4-BC9F-931C69B5D354"
#define ADVERTISING_UUID        "8D9A"

// 蓝牙
extern bool deviceConnected;
extern bool ble_command_busy;
extern uint8_t Tx_Data[16];
extern uint8_t Rx_Data[21];
extern uint8_t BLE_Name_Data[BLE_NAME_SIZE];

typedef enum{
  BLE_NOT_OPEN,
  BLE_CLIENT_DISCONNECT, // 作为BLE Client, 未连接蓝牙手柄
  BLE_SERVER_CONNECTED,   // 作为BLE Server
  BLE_CLIENT_CONNECTED    // 作为BLE Client, 已经连接蓝牙手柄
} enum_Ble_Status; //  BLE 工作状态

typedef enum{
  BLE_TYPE_NONE = 0, // 不启用BLE
  BLE_TYPE_SERVER,   // 作为BLE Server 连接手机APP遥控
  BLE_TYPE_CLIENT    // 作为BLE Client 连接蓝牙手柄
} enum_Ble_Type; // BLE 角色

typedef enum{
  BLE_SERVER_SEMAPHORE_RX = 1,
  BLE_CLIENT_SEMAPHORE_CONN
} enum_Ble_Mesg;

class BLE_THUNDERGO
{
  private:
    // 蓝牙命名
    int BLE_Named = 0;
    uint64_t SEP32_ID;  // 16进制12位，ESP32_ID

    // 蓝牙
    BLECharacteristic *pCharacteristic;
    std::string User_BLE_Name = "Thunder_";

    BLEAdvertising *pAdvertising = NULL;
    BLEService *pService = NULL;
    BLEServer *pServer = NULL;

    static QueueHandle_t Queue_Semaphore_BLE;
    static int ble_connect_type;

    void Setup_Ble_Security();

  public:
    // 内存读取
    uint64_t Get_ID(void);              // 获取并返回芯片ID
    void Get_BLE_Name ();      // 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID
    void Set_BLE_Name ();     // 写入自定义蓝牙名称

    // 蓝牙
    void Setup_BLE(void);                           // 配置BLE
    void Start_Advertisement();
    void Delete_Ble_Server_Service();
    void Tx_BLE(uint8_t Tx_Data[], int byte_num);   // 发送蓝牙数据

    static void CreateQueueBLE();
    static int Take_Semaphore_BLE();
    static void Give_Semaphore_BLE(int ble_mesg_type);

    static void SetBleConnectType(int new_status);
    static int GetBleConnectType(); 
};

#endif
