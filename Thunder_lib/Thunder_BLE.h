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
#include <Thunder_Motor.h>
#include <Thunder_Display.h>

// 单色点阵
#include <HT16D35B.h>

// 内存读取
#define EEPROM_SIZE 64
#define BLE_NAME_SIZE 32

// 蓝牙名称首地址
#define ADD_BLE_NAME        0x00   //蓝牙名称存储地址，不能改变，否则整个BLE name 读写的算法要更改

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

class THUNDER_BLE
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

    void Setup_Ble_Security();

  public:
    // 内存读取
    uint64_t Get_ID(void);              // 获取并返回芯片ID
    void Setup_EEPROM(void);            // 配置EEPROM
    void Read_BLE_Name (uint32_t addr);      // 查看是否有自定义蓝牙名称，如没自定义则读取芯片ID
    void Write_BLE_Name (uint32_t addr);     // 写入自定义蓝牙名称
    uint32_t Write_ROM (uint32_t addr, void *src_value, uint8_t size); // 写ROM
    uint32_t Read_ROM (uint32_t addr, void *dest_value, uint8_t size);
    uint32_t Reset_ROM (void);              // 重置ROM

    // 蓝牙
    void Setup_BLE(void);                           // 配置BLE
    void New_Ble_Server_Service();
    void Delete_Ble_Server_Service();
    void Tx_BLE(uint8_t Tx_Data[], int byte_num);   // 发送蓝牙数据
};

#endif
