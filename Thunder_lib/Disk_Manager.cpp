#include <Arduino.h>
#include <EEPROM.h>
#include "Disk_Manager.h"

DISK_MANAGER Disk_Manager;

DISK_MANAGER::DISK_MANAGER(void)
{
}

bool DISK_MANAGER::Disk_Manager_Initial()
{
  if(!EEPROM.begin(DISK_SIZE_ALL))
  {
    Serial.println("Init Disk_Manager fail!"); 
    return false;
  }
  Serial.println("Init Disk_Manager OK"); 

  return true;
}

bool DISK_MANAGER::Wirte_Program_User(enum_Process_Status new_mode)
{
  EEPROM.writeByte( DISK_ADDR_PROGRAM_MODE, (uint8_t)new_mode );

  return EEPROM.commit();
}

enum_Process_Status DISK_MANAGER::Read_Program_Mode()
{
  uint8_t read_value;

  read_value = EEPROM.readByte( DISK_ADDR_PROGRAM_MODE );

  return (enum_Process_Status)read_value;
}

bool DISK_MANAGER::Wirte_Ble_Server_Mac(const uint8_t* new_mac)
{
  size_t length;

  length = EEPROM.writeBytes( DISK_ADDR_BLE_SERVER_MAC, new_mac, DISK_SIZE_BLE_SERVER_MAC );
  if(length != DISK_SIZE_BLE_SERVER_MAC){
    return false;
  }

  return EEPROM.commit();
}

bool DISK_MANAGER::Read_Ble_Server_Mac(uint8_t* mac_addr)
{
  size_t length;

  length = EEPROM.readBytes( DISK_ADDR_BLE_SERVER_MAC, mac_addr, DISK_SIZE_BLE_SERVER_MAC );
  if(length != DISK_SIZE_BLE_SERVER_MAC){
    return false;
  }

  return true;
}