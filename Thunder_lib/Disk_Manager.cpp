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