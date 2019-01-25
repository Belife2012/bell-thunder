#include "data_type.h"

#define DISK_SIZE_ALL               128
#define DISK_ADDR_PROGRAM_MODE      0
#define DISK_SIZE_PROGRAM_MODE      1 // 1
#define DISK_ADDR_BLE_SERVER_MAC    1
#define DISK_SIZE_BLE_SERVER_MAC    6 // 7

class DISK_MANAGER{
public: 
    DISK_MANAGER(void);

    bool Disk_Manager_Initial(void);
    bool Wirte_Program_User(enum_Process_Status new_mode);
    enum_Process_Status Read_Program_Mode();
    bool Wirte_Ble_Server_Mac(const uint8_t* new_mac);
    bool Read_Ble_Server_Mac(uint8_t* mac_addr);

private:

};

extern DISK_MANAGER Disk_Manager;
