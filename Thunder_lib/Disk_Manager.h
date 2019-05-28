#include "data_type.h"

#define DISK_SIZE_ALL                   128
#define DISK_ADDR_PROGRAM_MODE          0
#define DISK_SIZE_PROGRAM_MODE          1  // 1
#define DISK_ADDR_BLE_SERVER_MAC        1
#define DISK_SIZE_BLE_SERVER_MAC        6  // 7
#define ATTITUDE_ADDR_CALIBRATE_FALG    7 
#define ATTITUDE_SIZE_CALIBRATE_FALG    1  // 8
#define ATTITUDE_ADDR_CALIBRATE_DATA    8 
#define ATTITUDE_SIZE_CALIBRATE_DATA    48 // 56

#define ATTITUDE_FLAG_HAVE_CALIBRATE    (0x01)
#define ATTITUDE_FLAG_NO_CALIBRATE      (0xff)

class DISK_MANAGER{
public: 
    DISK_MANAGER(void);

    bool Disk_Manager_Initial(void);
    bool Wirte_Program_User(enum_Process_Status new_mode);
    enum_Process_Status Read_Program_Mode();
    bool Wirte_Ble_Server_Mac(const unsigned char* new_mac);
    bool Read_Ble_Server_Mac(unsigned char* const mac_addr);

    // 所有calibrate 数据要通过连续数组传地址过来
    bool Write_Attitude_Calibrate(const float *calibrate_data);
    bool Read_Attitude_Calibrate(float * const calibrate_data);

private:

};

extern DISK_MANAGER Disk_Manager;
