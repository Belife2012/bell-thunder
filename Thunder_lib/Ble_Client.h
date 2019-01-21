#ifndef __BLE_CLIENT_H__
#define __BLE_CLIENT_H__

#include "BLEScan.h"

class BLE_CLIENT
{
private:
    /* data */
    BLEClient*  pClient = NULL;
    BLEScan* pBLEScan = NULL;

    bool connectToServer(BLEAddress pAddress);

public:
    BLE_CLIENT(/* args */);
    ~BLE_CLIENT();

    void Setup_Ble_Client();
    void Scan_Ble_Server();
    void Stop_Scan();
    void Connect_Ble_Server();
    void Disconnect_Ble_Server();
    void Write();
};

#endif
