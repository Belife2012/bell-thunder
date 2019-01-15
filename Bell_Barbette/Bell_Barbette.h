#ifndef BELL_BARBETTE
#define BELL_BARBETTE

#include "Wire.h"

#define BARBETTE_ADDR 0x24

class Bell_Barbette
{
public:
    Bell_Barbette();
    void Begin();
    void Open_Fire();
    void Stop_Fire();
    uint8_t Fire_With_Mode(uint8_t mode);
    void Supply_On();
    void Supply_Off();
    void Supply_With_Rate(uint8_t rate);
    uint16_t Get_Bullet();
    uint32_t Get_Used_Time();
    void Set_Barbette_Pos(uint8_t pos);
    uint8_t Fire_Onece(uint8_t mode);
    void Clear_Bullet();
    
    void Auto_Fire(uint8_t mode);
    void Adjust_Pos(uint8_t value);

private:
    uint8_t compute_crc(uint8_t *buf,uint8_t num);
    uint8_t send_data(uint8_t *data);
    void receive_data(uint8_t *data,uint8_t num);
};


#endif