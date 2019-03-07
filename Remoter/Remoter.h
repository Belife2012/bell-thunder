#ifndef __REMOTER_H__
#define __REMOTER_H__
#include<Arduino.h>

#define PACKAGE_LENGTH      (10)

typedef struct{
    uint8_t data_index;
    uint8_t data_bit_mask;
} struct_Key_Location;

typedef enum{
    KEY_UP = 0,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    // KEY_UP_RIGHT,
    // KEY_DOWN_RIGHT,
    // KEY_DOWN_LEFT,
    // KEY_UP_LEFT,

    KEY_SELECT, // I
    KEY_BACK, // III
    // KYE_HOME, // 这个键的数据比较特殊
    KEY_A, // A
    KEY_B, // B
    KEY_X, // C
    KEY_Y, // D
    KEY_L1,
    KEY_L2,
    KEY_R1,
    KEY_R2,
    KEY_ROCKER_L,
    KEY_ROCKER_R,
    KEY_AMOUNT
} enum_Remoter_Key;

typedef enum{
    KEY_ROCKER_L_X = 0,
    KEY_ROCKER_L_Y,
    KEY_ROCKER_R_X,
    KEY_ROCKER_R_Y,
    KEY_L2_ANALOG,
    KEY_R2_ANALOG,
    KEY_ANALOG_AMOUNT
} enum_Remoter_Value;

typedef enum{
    KEY_PRESSING = 0,
    KEY_RELEASING
} enum_Key_Action;

const struct_Key_Location key_location_info[KEY_AMOUNT] = {
    /* 需要比较等于才成立, 只有一个成立 */
    {8, 0x01},
    {8, 0x05},
    {8, 0x07},
    {8, 0x03},
    // {8, 0x02}, 
    // {8, 0x04},
    // {8，0x06},
    // {8, 0x08},

    /* 使用位与计算，可以多个同时成立 */
    {7, 0x04},
    {7, 0x08},
    {6, 0x01},
    {6, 0x02},
    {6, 0x08},
    {6, 0x10},
    {6, 0x40},
    {7, 0x01},
    {6, 0x80},
    {7, 0x02},
    {7, 0x20},
    {7, 0x40},
};

class REMOTER
{
private:
    /* data */
    uint32_t keys_value = 0x00000000; // 每一 bit 代表一个按键key的状态
    uint32_t keys_pressing = 0x00000000; // press动作发生时, 会置位
    uint32_t keys_releasing = 0x00000000; // release动作发生时, 会置位
    int control_value[KEY_ANALOG_AMOUNT];

    bool enable_remote = true;

public:
    REMOTER(/* args */);
    ~REMOTER();

    void Analyze_Raw_Data(const uint8_t* raw_data, const uint8_t length);
    void Clear_All_keys();
    int Get_Control_Value(enum_Remoter_Value key_index);
    bool Get_Key_Value(enum_Remoter_Key key_index);
    bool Get_Key_Action(enum_Remoter_Key key_index, enum_Key_Action key_action);
    void Disable_Remote(void);
    void Enable_Remote(void);
};


#endif
