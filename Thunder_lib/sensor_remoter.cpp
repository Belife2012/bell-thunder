#include <Arduino.h>
#include <bell_thunder.h>
#include "sensor_remoter.h"

SENSOR_REMOTER::SENSOR_REMOTER(/* args */)
{
}

SENSOR_REMOTER::~SENSOR_REMOTER()
{
}

void SENSOR_REMOTER::Analyze_Raw_Data(const uint8_t* raw_data, const uint8_t length)
{
    int i=0;
    
#ifdef DEBUG_REMOTER_DATA
    for(; i<4 && i<length; i++){
      Serial.printf("%4d ", (int8_t)raw_data[i]);
    }
    for(; i<length; i++){
      Serial.printf("%02x ", raw_data[i]);
    }
    Serial.println();
#endif

    if(enable_remote)
    {
        if(length == 10){// KYE_HOME这个键发生事件时，数据长度是3，无法用于计算下面键值
            // 相等才是按键Press，只有一个按键成立
            for(i=0; i<KEY_SELECT; i++){
                if( raw_data[key_location_info[i].data_index] == key_location_info[i].data_bit_mask ){
                    if(keys_value & (0x00000001 << i)){
                        keys_pressing &= (~(0x00000001 << i)); // 清掉press动作发生时
                    }else{
                        keys_releasing &= (~(0x00000001 << i)); // 清掉release动作发生时
                        keys_pressing |= 0x00000001 << i; // 置位release动作发生时
                        keys_value |= 0x00000001 << i;
                    }
                }else{
                    if(keys_value & (0x00000001 << i)){
                        keys_releasing |= 0x00000001 << i;
                        keys_pressing &= (~(0x00000001 << i));
                        keys_value &= (~(0x00000001 << i));
                    }else{
                        keys_releasing &= (~(0x00000001 << i));
                    }
                }
            }
            for(i=KEY_SELECT; i<KEY_AMOUNT; i++){
                if( raw_data[key_location_info[i].data_index] & key_location_info[i].data_bit_mask ){
                    if(keys_value & (0x00000001 << i)){
                        keys_pressing &= (~(0x00000001 << i)); // 清掉press动作发生时
                    }else{
                        keys_releasing &= (~(0x00000001 << i)); // 清掉release动作发生时
                        keys_pressing |= 0x00000001 << i; // 置位release动作发生时
                        keys_value |= 0x00000001 << i;
                    }
                }else{
                    if(keys_value & (0x00000001 << i)){
                        keys_releasing |= 0x00000001 << i;
                        keys_pressing &= (~(0x00000001 << i));
                        keys_value &= (~(0x00000001 << i));
                    }else{
                        keys_releasing &= (~(0x00000001 << i));
                    }
                }
            }

            for(i=0; i<KEY_L2_ANALOG; i++){
                control_value[i] = (int8_t)raw_data[i];

                if(i==KEY_ROCKER_L_X || i==KEY_ROCKER_R_X){
                    if((raw_data[i] & (uint8_t)0x80) == 0){
                        control_value[i] = (control_value[i] - 127) * 100 / 127;
                    }else{
                        control_value[i] = ( control_value[i] + 128 ) * 100 / 127;
                    }
                }else{
                    if((raw_data[i] & (uint8_t)0x80) == 0){
                        control_value[i] = (127 - control_value[i]) * 100 / 127;
                    }else{
                        control_value[i] = ( (-128) - control_value[i] ) * 100 / 127;
                    }
                }
            }
            for(i=KEY_L2_ANALOG; i<KEY_ANALOG_AMOUNT; i++){
                control_value[i] = (int)raw_data[i] * 100 / 255;
            }
        }
    }else{
        Clear_All_keys();
    }
}

void SENSOR_REMOTER::Enable_Remote(bool enable)
{
    if(enable == true){
        enable_remote = true;
    } else {
        enable_remote = false;
        Clear_All_keys();
    }
}

void SENSOR_REMOTER::Clear_All_keys()
{
    memset(control_value, 0, sizeof(control_value));
    keys_value = 0;
    keys_pressing = 0;
    keys_releasing = 0;
}

/**
 * @brief 检测是否发生了按下动作（每次按下只能读到一次true）或释放动作（每次释放只能读到一次true）
 * 
 * @param key_index 按键标识
 * @param key_action 检测的动作
 * @return true 检测的动作有发生
 * @return false 检测的动作没有发生
 */
bool SENSOR_REMOTER::Check_Key_Action(enum_Remoter_Key key_index, enum_Key_Action key_action)
{
    bool ret;

    if(key_action == KEY_PRESSING){
        if( (keys_pressing & (0x00000001 << key_index)) == 0 ){
            ret = false;
        }else{
            ret = true;
            keys_pressing &= (~(0x00000001 << key_index)); // 动作只能被捕获一次
        }
    }else if(key_action == KEY_RELEASING){
        if( (keys_releasing & (0x00000001 << key_index)) == 0 ){
            ret = false;
        }else{
            ret = true;
            keys_releasing &= (~(0x00000001 << key_index)); // 动作只能被捕获一次
        }
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*----------------------------- Thunder IDE API -----------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief 打开/关闭蓝牙遥控器功能，默认是打开蓝牙遥控器功能的，但是蓝牙遥控器功能会占用
 * 系统资源，如果不需要蓝牙遥控器功能，可以使用此函数关闭
 * 
 * @param enable true打开蓝牙遥控器功能， false关闭蓝牙遥控器功能
 */
void SENSOR_REMOTER::Turnon_Remote(bool enable)
{
    if(enable == true){
        Bell_Thunder.Set_Ble_Type(BLE_TYPE_CLIENT);
    } else {
        Bell_Thunder.Set_Ble_Type(BLE_TYPE_NONE);
    }
}

/**
 * @brief 获取摇杆或按键的数值
 * 
 * @param key_index 摇杆、按键标识
 * @return int 摇杆、按键的模拟数值
 */
int SENSOR_REMOTER::Get_Control_Value(enum_Remoter_Value key_index)
{
    return control_value[key_index];
}

/**
 * @brief 判断按键状态
 * 
 * @param key_index 按键标识
 * @return true 按键状态为按下
 * @return false 按键状态为释放
 */
bool SENSOR_REMOTER::Check_Key(enum_Remoter_Key key_index)
{
    return (keys_value & (0x00000001 << key_index)) == 0 ? false : true;
}
