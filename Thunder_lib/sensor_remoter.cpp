#include <Arduino.h>
#include <bell_thunder.h>

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

void SENSOR_REMOTER::Enable_Remote()
{
    enable_remote = true;
}
void SENSOR_REMOTER::Disable_Remote()
{
    enable_remote = false;
    Clear_All_keys();
}

void SENSOR_REMOTER::Clear_All_keys()
{
    memset(control_value, 0, sizeof(control_value));
    keys_value = 0;
    keys_pressing = 0;
    keys_releasing = 0;
}

int SENSOR_REMOTER::Get_Control_Value(enum_Remoter_Value key_index)
{
    return control_value[key_index];
}

bool SENSOR_REMOTER::Get_Key_Value(enum_Remoter_Key key_index)
{
    return (keys_value & (0x00000001 << key_index)) == 0 ? false : true;
}

bool SENSOR_REMOTER::Get_Key_Action(enum_Remoter_Key key_index, enum_Key_Action key_action)
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
