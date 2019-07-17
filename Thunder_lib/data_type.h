#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

typedef enum {
  PROGRAM_USER_1 = 0,
  PROGRAM_USER_2,
  PROGRAM_USER_3,
  PROGRAM_USER_4,
  PROGRAM_THUNDER_GO,
  PROGRAM_RUNNING
} enum_Program_Index;

typedef enum {
  PROCESS_USER_1 = 0,
  PROCESS_USER_2,
  PROCESS_USER_3,
  PROCESS_USER_4,
  PROCESS_THUNDER_GO,
  PROCESS_INDICATE_SWITCH,
  PROCESS_WAIT_SWITCH,
  PROCESS_STOP,
  PROCESS_INDICATE_STOP,
  PROCESS_READY_RUN
} enum_Process_Status;

typedef enum{
  KEY_NONE = 0,
  KEY_LONG_NO_RELEASE,
  KEY_LONG_RELEASE,
  KEY_CLICK_ONE,
  KEY_CLICK_TWO,
  KEY_CLICK_THREE,
  KEY_CLICK_FOUR
} enum_Key_Value;

typedef enum{
  BLE_TYPE_NONE = 0, // 不启用BLE
  BLE_TYPE_SERVER,   // 作为BLE Server 连接手机APP遥控
  BLE_TYPE_CLIENT    // 作为BLE Client 连接蓝牙手柄
} enum_Ble_Type;

typedef enum{
  BLE_NOT_OPEN,
  BLE_CLIENT_DISCONNECT, // 作为BLE Client, 未连接蓝牙手柄
  BLE_SERVER_CONNECTED,   // 作为BLE Server
  BLE_CLIENT_CONNECTED    // 作为BLE Client, 已经连接蓝牙手柄
} enum_Ble_Status; //  BLE 工作状态

typedef enum{
  BLE_SERVER_SEMAPHORE_RX = 1,
  BLE_CLIENT_SEMAPHORE_CONN
} enum_Ble_Mesg;

/* 串口指令的数据包相关宏 旧协议 */
#define UART_GENERAL_IR_SENSOR           (0x51)
#define UART_GENERAL_US_SENSOR           (0x52)
#define UART_GENERAL_COLOR_SENSOR        (0x53)
#define UART_GENERAL_BATTERY_SENSOR      (0x54)
#define UART_GENERAL_VERSION_INFO        (0x55)
#define UART_GENERAL_COLOR_CARD          (0x56)

#define UART_GENERAL_SEARCH_LINE         (0x66)

#define UART_GENERAL_BLE_NAME            (0xA1)
#define UART_GENERAL_SERVO_CTRL          (0xA2)
#define UART_GENERAL_PLAY_VOICE          (0xA3)

#define UART_GENERAL_MOTOR_SINGLE        (0xB1)
#define UART_GENERAL_MOTOR_DOUBLE        (0xB2)
#define UART_GENERAL_MOTOR_SINGLE_PID    (0xB3)
#define UART_GENERAL_MOTOR_DOUBLE_PID    (0xB4)
#define UART_GENERAL_GET_MOTOR_SPEED     (0xB5)

#define UART_GENERAL_SINGLE_RGBLED       (0xC1)
#define UART_GENERAL_LEFT_RGBLED         (0xC2)
#define UART_GENERAL_RIGHT_RGBLED        (0xC3)

#define UART_GENERAL_DEBUG_LED           (0xD1) // 与点阵屏的扫面相关
#define UART_GENERAL_PICTURE_LED         (0xD2)
#define UART_GENERAL_DEBUG_PRE_LED       (0xD3) // 
#define UART_GENERAL_DEBUG_SUF_LED       (0xD4) // 

#define UART_GENERAL_ANIMATION_LED       (0xE1)
/* 新协议 */
#define UART_CALL_SPECIAL_FUNCTION       (0xF1)
/* 炮台协议 */
#define UART_BARBETTE_CTRL               (0xAB)
#define UART_BARBETTE_INFO               (0xDB)



#endif
