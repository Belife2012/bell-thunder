#ifndef _TASK_MESG_
#define _TASK_MESG_

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// #define COMPATIBILITY_OLD_ESP_LIB

#define MAX_APPS_TASK_COUNTER 5
#define APP_TASK_PRIORITY_MAX 5

#define POLLING_CHECK_PERIOD    (10)
#define MOTOR_CONTROL_PERIOD    (10)

typedef enum
{
  FLUSH_COMMUNICATIONS = 0,
  FLUSH_MATRIX_LED,
  FLUSH_COLOR_LED,
  FLUSH_MOTOR_PID_CTRL,
  FLUSH_CHARACTER_ROLL,
  FLUSH_BATTERY_MEASURE,
  FLUSH_MAX_NUM
} enum_Flush_Type;

typedef void(*func_Program_Setup)(void);
typedef void(*func_Program_Loop)(void);

typedef struct{
  uint8_t sequence;
  uint8_t index;
  func_Program_Setup Mysetup;
  func_Program_Loop Myloop;
} struct_Apps_Param;

extern uint32_t led_indication_counter;

void Programs_System(void);
void Program_1(void);
void Program_2(void);
void Program_3(void);
void Program_4(void);
void Program_ThunderGo(void);

class TASK_MESG
{
public:
  TASK_MESG();
  ~TASK_MESG();

  uint8_t Create_New_Loop(uint8_t program_sequence, 
                          func_Program_Setup program_setup, 
                          func_Program_Loop program_loop );
  void Clear_All_Loops();
  void Create_Deamon_Threads();
  void Remove_Deamon_Threads();
  void Set_Flush_Task(byte flushType);
  void Remove_Flush_Task(byte flushType);

  UBaseType_t Get_flush_Tasks();

  void Suspend_Others_AppsTask();
  void Resume_Others_AppsTask();
  void Take_Semaphore_IIC();
  void Give_Semaphore_IIC();
  void Take_Semaphore_BLE();
  void Give_Semaphore_BLE();
  void Set_Current_Task_Supreme();
  void Clear_Current_Task_Supreme();
  void Enter_Task_Critical();
  void Exit_Task_Critical();

  // 串口“示波器”的数据队列
  QueueHandle_t Queue_encoder_left;
  QueueHandle_t Queue_encoder_right;

private:
  TaskHandle_t Task_Apps[MAX_APPS_TASK_COUNTER];
  struct_Apps_Param *task_param[MAX_APPS_TASK_COUNTER];
  UBaseType_t former_Priority;
  portMUX_TYPE spinlockMUX;

  UBaseType_t flush_Tasks;

  uint8_t tasks_num;
  uint8_t deamon_task_running;
  volatile SemaphoreHandle_t xSemaphore_IIC;
  volatile SemaphoreHandle_t xSemaphore_BLE;
};

extern TASK_MESG Task_Mesg;

#endif
