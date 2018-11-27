#ifndef _TASK_MESG_
#define _TASK_MESG_

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// #define COMPATIBILITY_OLD_ESP_LIB

#define MAX_APPS_TASK_COUNTER 1
#define APP_TASK_PRIORITY_MAX 5

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

void setup_1(void);
void loop_1(void);

class TASK_MESG
{
public:
  TASK_MESG();
  ~TASK_MESG();

  uint8_t Create_New_Loop();
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
