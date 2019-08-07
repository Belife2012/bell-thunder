#ifndef _TASK_MESG_
#define _TASK_MESG_

#include <Arduino.h>
#include <function_macro.h>
#include "data_type.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// #define COMPATIBILITY_OLD_ESP_LIB

#define MAX_APPS_TASK_COUNTER 32
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

/* 按键与指示灯 */
extern uint32_t led_indication_counter;

#ifdef COMPETITION_FW_001

extern uint8_t competition_action_status;
void Program_AutoCtrl(void);
uint8_t Create_New_Loop_AutoCtrl(uint8_t program_sequence, 
                  func_Program_Setup program_setup, func_Program_Loop program_loop );
void Clear_All_Loops_AutoCtrl(void);

#endif
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
  void Set_Flush_Task(byte flushType);
  void Remove_Flush_Task(byte flushType);

  UBaseType_t Get_flush_Tasks();
  
  void Enter_Task_Critical();
  void Exit_Task_Critical();
  
  void Toggle_Competition_Status(int status_index);

private:
  TaskHandle_t Task_Apps[MAX_APPS_TASK_COUNTER];
  struct_Apps_Param *task_param[MAX_APPS_TASK_COUNTER];
  portMUX_TYPE spinlockMUX;

  UBaseType_t flush_Tasks;

  uint8_t tasks_num;
  uint8_t deamon_task_running;
};

extern TASK_MESG Task_Mesg;

#endif
