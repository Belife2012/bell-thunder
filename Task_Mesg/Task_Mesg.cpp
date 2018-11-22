#include "Task_Mesg.h"
#include <Thunder_lib.h>

TASK_MESG Task_Mesg;

/**************************************Deamon Thread*******************************************/
void Driver_Flush(void *pvParameters)
{
  uint32_t current_time;
  uint32_t character_roll_time;
  uint32_t battery_measure_time;

  character_roll_time = millis();
  battery_measure_time = millis();
  for (;;)
  {
    current_time = millis();

    if (Task_Mesg.Get_flush_Tasks() & (0x00000001 << FLUSH_COMMUNICATIONS))
    {
      Thunder.Check_Communication();
    }
    if (Task_Mesg.Get_flush_Tasks() & (0x00000001 << FLUSH_MATRIX_LED))
    {
      Thunder.LED_Show();
    }
    if (Task_Mesg.Get_flush_Tasks() & (0x00000001 << FLUSH_COLOR_LED))
    {
      I2C_LED.LED_Flush();
      // delay(1);
    }
    if (Task_Mesg.Get_flush_Tasks() & (0x00000001 << FLUSH_CHARACTER_ROLL))
    {
      if (current_time - character_roll_time > 150)
      {
        Dot_Matrix_LED.Play_String_NextFrame();
        character_roll_time = millis();
      }
    }
    if (Task_Mesg.Get_flush_Tasks() & (0x00000001 << FLUSH_BATTERY_MEASURE))
    {
      if (current_time - battery_measure_time > 300)
      {
        Thunder.Get_Battery_Data();
        battery_measure_time = millis();
      }
    }

    // 每30ms进行一次查询
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}
void Deamon_Motor(void *pvParameters)
{
  for (;;)
  {
    if (Task_Mesg.Get_flush_Tasks() & (0x00000001 << FLUSH_MOTOR_PID_CTRL))
    {
      Thunder.En_Motor();
    }
    // 电机 PID运算周期为 50ms
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/************************************new app Thread*****************************************/
void New_Loop_Task(void *pvParameters)
{
  setup_1();
  for (;;)
  {
    loop_1();
  }
}

TASK_MESG::TASK_MESG()
{
  Queue_encoder_left = xQueueCreate(3, sizeof(float));
  Queue_encoder_right = xQueueCreate(3, sizeof(float));

  for (uint8_t i = 0; i < MAX_APPS_TASK_COUNTER; i++)
  {
    Task_Apps[i] = NULL;
  }

  // 创建 IIC互斥体
  xSemaphore_IIC = xSemaphoreCreateMutex();
  if (xSemaphore_IIC == NULL)
  {
    while (1)
    {
      Serial.println("Mutex_IIC create fail");
    }
  }

  vPortCPUInitializeMutex(&spinlockMUX);

  former_Priority = 0;
  tasks_num = 0;
  flush_Tasks = 0;
  deamon_task_running = 0;
}

TASK_MESG::~TASK_MESG()
{
}

#if 1
/* 
 * 用于硬件驱动，阻止其他线程占用CPU，恢复前不允许delay
 * 
 * @parameters:
 * @return
 */
void TASK_MESG::Suspend_Others_AppsTask()
{
  // former_Priority = uxTaskPriorityGet( NULL );

  // // highese Priority in the AppTasks
  // vTaskPrioritySet(NULL, 12);
}
/* 
 * 用于硬件驱动，恢复其他线程
 * 
 * @parameters:
 * @return
 */
void TASK_MESG::Resume_Others_AppsTask()
{
  // // No suspend any one, then no resume
  // if(former_Priority == 0){
  //     return;
  // }

  // vTaskPrioritySet(NULL, former_Priority);
}
#endif

/* 
 * 用于IIC硬件驱动，阻止其他线程占用IIC，恢复前 不建议使用delay
 * 
 * @parameters:
 * @return
 */
void TASK_MESG::Take_Semaphore_IIC()
{
  do
  {
  } while (xSemaphoreTake(xSemaphore_IIC, portMAX_DELAY) != pdPASS);
}
/* 
 * 用于硬件驱动，恢复其他线程使用IIC
 * 
 * @parameters:
 * @return
 */
void TASK_MESG::Give_Semaphore_IIC()
{
  xSemaphoreGive(xSemaphore_IIC);
}
/* 
 * 用于硬件驱动(如 语音芯片时序控制)，提升当前task 的优先级别Priority为所有应用线程的最高级别，
 * clear 前不能使用delay，不然其他低级别的线程会占用CPU，干扰到硬件操作的完整性
 * 
 * @parameters:
 * @return
 */

void TASK_MESG::Set_Current_Task_Supreme()
{
  former_Priority = uxTaskPriorityGet(NULL);

  // highese Priority in the AppTasks
  vTaskPrioritySet(NULL, APP_TASK_PRIORITY_MAX);
}
/* 
 * 用于硬件驱动，恢复当前线程的Priority
 * 
 * @parameters:
 * @return
 */
void TASK_MESG::Clear_Current_Task_Supreme()
{
  // No suspend any one, then no resume
  if (former_Priority == 0)
  {
    return;
  }

  vTaskPrioritySet(NULL, former_Priority);
}

/* 
 * 进入临界区
 * 
 * @parameters: 
 * @return: 
 */
void TASK_MESG::Enter_Task_Critical()
{
  portENTER_CRITICAL(&spinlockMUX);
}

/* 
 * 退出临界区
 * 
 * @parameters: 
 * @return: 
 */
void TASK_MESG::Exit_Task_Critical()
{
  portEXIT_CRITICAL(&spinlockMUX);
}

/*
 * 创建多一个线程，新的loop_1和 setup_1就可以使用了
 * 
 * @parameters: 
 * @return: 返回0表示创建成功
 *          返回1表示创建数目已经达到最大值，不能继续创建
 */
uint8_t TASK_MESG::Create_New_Loop()
{
  if (tasks_num >= MAX_APPS_TASK_COUNTER)
  {
    return 1;
  }

  /***create New tasks, Priority: 1~7 ***/
  xTaskCreatePinnedToCore(New_Loop_Task, "newLoopTask", 8192, NULL, 1, &Task_Apps[0], 1);
  tasks_num++;

  return 0;
}

/*
 * 创建后台线程，守护电机PID控制和检测通信数据/屏幕刷新控制：
 * 一个用于需要定时刷新的功能，要求实时性不高
 * 另外一个用于电机PID固定调节周期使用
 * 
 * @parameters: 
 * @return: 
 */
void TASK_MESG::Create_Deamon_Threads()
{
  // deamon task已经开始，不能重复创建
  if (deamon_task_running == 1)
  {
    return;
  }
  // deamon Tasks , Priority: 8~10
  xTaskCreatePinnedToCore(Driver_Flush, "DriverFlush", 8192, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(Deamon_Motor, "deamonMotor", 4096, NULL, 10, NULL, 1);
  deamon_task_running = 1;
}

/*
 * 
 * 
 * @parameters: 
 * @return: 
 */
void TASK_MESG::Remove_Deamon_Threads()
{
}

UBaseType_t TASK_MESG::Get_flush_Tasks()
{
  return flush_Tasks;
}

/*
 * 设置某个刷新线程 在后台守护线程运行
 * 
 * @parameters: 
 *      0 通信检查，Thunder.Check_Communication();
 *      1 LED点阵显示刷新，Thunder.LED_Show();
 * @return: 
 */
void TASK_MESG::Set_Flush_Task(byte flushType)
{
  if (flushType >= FLUSH_MAX_NUM)
  {
    return;
  }
  else
  {
    flush_Tasks |= (0x00000001 << flushType);
  }
}
/*
 * 移除某个刷新线程 在后台守护线程运行
 * 
 * @parameters: 
 *      0 通信检查，Thunder.Check_Communication();
 *      1 LED点阵显示刷新，Thunder.LED_Show();
 * @return: 
 */
void TASK_MESG::Remove_Flush_Task(byte flushType)
{
  if (flushType > 1)
  {
    return;
  }
  else
  {
    flush_Tasks &= ~(0x00000001 << flushType);
  }
}
