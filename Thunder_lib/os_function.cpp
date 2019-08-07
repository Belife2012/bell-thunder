#include "os_function.h"

/* 
 * 用于硬件驱动(如 语音芯片时序控制)，提升当前task 的优先级别Priority为所有应用线程的最高级别，
 * clear 前不能使用delay，不然其他低级别的线程会占用CPU，干扰到硬件操作的完整性
 * 配合Clear_Current_Task_Supreme使用，恢复原来的Priority
 * 
 * @parameters:
 * @return
 */
void TASK_SUPREME::Set_Current_Task_Supreme()
{
  if(former_Priority != 0) {
      return ;
  }

  // highese Priority in the AppTasks
  vTaskPrioritySet(NULL, APP_TASK_PRIORITY_MAX);
  
  former_Priority = uxTaskPriorityGet(NULL);
}
/* 
 * 用于硬件驱动，恢复当前线程的Priority
 * 
 * @parameters:
 * @return
 */
void TASK_SUPREME::Clear_Current_Task_Supreme()
{
  // No suspend any one, then no resume
  if (former_Priority == 0)
  {
    return;
  }

  vTaskPrioritySet(NULL, former_Priority);
  former_Priority = 0;
}

