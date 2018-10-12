#ifndef _TASK_MESG_
#define _TASK_MESG_

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define MAX_APPS_TASK_COUNTER   1

void setup_1(void);
void loop_1(void);

class TASK_MESG
{
public:
    TASK_MESG();
    ~TASK_MESG();

    uint8_t Create_New_Loop();
    void Create_Deamon_Threads();

    void Suspend_Others_AppsTask();
    void Resume_Others_AppsTask();

    // 串口“示波器”的数据队列
    QueueHandle_t Queue_encoder_left;
    QueueHandle_t Queue_encoder_right;

private:

    TaskHandle_t Task_Apps[MAX_APPS_TASK_COUNTER];
    UBaseType_t former_Priority;

    uint8_t tasks_num;
};

extern TASK_MESG Task_Mesg;

#endif
