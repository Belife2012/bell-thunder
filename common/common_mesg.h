#ifndef _COMMON_MESG_
#define _COMMON_MESG_

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define MAX_APPS_TASK_COUNTER   5

class Common_Mesg
{
public:
    Common_Mesg();
    ~Common_Mesg();

    void Suspend_Others_AppsTask();
    void Resume_Others_AppsTask();

    // 串口“示波器”的数据队列
    QueueHandle_t Queue_encoder_left;
    QueueHandle_t Queue_encoder_right;

    TaskHandle_t Task_Apps[MAX_APPS_TASK_COUNTER];
    // TaskHandle_t current_Task;

    UBaseType_t former_Priority;
};

extern Common_Mesg commonMesg;

#endif
