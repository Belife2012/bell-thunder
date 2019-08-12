#ifndef __OS_FUNCTION_H__
#define __OS_FUNCTION_H__
#include <Arduino.h>

#define APP_TASK_PRIORITY_MAX   7

class TASK_SUPREME
{
private:
    /* data */
    UBaseType_t former_Priority;
public:
    TASK_SUPREME():former_Priority(0) {}

    void Set_Current_Task_Supreme();
    void Clear_Current_Task_Supreme();
};

#endif
