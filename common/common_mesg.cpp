#include "common_mesg.h"

Common_Mesg commonMesg;

Common_Mesg::Common_Mesg()
{
    Queue_encoder_left = xQueueCreate( 3, sizeof(float) );
    Queue_encoder_right = xQueueCreate( 3, sizeof(float) );

    for(uint8_t i = 0; i < MAX_APPS_TASK_COUNTER; i++){
        Task_Apps[i] = NULL;
    }

    former_Priority = 0;
}

Common_Mesg::~Common_Mesg()
{
    
}

void Common_Mesg::Suspend_Others_AppsTask()
{
    former_Priority = uxTaskPriorityGet( NULL );

    // highese Priority in the AppTasks
    vTaskPrioritySet(NULL, 12);
}
void Common_Mesg::Resume_Others_AppsTask()
{
    if(former_Priority == 0){
        return;
    }
    
    vTaskPrioritySet(NULL, former_Priority);
}

// void Common_Mesg::Suspend_Others_AppsTask()
// {
//     current_Task = xTaskGetCurrentTaskHandle();

//     for(uint8_t i = 0; i < MAX_APPS_TASK_COUNTER; i++){
//         if(Task_Apps[i] != current_Task && Task_Apps[i] != NULL){
//             vTaskSuspend(Task_Apps[i]);
//         }
//     }
// }
// void Common_Mesg::Resume_Others_AppsTask()
// {
//     current_Task = xTaskGetCurrentTaskHandle();

//     for(uint8_t i = 0; i < MAX_APPS_TASK_COUNTER; i++){
//         if(Task_Apps[i] != current_Task && Task_Apps[i] != NULL){
//             vTaskResume(Task_Apps[i]);
//         }
//     }
// }

