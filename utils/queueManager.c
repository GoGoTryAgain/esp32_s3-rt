#include "queueManager.h"

QueueMsgManager_t g_msgQueue = {NULL};
AccGyroMsg_t g_accGyroMsg;


void InitQueueManagerTask();
void HandleReceiveData(void *arg);


void initQueueManager() { 
    g_msgQueue.msgQueueAcc = xQueueCreate(10, sizeof(AccMsg_t));
    g_msgQueue.msgQueueGyro = xQueueCreate(3, sizeof(GyroMsg_t));
    InitQueueManagerTask();
}

void InitQueueManagerTask()
{
    xTaskCreate(HandleReceiveData, "HandleReceiveData", 1024, NULL, 2, NULL);
}

void HandleReceiveData(void *arg)
{
    for (;;) {
        BaseType_t ret = xQueueReceive(g_msgQueue.msgQueueAcc, &g_accGyroMsg.acc, portMAX_DELAY);
        if (ret == pdTRUE) {

        }
        
    } 
}

AccMsg_t GetAccData()
{ 
    return g_accGyroMsg.acc;
}