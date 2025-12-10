#include "queueManager.h"

QueueMsgManager_t g_msgQueue = {NULL};


void initQueueManager() { 
    g_msgQueue.msgQueueAcc = xQueueCreate(10, sizeof(AccMsg_t));
    g_msgQueue.msgQueueGyro = xQueueCreate(3, sizeof(GyroMsg_t));
}

