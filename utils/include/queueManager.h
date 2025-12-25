#ifndef QUEUEMANAGER_H
#define QUEUEMANAGER_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

typedef struct
{
    QueueHandle_t msgQueueAcc;
    QueueHandle_t msgQueueAcc2Ble;
    QueueHandle_t msgQueueGyro;
} QueueMsgManager_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} AccMsg_t;

typedef struct
{
    float x;
    float y;
    float z;
} GyroMsg_t;

extern QueueMsgManager_t g_msgQueue;

void initQueueManager();

#endif