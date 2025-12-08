#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drv_oled.h"
#include "esp_log.h"
#include "drv_MPU6050.h"



void Task_init()
{
    //OLED_Task_Init();

    xTaskCreate(
        MPU6050_Init,   // 任务函数
        "MPU6050_Init",      // 任务名
        8192,         // 栈大小
        NULL,         // 无参数
        1,            // 优先级（高于 IDLE 任务）
        NULL         // 不保存任务句柄
    );
    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}