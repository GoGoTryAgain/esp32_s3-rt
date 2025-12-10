#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "drv_oled.h"
// #include "esp_log.h"
// #include "drv_MPU6050.h"
#include "app_task.h"
#include "queueManager.h"

static const char *TAG = "app_main";

void app_main(void)
{
    initQueueManager();
    //test_gpio_pins();
    Task_init();

}