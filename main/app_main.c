#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drv_oled.h"
#include "esp_log.h"

static const char *TAG = "app_main";

void app_main(void)
{
    //test_gpio_pins();

    ESP_LOGI(TAG, "OLED_Init");
    OLED_Init(); 
    ESP_LOGI(TAG, "clear oled");
    OLED_Clear(); 

    char t = ' ';
    OLED_ShowString(3,1,"0.96' OLED TEST",16); 
    OLED_ShowString(0,6,"ASCII:",16);  
    OLED_ShowString(63,6,"CODE:",16); 
    OLED_ShowChar(48,6,t,16);	
	while(1) 
	{		
		t++;
		if (t > '~') {
            t = ' ';
        }
		OLED_ShowNum(103,6,t,3,16);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}	

}