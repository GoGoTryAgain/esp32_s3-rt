#include "drv_oled.h"
#include "font.h"
#include "esp_log.h"
#include "esp_err.h"
#include "I2c_ctrl.h"
#include "queueManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


static const char *TAG = "oled_driver";
TaskHandle_t show_example_handle = NULL;

static i2c_master_dev_handle_t I2C_dev_handle = NULL;
static i2c_master_bus_handle_t I2C_bus_handle = NULL;

SemaphoreHandle_t gInitSem;

void ShowMpu60xData(void *arg);


void OLED_WR_Byte(unsigned data,unsigned DataMode)
{
    esp_err_t ret;
    uint8_t write_buf[2] = {0};
    if (DataMode != 0) {
        write_buf[0] = 0x40; // Command mode
        write_buf[1] = data;
        ret = i2c_master_transmit(I2C_dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
    } else {
        write_buf[0] = 0x00; // Command mode
        write_buf[1] = data;
        ret = i2c_master_transmit(I2C_dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
    }
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "I2C transmit failed， ack error!:%d", ret);
    }
}

void OLED_RegInit(void)
{
    OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//
	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//
	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//
	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//
	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//
	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//
	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//
	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
}

static void i2c_master_init()
{
    I2C_bus_handle = GetI2CBusHandle(I2C_MASTER_NUM);
    ESP_ERROR_CHECK(I2C_bus_handle != NULL ? ESP_OK : ESP_FAIL);


    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(I2C_bus_handle, &dev_config, &I2C_dev_handle));
}


void OLED_Task_Init(void)
{
    gInitSem = xSemaphoreCreateBinary();
    xTaskCreate(
        OLED_Init,   // 任务函数
        "oled_Init",      // 任务名
        2048,         // 栈大小
        NULL,         // 无参数
        2,            // 优先级（高于 IDLE 任务）
        NULL         // 不保存任务句柄
    );
    // xTaskCreate(
    //     Oled_ShowExample,
    //     "Oled_ShowExample",
    //     8192,
    //     NULL,
    //     1,
    //     &show_example_handle
    // );
    xTaskCreate(
        ShowMpu60xData,
        "ShowMpu60xData",
        8192,
        NULL,
        1,
        NULL
    );

    UBaseType_t stack_words = uxTaskGetStackHighWaterMark(show_example_handle);
    ESP_LOGI(TAG, "stack size: %d words", stack_words);
}
void OLED_Init(void *arg)
{ 	
    i2c_master_init();
    OLED_RegInit();
    ESP_LOGI(TAG, "to send sem");
    xSemaphoreGive(gInitSem); // 释放信号量
    vTaskDelete(NULL); // 删除当前任务，不会触发错误
}  

void ShowMpu60xData(void *arg)
{
    AccMsg_t recv_msg;
    for (;;) {
        BaseType_t ret = xQueueReceive(g_msgQueue.msgQueueAcc, &recv_msg, portMAX_DELAY);
        if (show_example_handle != NULL) {
            vTaskDelete(show_example_handle);
            show_example_handle = NULL;
        }
        if (ret == pdTRUE) {
            OLED_ShowString(0,0,"x:",16);  
            OLED_ShowNum(6, 0, recv_msg.x, 4, 16);
            OLED_ShowString(0,2,"y:",16);  
            OLED_ShowNum(6, 2, recv_msg.y, 4, 16);
            OLED_ShowString(0,4,"z:",16);  
            OLED_ShowNum(6, 4, recv_msg.z, 4, 16);
        }
    } 
}


void OLED_Clear(void)  
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);   
		OLED_WR_Byte (0x00,OLED_CMD);
		OLED_WR_Byte (0x10,OLED_CMD);
		for(n=0;n<128;n++)
        {
            OLED_WR_Byte(0,OLED_DATA);
        }
	}
}

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte((((x+2)&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte(((x+2)&0x0f),OLED_CMD); 
} 

void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';
		if(x>Max_Column-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else {	
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_Byte(F6x8[c][i],OLED_DATA);
				
			}
}

void OLED_ShowString(uint8_t x,uint8_t y,char *chr,uint8_t Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{	
        OLED_ShowChar(x,y,chr[j],Char_Size);
		x+=8;
		if (x>120) {
            x=0;
            y+=2;
        }
		j++;
	}
}

uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
void OLED_ShowNum(uint8_t x,uint8_t y,uint8_t num,uint8_t len,uint8_t size2)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 

void check_stack_usage() {
    if (show_example_handle != NULL) {
        // uxTaskGetStackHighWaterMark返回剩余栈空间（单位：字，ESP32中1字=4字节）
        UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(show_example_handle);
        // 计算已使用栈空间 = 总栈大小 - 剩余栈空间（注意单位转换）
        uint32_t total_stack = 8192; // 创建任务时的栈大小（字节）
        uint32_t used_stack = total_stack - stack_high_water_mark;
        uint32_t remaining_stack = stack_high_water_mark;
        ESP_LOGI(TAG, "max:%u, used:%u, remain:%u", total_stack, used_stack, remaining_stack);
    }
}

void Oled_ShowExample(void *arg)
{
    ESP_LOGI(TAG, "enter get sem");
    if (xSemaphoreTake(gInitSem, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "get oled inited");
    }
    ESP_LOGI(TAG, "to clear oled");
    
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
        check_stack_usage();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}	
}