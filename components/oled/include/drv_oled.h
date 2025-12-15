#ifndef DRV_OLED_H
#define DRV_OLED_H
#include <stdint.h>

#define OLED_ADDR         0x3C        /* OLED address */  


#define OLED_CMD  0	
#define OLED_DATA 1	
#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	
#define MAX_PAGE_SIZE 8
#define MAX_CLOLUMN_SIZE 128

#define PAGE_ADDR   0xB0
#define CMD_MODE 0x40
#define DATA_MODE 0x00


void OLED_Init(void *arg);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);
void OLED_ShowString(uint8_t x,uint8_t y,char *chr,uint8_t Char_Size);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size);
void Oled_ShowExample(void *arg);
void OLED_Task_Init(void);

#endif // DRV_OLED_H
