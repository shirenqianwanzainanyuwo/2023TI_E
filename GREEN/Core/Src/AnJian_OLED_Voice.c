#include "AnJian_OLED_Voice.h"
#include "stm32f4xx_it.h"
#include <stdio.h>
#include "oled.h"
#include "usart.h"

int key;
extern int meet,meet_last;
extern int angle_pich,angle_yaw;
uint8_t OLED_Display[20];

//语音模块
uint8_t Voice_StopStart[6]={0xAA,0x07,0x02,0x00,0x01,0xB4};
uint8_t Voice_Follow[6]={0xAA,0x07,0x02,0x00,0x02,0xB5};
uint8_t Voice_Stop[4]={0xAA,0x03,0x00,0xAD};


//key奇数暂停，偶数启动
void key_scan(void)
{

   if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET)
  {
		OLED_Clear();
    key++;
		printf("key=%d\n",key);
		HAL_UART_Transmit(&huart3, (uint8_t *)Voice_StopStart, sizeof(Voice_StopStart),0xFFFF);
		

	}
	
}

void OLED_Printf(void){

	if(meet==1){
	//已跟踪到
		OLED_ShowCHinese1(0,0,0); 
		OLED_ShowCHinese1(16,0,1);	
		OLED_ShowCHinese1(32,0,2);	
		OLED_ShowCHinese1(48,0,3);	
		if(key%2==0&meet==meet_last){
		 HAL_UART_Transmit(&huart3, (uint8_t *)Voice_Follow, sizeof(Voice_Follow),0xFFFF);
		}
	}
	if(meet==0){
	//未跟踪到
		OLED_ShowCHinese2(0,0,0); 
		OLED_ShowCHinese2(16,0,1);	
		OLED_ShowCHinese2(32,0,2);	
		OLED_ShowCHinese2(48,0,3);	
		
	}
		
	//启动
	OLED_ShowCHinese4(0,2,0); 
	OLED_ShowCHinese4(16,2,1);
	//OLED显示目标角度
	sprintf((char *)OLED_Display,"Pich:%d",angle_pich);
	OLED_ShowString(0,4,OLED_Display, 16);
	sprintf((char *)OLED_Display,"Yaw:%d",angle_yaw);
	OLED_ShowString(0,6,OLED_Display, 16);

}



