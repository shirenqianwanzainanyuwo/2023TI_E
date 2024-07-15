#include "YunTai_UART2.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "usart.h"


uint8_t GET_ANGLE_Pich[9]="#000PRAD!";	//pich
uint8_t GET_ANGLE_Yaw[9]="#001PRAD!";	//yaw

extern uint8_t Print_ANGLE[20];

typedef struct {
		int id0;
    int angle_pich;
		int id1;
    int angle_yaw;
} RecData;

 RecData rec_data;

void recieve(void){
	//云台位置#000P1500!#001P1500!
	//角度
	//pich
		rec_data.angle_pich=0;
		for (int i = 5; i < 9; i++) {
			int digit = Print_ANGLE[i] - '0';
			rec_data.angle_pich = rec_data.angle_pich * 10 + digit;
		}
		//printf("pich is: %d\n", rec_data.angle_pich);
				
		//id
		rec_data.id0 = (int)Print_ANGLE[3] - '0'; 
		//printf("id0 is: %d\n", rec_data.id0);
		
	//yaw
		rec_data.angle_yaw=0;
		for (int i = 15; i < 19; i++) {
			int digit = Print_ANGLE[i] - '0';
			rec_data.angle_yaw = rec_data.angle_yaw * 10 + digit;
		}
		//printf("yaw is: %d\n", rec_data.angle_yaw);
				
		//id
		rec_data.id1 = (int)Print_ANGLE[13] - '0'; 
		//printf("id1 is: %d\n", rec_data.id1);
		
		printf("curent_angle:(%d,%d)\n", rec_data.angle_pich,rec_data.angle_yaw);
			
		//HAL_UART_Transmit(&huart1, (uint8_t *)Print_ANGLE, sizeof(Print_ANGLE),0xFFFF);
			
}


///////////////////////////////
void send(int SendID_Pich,int SendAngle_Pich,int SendID_Yaw,int SendAngle_Yaw){//pich yaw
		
	//角度控制
		//pich
		char SET_ANGLE_Pich[15]; 

    SET_ANGLE_Pich[0] = '#';
    snprintf(SET_ANGLE_Pich + 1, 4, "%03d", SendID_Pich); 
    SET_ANGLE_Pich[4] = 'P';
    snprintf(SET_ANGLE_Pich + 5, 5, "%04d", SendAngle_Pich); 
    SET_ANGLE_Pich[9] = 'T';
    snprintf(SET_ANGLE_Pich + 10, 5, "%04d", 100); 
    SET_ANGLE_Pich[14] = '!';

	
		//yaw
		char SET_ANGLE_Yaw[15]; 

    SET_ANGLE_Yaw[0] = '#';
    snprintf(SET_ANGLE_Yaw + 1, 4, "%03d", SendID_Yaw); 
    SET_ANGLE_Yaw[4] = 'P';
    snprintf(SET_ANGLE_Yaw + 5, 5, "%04d", SendAngle_Yaw); 
    SET_ANGLE_Yaw[9] = 'T';
    snprintf(SET_ANGLE_Yaw + 10, 5, "%04d", 100); 
    SET_ANGLE_Yaw[14] = '!';

		//HAL_UART_Transmit(&huart1, (uint8_t *)SET_ANGLE_Pich, sizeof(SET_ANGLE_Pich),0xFFFF);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)SET_ANGLE_Pich, sizeof(SET_ANGLE_Pich));   
		   
	 HAL_Delay(35);
			
		//HAL_UART_Transmit(&huart1, (uint8_t *)SET_ANGLE_Yaw, sizeof(SET_ANGLE_Yaw),0xFFFF);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)SET_ANGLE_Yaw, sizeof(SET_ANGLE_Yaw));   

    HAL_Delay(35);
		
	
	
	//获取角度
	
		//pich
	 //HAL_UART_Transmit(&huart1, (uint8_t *)GET_ANGLE_Pich, sizeof(GET_ANGLE_Pich),0xFFFF);
   HAL_UART_Transmit_DMA(&huart2, (uint8_t *)GET_ANGLE_Pich, sizeof(GET_ANGLE_Pich));   
		 
		 HAL_Delay(25);

		//yaw
	 //HAL_UART_Transmit(&huart1, (uint8_t *)GET_ANGLE_Yaw, sizeof(GET_ANGLE_Yaw),0xFFFF);
   HAL_UART_Transmit_DMA(&huart2, (uint8_t *)GET_ANGLE_Yaw, sizeof(GET_ANGLE_Yaw));   

	
		HAL_Delay(25);

}