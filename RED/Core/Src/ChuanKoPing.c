#include "ChuanKoPing.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "usart.h"

extern uint8_t ChuanKoPing_receive_buff[9];//串口屏接收数据
extern int angle_pich,angle_yaw;
extern int task,ifstop;

void Send_To_ChuankoPing(){
	
	//云台1
    uint8_t predefinedData1[] = {0xA5, 0x5A, 0x05, 0x82, 0x00, 0x20};
    uint8_t sendData1[sizeof(predefinedData1) + 2];
    memcpy(sendData1, predefinedData1, sizeof(predefinedData1));
    sendData1[sizeof(predefinedData1)] = (angle_pich >> 8) & 0xFF; //高位
    sendData1[sizeof(predefinedData1) + 1] = angle_pich & 0xFF;    //低位
		HAL_UART_Transmit(&huart3, (uint8_t *)sendData1, sizeof(sendData1),0xFFFF);
		
	//云台2
    uint8_t predefinedData2[] = {0xA5, 0x5A, 0x05, 0x82, 0x00, 0x30};
    uint8_t sendData2[sizeof(predefinedData2) + 2];
    memcpy(sendData2, predefinedData2, sizeof(predefinedData2));
    sendData2[sizeof(predefinedData2)] = (angle_yaw >> 8) & 0xFF; //高位
    sendData2[sizeof(predefinedData2) + 1] = angle_yaw & 0xFF;    //低位
		HAL_UART_Transmit(&huart3, (uint8_t *)sendData2, sizeof(sendData2),0xFFFF);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3){//串口屏是3
		int Task_tmp=ChuanKoPing_receive_buff[8];
	
		if(Task_tmp==11){//暂停B
			ifstop=1;
			
		}
		if(Task_tmp==10){//启动A
			ifstop=0;
		}
		else{
		task=ChuanKoPing_receive_buff[8];
		ifstop=0;
		}
		printf("task=%d\n",task);
		printf("ifstop=%d\n",ifstop);
		
	}
		HAL_UART_Receive_IT(&huart3, (uint8_t *)ChuanKoPing_receive_buff, sizeof(ChuanKoPing_receive_buff));
}