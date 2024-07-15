#include "NANO_Tansmit.h"
#include "ChuanKoPing.h"
#include "usart.h"
#include <stdio.h>


#define USART_TX_LEN 4	
uint8_t USART_TX_BUF[USART_TX_LEN];	


extern uint8_t NANO_receive_buff[255];
extern int task,ifstop;
extern int id0,angle_pich,id1,angle_yaw;


typedef struct {
		int X_AIM;
    int X_CURRENT;
		int Y_AIM;
    int Y_CURRENT;
} NANO_RecData;

 NANO_RecData NANO_rec_data;




////////////////////////////////////////////
void NANO_recieve(void){
				NANO_rec_data.X_AIM = *((int *) (&NANO_receive_buff[0]));
				NANO_rec_data.Y_AIM = *((int *) (&NANO_receive_buff[4]));
				NANO_rec_data.X_CURRENT = *((int *) (&NANO_receive_buff[8]));
				NANO_rec_data.Y_CURRENT = *((int *) (&NANO_receive_buff[12]));
				ifstop = *((int *) (&NANO_receive_buff[16]));
				
	      printf("AIM:(%d,%d )",NANO_rec_data.Y_AIM,NANO_rec_data.X_AIM);
				printf("CURRENT:(%d,%d )",NANO_rec_data.Y_CURRENT,NANO_rec_data.X_CURRENT);

				printf("ifstop=%d \n\n",ifstop);
}

void NANO_send(void){//发送任务1234

  	Int_to_Byte(task,&USART_TX_BUF[0]);
  	ble_send(USART_TX_BUF,USART_TX_LEN);
	
	
}

////////////////////////////////////////////////////////
//send data to hc-05
int ble_send(uint8_t *data,int len)
{
	while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC)!=SET);	//wait until tx complete
	if(!data)return -1;
	HAL_UART_Transmit_DMA(&huart6,(uint8_t *)data,len);	//使用DMA模式发送
	while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC)!=SET);
	return 0;
}

//change to byte
void Int_to_Byte(int i,uint8_t *byte)
{

	unsigned long longdata = 0;
	longdata = *(unsigned long*)&i;          
	byte[3] = (longdata & 0xFF000000) >> 24;
	byte[2] = (longdata & 0x00FF0000) >> 16;
	byte[1] = (longdata & 0x0000FF00) >> 8;
	byte[0] = (longdata & 0x000000FF);

}
void Float_to_Byte(float f,uint8_t *byte)
{

	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;          
	byte[3] = (longdata & 0xFF000000) >> 24;
	byte[2] = (longdata & 0x00FF0000) >> 16;
	byte[1] = (longdata & 0x0000FF00) >> 8;
	byte[0] = (longdata & 0x000000FF);

}

void Short_to_Byte(short s,uint8_t *byte)
{
      
	byte[1] = (s & 0xFF00) >> 8;
	byte[0] = (s & 0xFF);
}

/////////////////////////////////////////////////////
void Limit_Angle(void){
//pich
	if(angle_pich<=1300){
		angle_pich=1300;
	}
	if(angle_pich>=1700){
		angle_pich=1700;
	}
	
//yaw
	if(angle_yaw<=1400){
		angle_yaw=1400;
	}
	if(angle_yaw>=1800){
		angle_yaw=1800;
	}
}

void Encoder_Angle(void){
//范围yaw1300-1700,pich1400-1800,10个机械角度为单位

 //检测到了
 if(ifstop==0){
	//pich
		if(NANO_rec_data.Y_AIM>NANO_rec_data.Y_CURRENT){
			//下移，pich
			angle_pich=angle_pich-1;
		}
		if(NANO_rec_data.Y_AIM<NANO_rec_data.Y_CURRENT){
			//上移,pich
			angle_pich=angle_pich+1;
		}
	
	//yaw		
		if(NANO_rec_data.X_AIM>NANO_rec_data.X_CURRENT){
			//右移，yaw
			angle_yaw=angle_yaw-1;
		}
		if(NANO_rec_data.X_AIM<NANO_rec_data.X_CURRENT){
			//左移，yaw
			angle_yaw=angle_yaw+1;
		}
		Limit_Angle();
		printf("aim_angle:(%d,%d)", angle_pich,angle_yaw);

	}

 //没有检测到，串口屏暂停，停止
  while (ifstop == 1) {
		angle_pich=angle_pich;
		angle_yaw=angle_yaw;
	
	 }

}