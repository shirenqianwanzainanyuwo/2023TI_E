#include "NANO_Tansmit.h"
#include "ChuanKoPing.h"
#include "usart.h"
#include <stdio.h>


#define USART_TX_LEN 4	
uint8_t USART_TX_BUF[USART_TX_LEN];	


extern uint8_t NANO_receive_buff[255];
extern int task,ifstop;
extern int id0,angle_pich,id1,angle_yaw;

float Trace_Kp=-0.05,Trace_Ki=0.0025,Trace_Kd=0;


typedef struct {
		int X_AIM;
		int Y_AIM;
    int X_CURRENT;
    int Y_CURRENT;
		
		int X_AIM_last;
		int Y_AIM_last;
    int X_CURRENT_last;
    int Y_CURRENT_last;	
	
} NANO_RecData;

 NANO_RecData NANO_rec_data;




////////////////////////////////////////////
//�����������
void NANO_recieve_Limit(void){
	
	if(NANO_rec_data.X_AIM>=620||NANO_rec_data.X_AIM<=0){
		NANO_rec_data.X_AIM=NANO_rec_data.X_AIM_last;
	}
		
	if(NANO_rec_data.X_CURRENT>=620||NANO_rec_data.X_CURRENT<=0){
		NANO_rec_data.X_CURRENT=NANO_rec_data.X_CURRENT_last;
	}
	
	
		
	if(NANO_rec_data.Y_AIM>=480||NANO_rec_data.Y_AIM<=0){
		NANO_rec_data.Y_AIM=NANO_rec_data.Y_AIM_last;
	}
		
	if(NANO_rec_data.Y_CURRENT>=480||NANO_rec_data.Y_CURRENT<=0){
		NANO_rec_data.Y_CURRENT=NANO_rec_data.Y_CURRENT_last;
	}


}

void NANO_recieve(void){
				NANO_rec_data.X_AIM_last=NANO_rec_data.X_AIM;
				NANO_rec_data.Y_AIM_last=NANO_rec_data.Y_AIM;
				NANO_rec_data.X_CURRENT_last=NANO_rec_data.X_CURRENT_last;
				NANO_rec_data.Y_CURRENT_last=NANO_rec_data.Y_CURRENT_last;
	
				NANO_rec_data.X_AIM = *((int *) (&NANO_receive_buff[0]));
				NANO_rec_data.Y_AIM = *((int *) (&NANO_receive_buff[4]));
				NANO_rec_data.X_CURRENT = *((int *) (&NANO_receive_buff[8]));
				NANO_rec_data.Y_CURRENT = *((int *) (&NANO_receive_buff[12]));
				ifstop = *((int *) (&NANO_receive_buff[16]));
				
				NANO_recieve_Limit();
				
	      printf("AIM:(%d,%d )",NANO_rec_data.Y_AIM,NANO_rec_data.X_AIM);
				printf("CURRENT:(%d,%d )",NANO_rec_data.Y_CURRENT,NANO_rec_data.X_CURRENT);

				printf("ifstop=%d \n\n",ifstop);
}

void NANO_send(void){//��������1234

  	Int_to_Byte(task,&USART_TX_BUF[0]);
  	ble_send(USART_TX_BUF,USART_TX_LEN);
	
	
}

////////////////////////////////////////////////////////
//send data to hc-05
int ble_send(uint8_t *data,int len)
{
	while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC)!=SET);	//wait until tx complete
	if(!data)return -1;
	HAL_UART_Transmit_DMA(&huart6,(uint8_t *)data,len);	//ʹ��DMAģʽ����
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
	if(angle_yaw<=1200){
		angle_yaw=1200;
	}
	if(angle_yaw>=1800){
		angle_yaw=1800;
	}
}



int Trace_Position_PID(int Target, int Actual_Angle)
{
    static int Err_LowOut_last = 0;
    static int Encoder_S = 0;
    static float a = 0.7;
    int Err, Err_LowOut, temp;
    Err = Target - Actual_Angle;
    Err_LowOut = (1 - a) * Err + a * Err_LowOut_last;
    Encoder_S += Err_LowOut;
    Encoder_S = Encoder_S > 2000 ? 2000 : (Encoder_S < -2000 ? -2000 : Encoder_S);
    temp = Trace_Kp * Err_LowOut + Trace_Ki * Encoder_S + Trace_Kd * (Err_LowOut - Err_LowOut_last);
    Err_LowOut_last = Err_LowOut;

    return temp;
}

//void Encoder_Angle(void){
////��Χyaw1300-1700,pich1400-1800,10����е�Ƕ�Ϊ��λ

// //��⵽��
// if(ifstop==0){
//	//pich
//		if(NANO_rec_data.Y_AIM>NANO_rec_data.Y_CURRENT){
//			//���ƣ�pich
//			angle_pich=angle_pich-1;
//		}
//		if(NANO_rec_data.Y_AIM<NANO_rec_data.Y_CURRENT){
//			//����,pich
//			angle_pich=angle_pich+1;
//		}
//	
//	//yaw		
//		if(NANO_rec_data.X_AIM>NANO_rec_data.X_CURRENT){
//			//���ƣ�yaw
//			angle_yaw=angle_yaw-1;
//		}
//		if(NANO_rec_data.X_AIM<NANO_rec_data.X_CURRENT){
//			//���ƣ�yaw
//			angle_yaw=angle_yaw+1;
//		}
//		Limit_Angle();
//		printf("aim_angle:(%d,%d)", angle_pich,angle_yaw);

//	}

// //û�м�⵽����������ͣ��ֹͣ
//  while (ifstop == 1) {
//		angle_pich=angle_pich;
//		angle_yaw=angle_yaw;
//	
//	 }

//}

void Encoder_Angle(void){
//??yaw1300-1700,pich1400-1800,10????????

 //????
 if(ifstop==0){
	angle_pich+=Trace_Position_PID(NANO_rec_data.Y_AIM,NANO_rec_data.Y_CURRENT);
	angle_yaw+=Trace_Position_PID(NANO_rec_data.X_AIM,NANO_rec_data.X_CURRENT);
	Limit_Angle();
	printf("aim_angle:(%d,%d)", angle_pich,angle_yaw);
	}

 //?????,?????,??
  while (ifstop == 1) {
		angle_pich=angle_pich;
		angle_yaw=angle_yaw;
	
	 }

}