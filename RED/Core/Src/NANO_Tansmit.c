#include "NANO_Tansmit.h"
#include "ChuanKoPing.h"
#include "usart.h"
#include <stdio.h>


#define USART_TX_LEN 4	
uint8_t USART_TX_BUF[USART_TX_LEN];	


extern uint8_t NANO_receive_buff[255];
extern int task,ifstop;
extern int id0,angle_pich,id1,angle_yaw;

float Trace_Kp_Pich=-0.02,Trace_Ki_Pich=-0.0002,Trace_Kd_Pich=0;
float Trace_Kp_Yaw=-0.03,Trace_Ki_Yaw=-0.0001,/*-0.01*/Trace_Kd_Yaw=0;/*-0.4*/

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
//处理错误数据
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

void NANO_send(void){//发送任务1234

  	Int_to_Byte(task,&USART_TX_BUF[0]);
  	ble_send(USART_TX_BUF,USART_TX_LEN);
	
	
}

/////////////////////////////////////////////////////////////////////////////////////////
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
	if(angle_yaw<=1200){
		angle_yaw=1200;
	}
	if(angle_yaw>=1800){
		angle_yaw=1800;
	}
}



int Trace_Position_PID_Yaw(int Target, int Actual_Angle)
{
    static int Err_last = 0;
    static int Yaw_s = 0;
    static float a = 0.7;
    int Err, Err_LowOut, temp;
    Err = Target - Actual_Angle;
    //Err_LowOut = (1 - a) * Err + a * Err_LowOut_last;
    Yaw_s += Err;
    Yaw_s = Yaw_s > 1600 ? 1600 : (Yaw_s < -1600 ? -1600 : Yaw_s);
    temp = Trace_Kp_Yaw * Err + Trace_Ki_Yaw * Yaw_s + Trace_Kd_Yaw * (Err - Err_last);
    Err_last = Err;

    return temp;
}

int Trace_Position_PID_Pitch(int Target, int Actual_Angle)
{
    static int Err_LowOut_last = 0;
    static int Pitch_s = 0;
    static float a = 0.7;
    int Err, Err_LowOut, temp;
    Err = Target - Actual_Angle;
    Err_LowOut = (1 - a) * Err + a * Err_LowOut_last;
    Pitch_s += Err_LowOut;
    Pitch_s = Pitch_s > 10000 ? 10000 : (Pitch_s < -10000 ? -10000 : Pitch_s);
    temp = Trace_Kp_Pich * Err_LowOut + Trace_Ki_Pich * Pitch_s + Trace_Kd_Pich * (Err_LowOut - Err_LowOut_last);
    Err_LowOut_last = Err_LowOut;

    return temp;
}



void Encoder_Angle(void){

 if(ifstop==0){
	//angle_pich+=Trace_Position_PID(NANO_rec_data.Y_AIM,NANO_rec_data.Y_CURRENT);
	angle_yaw+=Trace_Position_PID_Yaw(NANO_rec_data.X_AIM,NANO_rec_data.X_CURRENT);
	Limit_Angle();
	printf("aim_angle:(%d,%d)", angle_pich,angle_yaw);
	}

  while (ifstop == 1) {
		angle_pich=angle_pich;
		angle_yaw=angle_yaw;
	
	 }

}