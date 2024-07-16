#include "NANO_Tansmit.h"
#include "usart.h"
#include <stdio.h>

#define USART_TX_LEN 4	
uint8_t USART_TX_BUF[USART_TX_LEN];	


extern uint8_t NANO_receive_buff[255];

extern int angle_pich,angle_yaw;
extern int ifstop,key,meet,meet_last;

float Trace_Kp_Pich=-0.025,Trace_Ki_Pich=-0.0003,Trace_Kd_Pich=0;
float Trace_Kp_Yaw=-0.02,Trace_Ki_Yaw=-0.0003,Trace_Kd_Yaw=0.001;


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
	
				meet_last=meet;
				NANO_rec_data.X_AIM_last=NANO_rec_data.X_AIM;
				NANO_rec_data.Y_AIM_last=NANO_rec_data.Y_AIM;
				NANO_rec_data.X_CURRENT_last=NANO_rec_data.X_CURRENT_last;
				NANO_rec_data.Y_CURRENT_last=NANO_rec_data.Y_CURRENT_last;
	
				NANO_rec_data.X_AIM = *((int *) (&NANO_receive_buff[0]));
				NANO_rec_data.Y_AIM = *((int *) (&NANO_receive_buff[4]));
				NANO_rec_data.X_CURRENT = *((int *) (&NANO_receive_buff[8]));
				NANO_rec_data.Y_CURRENT = *((int *) (&NANO_receive_buff[12]));
				ifstop = *((int *) (&NANO_receive_buff[16]));
				meet= *((int *) (&NANO_receive_buff[20]));
	
				NANO_recieve_Limit();
				
				
				printf("AIM:(%d,%d )",NANO_rec_data.Y_AIM,NANO_rec_data.X_AIM);
				printf("CURRENT:(%d,%d )",NANO_rec_data.Y_CURRENT,NANO_rec_data.X_CURRENT);

				printf("ifstop=%d",ifstop);
				printf("meet=%d\n ",meet);
}




/////////////////////////////////////////////////////
void Limit_Angle(void){
//pich
	if(angle_pich<=1350){
		angle_pich=1350;
	}
	if(angle_pich>=1550){
		angle_pich=1550;
	}
	
//yaw
	if(angle_yaw<=1350){
		angle_yaw=1350;
	}
	if(angle_yaw>=1650){
		angle_yaw=1650;
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
    static int Err_last = 0;
    static int Pitch_s = 0;
    static float a = 0.7;
    int Err, Err_LowOut, temp;
    Err = Target - Actual_Angle;
    //Err_LowOut = (1 - a) * Err + a * Err_LowOut_last;
    Pitch_s += Err;
    Pitch_s = Pitch_s > 1900 ? 1900 : (Pitch_s < -1900 ? -1900 : Pitch_s);
    temp = Trace_Kp_Pich * Err + Trace_Ki_Pich * Pitch_s + Trace_Kd_Pich * (Err - Err_last);
    Err_last = Err;

    return temp;
}



void Encoder_Angle(void){

 if(ifstop==0){
	angle_pich+=Trace_Position_PID_Pitch(NANO_rec_data.Y_AIM,NANO_rec_data.Y_CURRENT);
	angle_yaw+=Trace_Position_PID_Yaw(NANO_rec_data.X_AIM,NANO_rec_data.X_CURRENT);
	Limit_Angle();
	printf("aim_angle:(%d,%d)", angle_pich,angle_yaw);
	}

  while (ifstop == 1||key%2==1) {
		angle_pich=angle_pich;
		angle_yaw=angle_yaw;
	
	 }

}