#include "NANO_Tansmit.h"
#include "usart.h"
#include <stdio.h>

#define USART_TX_LEN 4	
uint8_t USART_TX_BUF[USART_TX_LEN];	


extern uint8_t NANO_receive_buff[255];

extern int angle_pich,angle_yaw;
extern int ifstop,key,meet,meet_last;


typedef struct {
		int X_AIM;
    int X_CURRENT;
		int Y_AIM;
    int Y_CURRENT;
	  int NANO_STOP;

} NANO_RecData;

 NANO_RecData NANO_rec_data;




////////////////////////////////////////////
void NANO_recieve(void){
	
				meet_last=meet;
				NANO_rec_data.X_CURRENT = *((int *) (&NANO_receive_buff[0]));
				NANO_rec_data.Y_CURRENT = *((int *) (&NANO_receive_buff[4]));
				NANO_rec_data.X_AIM = *((int *) (&NANO_receive_buff[8]));
				NANO_rec_data.Y_AIM = *((int *) (&NANO_receive_buff[12]));
				NANO_rec_data.NANO_STOP = *((int *) (&NANO_receive_buff[16]));
				meet= *((int *) (&NANO_receive_buff[20]));
				
				
				printf("X_CURRENT=%d ",NANO_rec_data.X_CURRENT);
				printf("Y_CURRENT=%d ",NANO_rec_data.Y_CURRENT);
				printf("X_AIM=%d ",NANO_rec_data.X_AIM);				
				printf("Y_AIM=%d ",NANO_rec_data.Y_AIM);
				printf("NOT_FIND_AIM=%d ",NANO_rec_data.NANO_STOP);
				printf("meet=%d ",meet);
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
 if(NANO_rec_data.NANO_STOP==0){
	//pich
		if(NANO_rec_data.Y_AIM>NANO_rec_data.Y_CURRENT){
			//下移，pich
			angle_pich=angle_pich+1;
		}
		if(NANO_rec_data.Y_AIM<NANO_rec_data.Y_CURRENT){
			//上移,pich
			angle_pich=angle_pich-1;
		}
	
	//yaw		
		if(NANO_rec_data.X_AIM>NANO_rec_data.X_CURRENT){
			//右移，yaw
			angle_yaw=angle_yaw+1;
		}
		if(NANO_rec_data.X_AIM<NANO_rec_data.X_CURRENT){
			//左移，yaw
			angle_yaw=angle_yaw-1;
		}
		Limit_Angle();

	}

 //没有检测到,按键按下
 if(NANO_rec_data.NANO_STOP==1&key%2==1){
		angle_pich=angle_pich;
		angle_yaw=angle_yaw;
	
	 }

}