#ifndef __NANO_TANSMIT_H
#define __NANO_TANSMIT_H

#include "stm32f4xx.h"

void NANO_recieve_Limit(void);;
void NANO_recieve(void);
void NANO_send(void);

int ble_send(uint8_t *data,int len);
void Float_to_Byte(float f,uint8_t *byte);
void Short_to_Byte(short s,uint8_t *byte);
void Int_to_Byte(int i,uint8_t *byte);

void Limit_Angle(void);
int Trace_Position_PID(int Target, int Actual_Angle);
void Encoder_Angle(void);

#endif
