#ifndef _PMU_ADC_H_
#define _PMU_ADC_H_
#include "include.h"


void  Adc_Init(void);
u16 Get_Adc(u8 ch);
void Get_adc_avg(void);   

extern float Bat_volt;
#endif


