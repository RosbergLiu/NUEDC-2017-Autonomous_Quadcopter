#ifndef __AUTO_TAKEOFF_H
#define __AUTO_TAKEOFF_H


#include "stm32f4xx.h"


#define READY_FLY	0
#define ACC_PROCCESS	1
#define AUTO_HEIGHT	2
#define LINE_TRACKER 3
#define LANDING 4

#define TURNING_LEFT 5

//change in flymode 
extern u8 takeoff_state;
extern u8 takeoff_enable;
extern	u8 takeoff_acc_speed_first ;   
extern	u8 takeoff_acc_speed_second  ;
extern 	uint8_t line_tracker;
#endif

