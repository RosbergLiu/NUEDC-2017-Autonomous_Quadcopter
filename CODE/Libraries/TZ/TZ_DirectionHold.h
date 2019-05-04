/**
  ******************************************************************************
  * @file    TZ_DirectionHold.h
  * @author  Tom Zheng
  * @version V1.0
  * @date    09-Jul-2017
  * @brief   TZ_DirectionHold
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DIR_HOLD_H
#define __DIR_HOLD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "TZ_PID.h"
#include <math.h>
#include "usart.h"
#include "TZ_PosHold.h"
//#include "ultrasonic.h"
//#include "imu.h"
/* Defines -------------------------------------------------------------------*/

#define YAW_OUTPUT_MAX 2.5f

/* Public Variables ----------------------------------------------------------*/
extern TZ_PID_TypeDef PID_DirHold;

extern float dirhold_yaw_pid_output;
	
/* Function Prototypes -------------------------------------------------------*/
void dirhold_PID_Reset_I(void);
void dirhold_PID_Update(void);


#endif
