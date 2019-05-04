/**
  ******************************************************************************
  * @file    setheight.h
  * @author  Tom Zheng
  * @version V1.0
  * @date    27-Jul-2017
  * @brief   PID Controller
  *          
  *          ===================================================================      
  *          Note: This driver is intended for STM32F10x families devices only.
  *          ===================================================================
  *            
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SET_HEIGHT_H
#define __SET_HEIGHT_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
//#include "ultrasonic.h"
#include "TZ_PID.h"

#define ALARM_ON    GPIO_SetBits(GPIOB,GPIO_Pin_10)  
#define ALARM_OFF   GPIO_ResetBits(GPIOB,GPIO_Pin_10)     
	 
extern TZ_PID_TypeDef Height_Control;

extern float height_pid_output;
extern int16_t height_error;

extern int16_t desired_height;

	 
void althold_PID_Reset_I(void);

void althold_PID_Update(void);
     
void PORT3_Init(void);
#endif
