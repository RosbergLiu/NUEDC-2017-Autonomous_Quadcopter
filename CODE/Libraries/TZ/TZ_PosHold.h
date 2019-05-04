/**
  ******************************************************************************
  * @file    TZ_PosHold.h
  * @author  Tom Zheng
  * @version V1.0
  * @date    09-Jul-2017
  * @brief   Position Hold
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POS_HOLD_H
#define __POS_HOLD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "TZ_PID.h"
#include <math.h>
#include "usart.h"
#include "ultrasonic.h"
#include "imu.h"
#include "TZ_DirectionHold.h"
/* Defines -------------------------------------------------------------------*/
	 
#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
#ifndef PI
 #define PI M_PI_F
#endif

#define HEIGHT ultra.relative_height					//in cm
//#define HEIGHT 70.0f					//in cm
	 
#define DELTA_THETA_ROLL  (Roll * PI / 180)               			//in rad
#define DELTA_THETA_PITCH  (Pitch * PI / 180)              	   	//in rad
	 
#define CM_PER_PIXEL ((float)HEIGHT * 0.0078125f)

#define OUTPUT_MAX 2.5f

///////////Circle and line//////////////////////////接收数据格式///////////
typedef struct{
	int16_t dx;
	int16_t dy;
	int16_t offset;
	int16_t angle;
	uint8_t flag;//0无 1只有圆 2只有线 3圆线都有 4拐角(开发中)
}TZ_POS_TypeDef;


typedef enum{
	disabled,
	poshold,
	linehold
}update_mode;

/* Public Variables ----------------------------------------------------------*/
extern TZ_POS_TypeDef pos_raw;
extern POINT    circle;

extern TZ_PID_TypeDef PID_PosHold_roll, PID_PosHold_pitch;

extern float poshold_roll_pid_output;
extern float poshold_pitch_pid_output;

extern update_mode PID_update_mode;										//0禁用 1寻点 2寻线

/* Function Prototypes -------------------------------------------------------*/
void poshold_PID_Reset_I(void);
void poshold_PID_Update(void);

void linehold_PID_Update(void);

void all_PID_Reset(void);
void all_PID_Update(void);

#endif
