/**
  ******************************************************************************
  * @file    TZ_PID.h
  * @author  Tom Zheng
  * @version V1.0
  * @date    09-Jul-2017
  * @brief   PID_Controller
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_XXX_H
#define __PID_XXX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <math.h>
	 
/* Defines -------------------------------------------------------------------*/
	 
#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
#ifndef PI
 # define PI M_PI_F
#endif

typedef struct{
    float        _kp;
    float        _ki;
    float        _kd;
    float    	 _imax;

    float           _integrator;                                ///< integrator value
    float           _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    float           _d_lpf_alpha;                               ///< alpha used in D-term LPF
}TZ_PID_TypeDef;

typedef struct{
    s8      x;
    s8      y;
    u8      valid;
}POINT;



// Examples for _filter:
// f_cut = 10 Hz -> _alpha = 0.385869
// f_cut = 15 Hz -> _alpha = 0.485194
// f_cut = 20 Hz -> _alpha = 0.556864
// f_cut = 25 Hz -> _alpha = 0.611015
// f_cut = 30 Hz -> _alpha = 0.653373

#define AC_PID_D_TERM_FILTER 0.556864f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency


/* Public Variables ----------------------------------------------------------*/

/* Function Prototypes -------------------------------------------------------*/
void PID_Init(TZ_PID_TypeDef *instance, float kp, float ki, float kd, int16_t imax);
float PID_get_p(TZ_PID_TypeDef *instance, float error);
float PID_get_i(TZ_PID_TypeDef *instance, float error, float dt);
float PID_get_d(TZ_PID_TypeDef *instance, float input, float dt);
float PID_get_pid(TZ_PID_TypeDef *instance, float error, float dt);
void PID_reset_I(TZ_PID_TypeDef *instance);
void PID_set_d_lpf_alpha(TZ_PID_TypeDef *instance, int16_t cutoff_frequency, float time_step);
#endif
