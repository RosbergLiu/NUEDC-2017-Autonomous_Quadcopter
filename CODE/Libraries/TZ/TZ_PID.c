/**
  ******************************************************************************
  * @file    TZ_PID.c
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
	
/* Includes ------------------------------------------------------------------*/
#include "TZ_PID.h"


void PID_Init(TZ_PID_TypeDef *instance, float kp, float ki, float kd, int16_t imax)
{
    instance->_kp = kp;
    instance->_ki = ki;
    instance->_kd = kd;
    instance->_imax = imax;

	  instance->_integrator = 0;
    instance->_last_input = 0;
    instance->_last_derivative = NAN;
	  instance->_d_lpf_alpha = AC_PID_D_TERM_FILTER;
}

float PID_get_p(TZ_PID_TypeDef *instance, float error)
{
    return (float)error * instance->_kp;
}
float PID_get_i(TZ_PID_TypeDef *instance, float error, float dt)
{
    if((instance->_ki != 0) && (dt != 0)) {
        instance->_integrator += ((float)error * instance->_ki) * dt;
        if (instance->_integrator < -instance->_imax) {
            instance->_integrator = -instance->_imax;
        } else if (instance->_integrator > instance->_imax) {
            instance->_integrator = instance->_imax;
        }
        return instance->_integrator;
    }
    return 0;
}
float PID_get_d(TZ_PID_TypeDef *instance, float input, float dt)
{
    float derivative;
    if ((instance->_kd != 0) && (dt != 0)) {
		if (isnan(instance->_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			instance->_last_derivative = 0;
		} else {
			// calculate instantaneous derivative
			derivative = (input - instance->_last_input) / dt;
		}

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        derivative = instance->_last_derivative + instance->_d_lpf_alpha * (derivative - instance->_last_derivative);

        // update state
        instance->_last_input             = input;
        instance->_last_derivative    = derivative;

        // add in derivative component
        return instance->_kd * derivative;
    }
    return 0;
}
float PID_get_pid(TZ_PID_TypeDef *instance, float error, float dt)
{
    return PID_get_p(instance, error) + PID_get_i(instance, error, dt) + PID_get_d(instance, error, dt);
}

void PID_reset_I(TZ_PID_TypeDef *instance)
{
    instance->_integrator = 0;
	// mark derivative as invalid
    instance->_last_derivative = NAN;
}
void PID_set_d_lpf_alpha(TZ_PID_TypeDef *instance, int16_t cutoff_frequency, float time_step)
{
    float rc;
    // calculate alpha
    rc = 1/(2*PI*cutoff_frequency);
    instance->_d_lpf_alpha = time_step / (time_step + rc);
}
