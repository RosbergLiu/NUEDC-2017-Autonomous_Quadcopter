/**
  ******************************************************************************
  * @file   TZ_DirectionHold.c
  * @author  Tom Zheng
  * @version V1.0
  * @date    28-Jul-2017
  * @brief   TZ_DirectionHold
  *          
  *            
  ******************************************************************************
  */ 
	
/* Includes ------------------------------------------------------------------*/
#include "TZ_DirectionHold.h"

TZ_PID_TypeDef PID_DirHold = {
	0.01f,		//p
	0,				//i
	0,				//d
	0.8f,			//imax
	0,
	0,
	NAN,
	0.85f			//滤波（0~1  1为滤波程度最小）
};

float dirhold_yaw_pid_output;
	
int16_t raw_angle;

void dirhold_PID_Reset_I()
{
	PID_reset_I(&PID_DirHold);
	dirhold_yaw_pid_output = 0;
}

void dirhold_PID_Update()
{
	///////////////////////角度量有效条件////////////////////////////////
	if(pos_raw.flag == 0)
		return;
	
	if(pos_raw.angle == 0)
	{
		dirhold_yaw_pid_output = 0;
		return;
	}
	
	raw_angle = - pos_raw.angle;		//角度变量

	
	dirhold_yaw_pid_output = PID_get_pid(&PID_DirHold, raw_angle, 100);
	dirhold_yaw_pid_output = -(dirhold_yaw_pid_output > 0.05f ? 0.05f : (dirhold_yaw_pid_output < -0.05f) ? -0.05f: dirhold_yaw_pid_output);//输出量限幅

	//printf("%2.1f, %2.1f, %2.1f, %2.1f\r\n", x_cm, p, d, dirhold_yaw_pid_output);
	//printf("%2.1f, %2.1f, %2.1f, %2.1f\r\n", x_cm, y_cm, poshold_roll_pid_output, poshold_pitch_pid_output);
	//printf("Y %d,%2.1f\r\n", raw_angle,dirhold_yaw_pid_output);
	
}

