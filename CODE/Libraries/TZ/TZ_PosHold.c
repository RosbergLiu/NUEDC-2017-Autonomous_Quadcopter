/**
  ******************************************************************************
  * @file    TZ_PosHold.c
  * @author  Tom Zheng
  * @version V1.0
  * @date    28-Jul-2017
  * @brief   Position hold
  *          
  *            
  ******************************************************************************
  */ 
	
/* Includes ------------------------------------------------------------------*/
#include "TZ_PosHold.h"

// 0.08 0 70.0 0.7
//p 0.09f 50.0f    ***
//MAX1.5  p 0.06 d 70 
TZ_PID_TypeDef PID_PosHold_roll = {
	0.072f,
	0,
	100.0f,
	0.5f,
	0,
	0,
	NAN,
	0.95f
};

TZ_PID_TypeDef PID_PosHold_pitch = {
	0.072f,
	0,
	100.0f,
	0.5f,
	0,
	0,
	NAN,
	0.95f
};

TZ_POS_TypeDef pos_raw;

POINT circle;


/////////////////////////////////////////////////////////////////////////////
float poshold_roll_pid_output, poshold_pitch_pid_output;
int16_t raw_x, raw_y, raw_offset;
float x_cm, y_cm, offset_cm;
///////////////////////////////////////////////
update_mode PID_update_mode = disabled;

void poshold_PID_Reset_I()
{
	PID_reset_I(&PID_PosHold_roll);
	PID_reset_I(&PID_PosHold_pitch);
	poshold_roll_pid_output = poshold_pitch_pid_output = 0;
}

void poshold_PID_Update()
{
//	if(pos_raw.flag == 0 || pos_raw.flag == 2)
//		return;
    
    static u8 lose_cnt;
    
    if(!circle.valid)
    {
        if(lose_cnt < 3)
            lose_cnt++;
    }
    else lose_cnt = 0;
    
    if(lose_cnt == 3)
    {
        poshold_roll_pid_output = 0;
        poshold_pitch_pid_output = 0;
        return;
    }
    
	raw_x = circle.x, raw_y = -circle.y;
	
	x_cm = (float)raw_x * CM_PER_PIXEL - 1;// - HEIGHT * DELTA_THETA_ROLL;
	y_cm = (float)raw_y * CM_PER_PIXEL;// - HEIGHT * DELTA_THETA_PITCH;
	
	poshold_roll_pid_output = PID_get_pid(&PID_PosHold_roll, x_cm, 100);
	poshold_roll_pid_output = poshold_roll_pid_output > 3.0f ? 3.0f : ((poshold_roll_pid_output < -3.0f) ? -3.0f: poshold_roll_pid_output);
	
	poshold_pitch_pid_output = PID_get_pid(&PID_PosHold_pitch, y_cm, 100);
	poshold_pitch_pid_output = poshold_pitch_pid_output > 3.0f ? 3.0f : ((poshold_pitch_pid_output < -3.0f) ? -3.0f : poshold_pitch_pid_output);
	
	//printf("%2.1f, %2.1f, %2.1f, %2.1f\r\n", x_cm, y_cm, poshold_roll_pid_output, poshold_pitch_pid_output);
}

void linehold_PID_Update()
{
	if(pos_raw.flag < 2)
		return;
	
	raw_offset = pos_raw.offset;
	offset_cm = (float)raw_offset * CM_PER_PIXEL - HEIGHT * DELTA_THETA_ROLL;
	
	poshold_roll_pid_output = PID_get_pid(&PID_PosHold_roll, offset_cm, 100);
	poshold_roll_pid_output = poshold_roll_pid_output > 3.0f ? 3.0f : (poshold_roll_pid_output < -3.0f) ? -3.0f: poshold_roll_pid_output;

	poshold_pitch_pid_output = 0;
	
	//printf("R %2.1f, %2.1f\r\n", x_cm, poshold_roll_pid_output);
}

void all_PID_Reset()
{
	PID_update_mode = disabled;
	poshold_PID_Reset_I();
	//dirhold_PID_Reset_I();
}

void all_PID_Update()
{
	if(data_updated == 0)
		return;
	
	if(PID_update_mode == poshold)					//¶¨µã
		poshold_PID_Update();
	
	data_updated = 0;
}
