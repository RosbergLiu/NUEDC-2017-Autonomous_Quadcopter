#include "fly_mode.h"
#include "rc.h"
#include "auto_takeoff.h"

u8 mode_value[10];
u8 mode_state,mode_state_old;
u8 receive_task = 0;
u8 task_flag = 0;
s16 task1_cnt = 0;
u8 posctrl = 1;

void mode_check(float *ch_in,u8 *mode_value)
{
	static u8 task_cnt = 0;
    
    if(NS == 1)
    {
        if(*(ch_in+AUX1) <-200)
        {
            mode_state = 0;//0;
        }
        else if(*(ch_in+AUX1) >200)
        {
            mode_state = 2;
        }
        else
        {
            mode_state = 1;
        }
        
        if(*(ch_in+AUX2) > -200)
        {
            takeoff_enable = 1;
            if(*(ch_in+AUX2) > 300)
                line_tracker = 1;
            else
                line_tracker = 0;
        }
        else
        {
            line_tracker = 0;
            takeoff_enable = 0;//0;
        }
        
        if(*(ch_in + AUX1) > 200 && *(ch_in + AUX2) > -200 && *(ch_in + AUX2) < 200)
        {
            if(task_cnt < 5 && *(ch_in + AUX3) > 300)
            task_cnt++;
        }
        else task_cnt = 0;
        
        if(task_cnt == 4)
        {
            task_cnt++;
            if(*(ch_in + AUX4) < -200)
            {
                task_flag = 1;
            }
            else if(*(ch_in + AUX4) < 200) task_flag = 2;
						else task_flag = 3;
            fly_ready = 1;
            task1_cnt = 100;
						if(task_flag == 3) task1_cnt = 1800;
        }
        else if(task_cnt < 4) 
            task_flag = 0;
        
        
    }
	
    else{
        if(task_updated)   //无信号
        {
            task_updated = 0;
            task_flag = receive_task;
            if(1 == task_flag || 2 == task_flag)
            {
                takeoff_state = 0;
                mode_state = 2;
                takeoff_enable = 1;
                posctrl = 1;
                fly_ready = 1;
                task1_cnt = 150;
            }
            if(3 == task_flag)
            {
                takeoff_state = 0;
                mode_state = 2;
                takeoff_enable = 1;
                posctrl = 1;
                fly_ready = 1;
                task1_cnt = 1800;
            }
        }
    }
	
		///////////fly-mode.c////////////////////ZSF--takeoff_enable/////////////////////
	
//	if(*(ch_in+AUX2) > -200)
//	{
//		takeoff_enable = 1;
//		if(*(ch_in+AUX2) > 300)
//			line_tracker = 1;
//		else
//			line_tracker = 0;
//	}
//	else
//	{
//		line_tracker = 0;
//		takeoff_enable = 0;//0;
//	}
//    
//    if(*(ch_in + AUX3) > 300)
//        task_flag = 0;
//    
//    
//    if(*(ch_in + AUX1) > 200 && *(ch_in + AUX2) > -200 && *(ch_in + AUX2) < 200)
//    {
//        if(task_cnt < 5 && *(ch_in + AUX3) > 300)
//        task_cnt++;
//    }
//    else task_cnt = 0;
//    
//    if(task_cnt == 4)
//    {
//        task_cnt++;
//        if(*(ch_in + AUX4) < -200)
//        {
//            task_flag = 1;
//        }
//        else task_flag = 2;
//        fly_ready = 1;
//        task1_cnt = 100;
//    }
//    else if(task_cnt < 4) 
//        task_flag = 0;




//	if(*(ch_in+AUX2) > 300)
//	{
//		takeoff_enable = 1;
//	}
//	else
//	{
//		takeoff_enable = 0;//0;
//	}
//////////////////////////////////////////////////////////////////
	
		
	
	//=========== GPS、气压定高 ===========
    if(NS == 1)
    {
        if(mode_state == 0 )
        {
            *(mode_value+GPS) = *(mode_value+BARO) = 0;
        }
        else
        {
            *(mode_value+GPS) = *(mode_value+BARO) = 1;
        }
    }
    else
        *(mode_value+GPS) = *(mode_value+BARO) = 1;
//	//=========== 返航模式 ===========
//	if(fly_ready )
//	{
//		if(( mode_state == 2 && mode_state_old != 2) || rc_lose == 1)
//		{

//			*(mode_value+BACK_HOME) = 1;
//			

//		}
//		else if(mode_state != 2)
//		{
//			*(mode_value+BACK_HOME) = 0;
//		}
//	}
//	else
//	{
//		*(mode_value+BACK_HOME) = 0;
//	}
	
	
 
	//===========   ===========
	mode_state_old = mode_state; //历史模式
}
