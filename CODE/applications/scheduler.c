/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：scheduler.c
 * 描述    ：任务调度
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "scheduler.h"
#include "include.h"
#include "time.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "led.h"
#include "rc.h"
#include "imu.h"
#include "pwm_in.h"
#include "ctrl.h"
#include "ms5611.h"
#include "parameter.h"
#include "ultrasonic.h"
#include "height_ctrl.h"
#include "fly_mode.h"
#include "setheight.h"
#include "auto_takeoff.h"
#include "usart.h"
#include "TZ_PosHold.h"
#include "TZ_DirectionHold.h"
#include "oled.h"

s16 loop_cnt;
u8 protectflag = 0;

u8 alarm_arr[5] = {0x04,0x02,0x0a,0x0d,0x0c};
loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	
		LED_1ms_DRV( );								//20级led渐变显示
}

void Duty_1ms()
{
	Get_Cycle_T(1)/1000000.0f;

	ANO_DT_Data_Exchange();												//数传通信定时调用
}

float test[5];
void Duty_2ms()
{
	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0)/1000000.0f; 						//获取内环准确的执行周期
	
	test[0] = GetSysTime_us()/1000000.0f;
	
	MPU6050_Read(); 															//读取mpu6轴传感器

	MPU6050_Data_Prepare( inner_loop_time );			//mpu6轴传感器数据处理
	
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	IMUupdate(0.5f *inner_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll,&Pitch,&Yaw);

	CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
	RC_Duty( inner_loop_time , Rc_Pwm_In );				// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
	
	
	
	test[1] = GetSysTime_us()/1000000.0f;
	
	////////////////////ALL UPDATE//////////////////////////////////
	all_PID_Update();
}

void Duty_5ms()
{
	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2)/1000000.0f;								//获取外环准确的执行周期
	
	test[2] = GetSysTime_us()/1000000.0f;

 	CTRL_2( outer_loop_time ); 											// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	
	test[3] = GetSysTime_us()/1000000.0f;
	
	if(Roll > 35 || Roll < -35 || Pitch > 35 || Pitch < -35 || HEIGHT > 190)
	{	
        fly_ready = 0;
        protectflag = 1;
    }
    
    if(HEIGHT > 160)
        takeoff_state = LANDING;
    
}

void Duty_10ms()
{

		
	  ANO_AK8975_Read();			//获取电子罗盘数据	
}

void Duty_20ms()
{
	Parameter_Save();
}

void Duty_50ms()
{
    static u8 lazy_cnt = 20;        //怠速计时
	//Mode();	
	mode_check(CH_filter,mode_value);
	LED_Duty();								//LED任务
	Ultra_Duty();							//超声波
	Screen_Update();
	my_rc_duty();
    
    if(ultra.relative_height < 150 && ultra.relative_height > 50 && circle.valid)
    {
        ALARM_ON;
        Uart4_Send(alarm_arr,4);
    }
    else
    {        
        ALARM_OFF;
        Uart4_Send(alarm_arr+1,4);
    }
    
	if(takeoff_enable == 1)  //使能起飞
	{
		switch ( takeoff_state ){
///////////////////////////////INIT////////////////////////////////////////////////////////
			case READY_FLY :	{
				takeoff_state = ACC_PROCCESS;
			/////////////RESET/////////////////////////////
				althold_PID_Reset_I();
				all_PID_Reset();
                lazy_cnt = 50;
			///////////////////////////////////////////////
				break;
			}
/////////////////////////////TAKEOFF///////////////////////////////////////////////////////
			case ACC_PROCCESS :	{
                if(lazy_cnt > 1) 
                {
                    lazy_cnt--;
                    height_pid_output = -200;
                }
				else if (ultra.relative_height < 40)//第一阶段
				{
					height_pid_output =  takeoff_acc_speed_first;//120
                    PID_update_mode = disabled;
                    
                    if(task_flag == 1)
                    {
                        poshold_pitch_pid_output = 0.6f;
                        poshold_roll_pid_output = -0.8f;
                    }
                    else if(task_flag == 2 || task_flag == 3)
                    {
                        poshold_pitch_pid_output = 0.6f - 0.8f;
                        poshold_roll_pid_output = -0.8f;
                    }
                    
				}
				else 	if (ultra.relative_height < desired_height - 5)//第二阶段
				{
					//PID_update_mode = poshold;					//上升过程定点
                    if(posctrl)
                        PID_update_mode = poshold;						//position control
                    else
                    {
                        PID_update_mode = disabled;
                        poshold_PID_Reset_I();
                    }
					height_pid_output =  ( (40.0f - ultra.relative_height) * 140 / (desired_height - 40) + takeoff_acc_speed_second );//120
					height_pid_output = ( height_pid_output >180 ) ?180 : (height_pid_output<10) ? 10 : height_pid_output;
				}
				 else 
				 {
						takeoff_state = AUTO_HEIGHT;//达到目标高度
					 
						althold_PID_Reset_I();
						all_PID_Reset();
				 }
				break;
			}
/////////////////////////////POSITION_HOLD/////////////////////////////////////////////////
  		case AUTO_HEIGHT :	{
				althold_PID_Update();									//height control
            
                if(posctrl)
                    PID_update_mode = poshold;						//position control
				else
                {
                    PID_update_mode = disabled;
                    poshold_PID_Reset_I();
                }
                
                if(task_flag >= 1)
                {
                    task1_cnt--;
                    if(task1_cnt <= 1)
                    {
                        takeoff_state = LANDING;
                        task1_cnt = 0;
                    }
                }
                
				//Jump to Landing by tuning to right-end
				if(line_tracker == 1)	{
					althold_PID_Reset_I();
					all_PID_Reset();
					takeoff_state = LANDING;
				}
				break;
			}
////////////////////////////////LANDING////////////////////////////////////////////////////
			case LANDING:	{
				height_pid_output = -23.0f;
                    
				if(HEIGHT > 34)
                {
                    if(posctrl && (task_flag == 1))
                        PID_update_mode = poshold;						//position control
                    
                    if(task_flag == 2)
                    {
                        PID_update_mode = disabled;
                        poshold_pitch_pid_output = 0.1f;
                    }
                    
                    if(task_flag == 3)
                    {    
                        if(HEIGHT > 55)
                        {
                            PID_update_mode = poshold;
                        }
                        else 
                        {
                            PID_update_mode = disabled;
                            poshold_pitch_pid_output = 0.2f;
                            poshold_roll_pid_output = 0;
                        }
                    }
                }
				else	
                {
					all_PID_Reset();
                    PID_update_mode = disabled;
                    if(task_flag != 2) poshold_pitch_pid_output = -1.2f;
                    else poshold_pitch_pid_output = 0;
					height_pid_output = -95.0f;
				}
				if(HEIGHT  < 18)
                {
					fly_ready = 0;
                    task_flag = 0;
                }
                
				break;
			}
            default: takeoff_state = LANDING;
		}
	}
	else
		takeoff_state = 0;
	
		////////////////////TESTING//////////////////////////////////////////////////
		//poshold_PID_Update();
		//linehold_PID_Update();
		//dirhold_PID_Update();

		//printf("%d %d %d %d %d \r\n", pos_raw.dx, pos_raw.dy, pos_raw.offset, pos_raw.angle, pos_raw.flag);
//		printf("roll kp %1.3f ki %f kd %3.1f\r\n", PID_PosHold_roll._kp, PID_PosHold_roll._ki, PID_PosHold_roll._kd);
//		printf("pit kp %1.3f ki %f kd %3.1f\r\n", PID_PosHold_pitch._kp, PID_PosHold_pitch._ki, PID_PosHold_pitch._kd);
//		printf("height kp %1.3f ki %1.3f kd %1.3f\r\n",Height_Control._kp, Height_Control._ki, Height_Control._kd);
}


void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}



u16 auto_rc_ch[CH_NUM];

void my_rc_duty(void){
	
		if(NS!=1) Feed_Rc_Dog(2);

		auto_rc_ch[THR] = 1000;
		auto_rc_ch[YAW] = 1500;
		auto_rc_ch[ROL] = 1500;
		auto_rc_ch[PIT] = 1500;
		auto_rc_ch[AUX1] = 2000;
		auto_rc_ch[AUX2] = 1500;
		auto_rc_ch[AUX3] = 2000;
		auto_rc_ch[AUX4] = 1000;
	
	
}



	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

