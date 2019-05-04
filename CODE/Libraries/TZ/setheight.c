/**
  ******************************************************************************
  * @file    setheight.c
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
#include "setheight.h"

#include "ultrasonic.h"

//p 0.28 d 0.01     1690 
TZ_PID_TypeDef Height_Control = {
	0.28f,
	0,
	0.01f,
	100,
	0,
	0,
	NAN,
	0.95
};


int16_t desired_height = 105;

float height_pid_output;
int16_t height_error;

void althold_PID_Reset_I()
{
	PID_reset_I(&Height_Control);
	height_pid_output =  0;
}
void althold_PID_Update()
{
	height_error = desired_height - ultra.relative_height;
	height_pid_output = PID_get_pid(&Height_Control, height_error, 50);
	height_pid_output = height_pid_output > 15 ? 15 : (height_pid_output < -20) ? -20 : height_pid_output;
}


void PORT3_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
  
  //��ʼ����������Ӧ���� B10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//�ⲿ�����ߵ���
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
	
  GPIO_ResetBits(GPIOB,GPIO_Pin_10);  //��������Ӧ����GPIOB10����
}
