/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：usart.c
 * 描述    ：串口驱动
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "usart.h"
#include "data_transfer.h"
#include "ultrasonic.h"
#include <stdio.h>
#include "TZ_PosHold.h"
#include "fly_mode.h"
#include "auto_takeoff.h"


//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存
/////////////////////Usart1 结构数据//////////////////
uint8_t data[5];
uint8_t data_updated;
uint8_t task_updated;

void Usart1_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
//	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1

  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	//使能发送（进入移位）中断
//	if(!(USART1->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
//	}
}




void Usart1_IRQ(void)
{
	u8 com_data;	
    static u8 check_sum = 0;
	uint8_t *p = (uint8_t*)&circle;
	static uint8_t _step = 0, _data_counter = 0;
	
  //接收中断
	if(USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
		
restart:
		switch(_step)	{
			case 0: 								//PREAMBLE1 
				check_sum = 0;
                if(0xAE == com_data)
					_step = 1;
                else if(0xbc == com_data)
                    _step = 5;
				break;
			case 1:									//PREAMBLE2
				if(0x86 == com_data){
					_step = 2;
					_data_counter = 0;	            //数据计数清零
                    check_sum = 0;
				} else 	{
					_step = 0;
					goto restart;
				}
				break;
            case 2:   //Receive Data
				p[_data_counter++] = com_data;
                check_sum += com_data;
				if(_data_counter == 3)	{				//数据长度！！！
					_step = 3;
				}
				break;    
            case 3:
                if(check_sum == com_data)
                    _step = 4;
                else {
                    _step = 0;
                    p[0] = p[1] = p[2] = 0;
                    goto restart;
                }
                break;
            // Checksum and message processing
			case 4:
				_step = 0;
				if(0xDD != com_data)	{
					goto restart;
				}
				//Updated Successfully
				data_updated = 1;   
                break;
                
                
            case 5:
                if(0xf1 == com_data){
                    _step = 6;
                    _data_counter = 0;
                    check_sum = 0;
                }else {
                    _step = 0;
                    goto restart;
                }
                break;
            case 6:
                receive_task = com_data;
                _data_counter++;
                if(_data_counter == 3){
                    _step = 7;
                }
                break;
            case 7:
                check_sum = 3*receive_task;
                if(check_sum == com_data)
                    _step = 8;
                else {
                    _step = 0;
                    goto restart;
                }
                break;
            case 8:
                _step = 0;
                if(0xDD != com_data){
                    goto restart;
                }
                //data_updated = 1;
                task_updated = 1;
                break;
            
            default: 
                _step = 0;
                break;
			
				
        

				//printf("%d %d %d %d %d \r\n", pos_raw.dx, pos_raw.dy, pos_raw.offset, pos_raw.angle, pos_raw.flag);
		}
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
		//USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}
}

void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}

	if(!(USART1->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断
	}

}


void Usart2_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置USART2
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	//配置USART2时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStruct);

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}

void Usart2_IRQ(void)
{
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		ANO_DT_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}



}

void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}

	if(!(USART2->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //打开发送中断
	}

}



void Uart5_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 

void Uart5_IRQ(void)
{
	u8 com_data;

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		
		Ultra_Get(com_data);
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志
          
		if(Tx5Counter == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx5Buffer[count5++] = *(DataToSend+i);
	}

	if(!(UART5->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(UART5, USART_IT_TXE, ENABLE); //打开发送中断
	}

}


void Uart4_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART4, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART4, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}
}

u8 Tx4Buffer[256];
u8 Tx4Counter=0;
u8 count4=0; 

void Uart4_IRQ(void)
{
	u8 com_data;
    static uint8_t _step = 0;
    
  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
re4:
		switch(_step)
        {
            case 0:
                if(0xaa == com_data)
                    _step = 1;
                break;
            case 1:
                _step = 0;
                if(0xc9 == com_data)
                {
                    takeoff_state = LANDING;
                    ALARM_ON;
                }
                else goto re4;
                break;
            default: _step = 0;break;
        }
		
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = Tx4Buffer[Tx4Counter++]; //写DR清除中断标志
          
		if(Tx4Counter == count4)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

}

void Uart4_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx4Buffer[count4++] = *(DataToSend+i);
	}

	if(!(UART4->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(UART4, USART_IT_TXE, ENABLE); //打开发送中断
	}

}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

