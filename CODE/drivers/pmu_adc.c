#include "pmu_adc.h"
#define PDEF_BAT_GAIN   								0.008834     //0.00635
#define PDEF_BAT_ALARMLV1   						1111			 
#define PDEF_BAT_ALARMLV2   						1080		

void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道15 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PC5 通道15
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
}				  
 
 
u16 Get_Adc(u8 ch)   
{
 	
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ))
	{
		;
	}
  ADC1->SR=0;
	
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
 
float Bat_volt=0.0f;
uint8_t Bat_arm_level=0;
uint16_t curr_adc=0;	 
uint32_t sumadc=0;
uint8_t adcidx=0;
uint8_t adc_error=0;
uint16_t avg_adc;

void Get_adc_avg(void)
{	
	curr_adc=Get_Adc(ADC_Channel_15);
	if(curr_adc>4096) 
	{
		adc_error++;
	}		
	adcidx++;
	sumadc+=curr_adc;
	if(adcidx>=100)
	{
		avg_adc=sumadc/(float)adcidx;		
		adcidx=0;
		sumadc=0;
	}
	Bat_volt=avg_adc*PDEF_BAT_GAIN;//avg_adc/4096.0f*3.3f*PDEF_BAT_GAIN;		
	if(Bat_volt!=0)
	{
		if(Bat_volt<PDEF_BAT_ALARMLV2/100.0F)//如果低于10.80V 进入二级保护
			Bat_arm_level=0;//2;		
		else if(Bat_volt<PDEF_BAT_ALARMLV1/100.0F)//如果低于11.11V 进入一级保护
			Bat_arm_level=0;//1;
		else
			Bat_arm_level=0;
	}
}
 


