#include "pmu_adc.h"
#define PDEF_BAT_GAIN   								0.008834     //0.00635
#define PDEF_BAT_ALARMLV1   						1111			 
#define PDEF_BAT_ALARMLV2   						1080		

void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

  //�ȳ�ʼ��ADC1ͨ��15 IO��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PC5 ͨ��15
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
 
	ADC_Cmd(ADC1, ENABLE);//����ADת����	
}				  
 
 
u16 Get_Adc(u8 ch)   
{
 	
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ))
	{
		;
	}
  ADC1->SR=0;
	
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
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
		if(Bat_volt<PDEF_BAT_ALARMLV2/100.0F)//�������10.80V �����������
			Bat_arm_level=0;//2;		
		else if(Bat_volt<PDEF_BAT_ALARMLV1/100.0F)//�������11.11V ����һ������
			Bat_arm_level=0;//1;
		else
			Bat_arm_level=0;
	}
}
 


