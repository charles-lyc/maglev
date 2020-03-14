#include "adc.h"
#include "gpio.h"
#include "configure.h"

static volatile uint16_t ADC_Buffer[ADC_CHANNEL_NUM];

void ADC_DMA_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_ChannelConfig(ADC1, ADC_Channel_0 ,ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1 ,ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_2 ,ADC_SampleTime_239_5Cycles);

	GPIO_InitSimple(GPIOA, GPIO_Pin_0, GPIO_MODE_ANALOG, GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_10MHz, false);
	GPIO_InitSimple(GPIOA, GPIO_Pin_1, GPIO_MODE_ANALOG, GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_10MHz, false);
	GPIO_InitSimple(GPIOA, GPIO_Pin_2, GPIO_MODE_ANALOG, GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_10MHz, false);

	ADC_GetCalibrationFactor(ADC1);
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);     
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
	ADC_StartOfConversion(ADC1);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

bool ADC_ReadValueLPF(uint16_t *pBuff, uint16_t ChNum)
{
	uint16_t Buffer[ADC_CHANNEL_NUM];
	float SmoothData;
	float LPF_Beta=0.08;
	
	if(ChNum>ADC_CHANNEL_NUM)
		return false;
		
	memcpy(Buffer, (const uint16_t*)ADC_Buffer, sizeof Buffer);
	for(uint16_t i=0;i<ChNum;i++)
	{
		SmoothData = SmoothData - (LPF_Beta * (SmoothData - ADC_Buffer[i]));
		// second order: SmoothData2 = SmoothData2 - (LPF_Beta * (SmoothData2 - SmoothData));
		pBuff[i]=(uint16_t)SmoothData;
	}
	return true;
}


