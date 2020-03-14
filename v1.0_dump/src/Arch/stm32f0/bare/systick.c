#include "systick.h"

static volatile uint32_t SystemTickMs;
void (*pSysTickCallback)(void)=NULL;

void SysTick_Init(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
	SystemTickMs=0;
}

void SysTick_Handler(void)
{
	SystemTickMs++;
	if(pSysTickCallback)
		pSysTickCallback();
}

uint32_t SysTick_GetTick(void)
{
	return SystemTickMs;
}

void SysTick_CBRegister(void (*Func)(void))
{
	pSysTickCallback=Func;
}


