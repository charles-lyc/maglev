#include "gpio.h"

static uint16_t GPIO_GetPinSource(uint16_t GPIO_Pin)
{
	uint16_t pinnum=0;
	uint16_t i=15;
	
	do{
		if(GPIO_Pin&1)
			return pinnum;
		GPIO_Pin>>=1;
		pinnum++;
	} while(i-->0);
	return (uint16_t)-1;
}

void GPIO_InitSimple(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
					uint8_t GPIO_Mode,
					GPIOOType_TypeDef GPIO_OType, 
					GPIOPuPd_TypeDef GPIO_PuPd, 
					GPIOSpeed_TypeDef GPIO_Speed,
					bool bitset)
{
	uint16_t pinnum = GPIO_GetPinSource(GPIO_Pin);
	uint16_t portnum = ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));
	
	RCC->AHBENR |= (1UL<<portnum);
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	if(GPIO_Mode == GPIO_MODE_OUTPUT){
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	}
	else if(GPIO_Mode == GPIO_MODE_INPUT){
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	}
	else if(GPIO_Mode == GPIO_MODE_ANALOG){
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
	}
	else{
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
		GPIO_PinAFConfig(GPIOx,pinnum,GPIO_Mode);
	}
	GPIO_InitStructure.GPIO_OType = GPIO_OType;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	if(bitset)
		// GPIOx->BSRRL |= (1UL<<pinnum);
		GPIO_SetBits(GPIOx,GPIO_Pin);
	else	
		GPIO_ResetBits(GPIOx,GPIO_Pin);
}
	
