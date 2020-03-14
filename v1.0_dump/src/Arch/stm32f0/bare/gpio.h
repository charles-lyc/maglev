#ifndef GPIO_H
#define GPIO_H
#include "common use.h"

#define GPIO_MODE_INPUT		16
#define GPIO_MODE_OUTPUT	17
#define GPIO_MODE_ANALOG	18

typedef struct {
	GPIO_TypeDef *Port;
	uint16_t Pin;	
} sGPIO_t;

// Only support one by one pin!
// eg.GPIO_InitSimple(GPIOA,GPIO_Pin_9,GPIO_MODE_OUTPUT,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIO_Speed_25MHz,false);
// GPIO_InitSimple(GPIOA,GPIO_Pin_9, GPIO_AF_USART1,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIO_Speed_50MHz,true);
void GPIO_InitSimple(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
					uint8_t GPIO_Mode,
					GPIOOType_TypeDef GPIO_OType, 
					GPIOPuPd_TypeDef GPIO_PuPd, 
					GPIOSpeed_TypeDef GPIO_Speed,
					bool bitset);

#endif

