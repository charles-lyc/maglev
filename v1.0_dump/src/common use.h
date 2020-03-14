#ifndef COMMON_INC_H
#define COMMON_INC_H

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"
#include "stddef.h"
#include "string.h"
#include "configure.h"
#include "stm32f0xx.h"

// #define offsetof(TYPE, MEMBER) ((size_t)&((TYPE *)0)->MEMBER)
#define container_of(ptr, type, member) ({\
	const typeof(((type*)0)->member) *__mptr = (ptr);\
	(type*)((char*)__mptr - offsetof(type, member));})
#define DEBUG_BREAK           		__asm__("BKPT #0")	// __ASM ("bkpt 0;")
#define ENABLE_INT()				 __enable_irq()     // __set_PRIMASK(0)
#define DISABLE_INT()				 __disable_irq()    // __set_PRIMASK(1)
#define ABS(x)						((x)>=0 ? (x) : (-(x)))
#define SIZEOF_ELE(ele)				(sizeof(ele) / sizeof(*(ele)))
#define OFFSET_ADDR(type,ele) 		((unsigned int)(&((type *)0)->ele))
#define ROUND(A,B)              	(((A)<0)?(((A)-(B)/2)/B):(((A)+(B)/2)/(B))) 
#define SYSTEM_GETTICK()			SysTick_GetTick()
#define PERIOD_CALL(A,Tick)			if(A+Tick<SYSTEM_GETTICK())
#define PERIOD_RELOAD(A)			A=SystemGetTick()
#define DEBUG_BREAK           		__asm__("BKPT #0")
#define SWAP_BYTE_16(x)				((uint16_t) (((x>>8)&0x00ff)|((x<<8)&0xff00)))
#define OS_WAITE_SEMPHR_READY(x)	while(x==NULL)\
									{\
										vTaskDelay(100);\
									}
#define IS_IN_RANGE(x,a,b)			((x>a)&&(x<b))
#define IS_IN_RANGE_EQUAL(x,a,b)	((x>=a)&&(x<=b))

void Delay_Us(uint32_t TickNb);
void Delay_Ms(uint32_t TickNb);
signed long SYS_DivRound(signed long A,signed long B);
void SYS_EnableIRQs(void);
void SYS_DisableIRQs(void);
uint8_t CheckSum(void *pBuff, uint32_t Length);
unsigned short CRC16CCITT(const void *pBuffer, unsigned long BufferSize);

#endif
