#ifndef SYSTICK_H
#define SYSTICK_H
#include "common use.h"

void SysTick_Init(void);
uint32_t SysTick_GetTick(void);
void SysTick_CBRegister(void (*Func)(void));

#endif
