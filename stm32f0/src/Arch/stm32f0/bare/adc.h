#ifndef ADC_H
#define ADC_H
#include "common use.h"

void ADC_DMA_Init(void);
bool ADC_ReadValueLPF(uint16_t *pBuff, uint16_t ChNum);

#endif
