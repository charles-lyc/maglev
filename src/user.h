#pragma once

#include "main.h"
#include "types.h"

extern DMA_HandleTypeDef hdma_adc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;



#ifdef __cplusplus
extern "C"
{
#endif

	void user_loop(void);

#ifdef __cplusplus
}
#endif