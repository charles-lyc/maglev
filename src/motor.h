#pragma once

#include "types.h"
#include "main.h"

class Motor
{
private:
	GPIO_TypeDef *_gpio[2];
	uint16_t _pin[2];
	TIM_HandleTypeDef *_timer;
	uint32_t _channel;

	bool _ready;

public:
	Motor();
	Motor(
			 GPIO_TypeDef *gpio_1,
			 uint16_t pin_1,
				GPIO_TypeDef *gpio_2,
			 uint16_t pin_2,
			 TIM_HandleTypeDef *htim,
			 uint32_t channel);

	void write(float duty);
};
