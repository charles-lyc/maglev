#include "motor.h"

Motor::Motor(void)
{
	_ready = false;
}

Motor::Motor(
			 GPIO_TypeDef *gpio_1,
			 uint16_t pin_1,
				GPIO_TypeDef *gpio_2,
			 uint16_t pin_2,
			 TIM_HandleTypeDef *htim,
			 uint32_t channel)
{
	_ready = false;

	_timer = htim;
	_channel = channel;
	_gpio[0] = gpio_1;
	_pin[0] = pin_1;
	_gpio[1] = gpio_2;
	_pin[1] = pin_2;

	__HAL_TIM_SET_COMPARE(_timer, _channel, 0);
	HAL_TIM_PWM_Start(_timer, _channel);

	_ready = true;
}

void Motor::write(float duty)
{
	if (!_ready)
		return;

	float ccr;

	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(_timer);

	if (duty > 0)
	{
		HAL_GPIO_WritePin(_gpio[0], _pin[0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_gpio[1], _pin[1], GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(_gpio[0], _pin[0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(_gpio[1], _pin[1], GPIO_PIN_RESET);
	}

	if (fabsf(duty) > 1)
		duty = duty > 0 ? 1 : -1;
	
	if (duty > 0)
		ccr = duty * arr;
	else
		ccr = -duty * arr;

	__HAL_TIM_SET_COMPARE(_timer, _channel, ccr);
}
