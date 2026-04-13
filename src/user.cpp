#include "user.h"
#include "maglev.h"

static uint32_t btn_cnt = 0;
volatile uint16_t adc_raw[3];
Maglev maglev;
void drv_en(bool en);
void motor_write(float a, float b);
bool mag_btn_even(void);

void user_loop(void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim1, 1000 - 1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_raw, 3);

	__HAL_TIM_SET_AUTORELOAD(&htim3, 1000);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	maglev.init((uint16_t *)adc_raw,
				true,
				false,
				drv_en,
				motor_write);

	maglev.calibrate();

	while (1)
	{
		if (mag_btn_even())
			maglev.calibrate();

		maglev.main_loop();

		HAL_Delay(1);
	}
}

void drv_en(bool en)
{
	if (en)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}

void motor_write(float pwm_a, float pwm_b)
{
	uint16_t ccr1, ccr2, arr;

	float abs_a = fabsf(pwm_a);
	float abs_b = fabsf(pwm_b);

	arr = __HAL_TIM_GET_AUTORELOAD(&htim3) - 1;

	if (abs_a > arr)
		abs_a = arr;
	if (abs_b > arr)
		abs_b = arr;

	if (pwm_a > 0)
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
	}
	ccr2 = (uint16_t)abs_a;

	if (pwm_b > 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	ccr1 = (uint16_t)abs_b;

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	maglev.adc_sample_callback();
}

bool mag_btn_even(void)
{
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
	{
		if (btn_cnt++ == 100)
		{
			return true;
		}
	}
	else
		btn_cnt = 0;

	return false;
}
