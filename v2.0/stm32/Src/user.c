#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "utils_ex.h"
#include "pid.h"
#include "lpf_2p.h"
#include "user.h"

enum
{
	LED_STARTUP = 0,
	LED_IDLE,
	LED_RUN,
	LED_ERROR,
	LED_TEST,
} LED_Status;
#define I2C_DEV_ADDR (0x58 << 1)
#define I2C_DEV_ID (0x10)
#define AVG_WIN_SZ 256

extern volatile uint16_t ADC_Raw[3];
int32_t ADC_RawAvg[2];
int32_t ADC_Offset[3] = {3160, 3000, 3400};
float Hall_Pos[3], Hall_Pos_Last[3];
float Hall_Pos_lpf[2];
float Hall_Speed_lpf[2];
float TargetSpeed[2];
lpf_2p_t lpf2p_pos[2], lpf2p_speed[2];
int32_t avg_pos[2][AVG_WIN_SZ];
int32_t avg_indx[2];
pid_t PID_Pos[2];
pid_t PID_Speed[2];
pid_param_t PID_Param_Pos[2], PID_Param_Speed[2];
uint8_t LED_Brightness[16];
ADC_HandleTypeDef hadc1;
static uint32_t CtrlTick;
static uint32_t BackgroundTick;
const uint8_t LED_AddrMap[16] = {
	8 + 7,
	8 + 6,
	8 + 5,
	8 + 4,
	8 + 3,
	8 + 2,
	8 + 1,
	8 + 0,
	7,
	6,
	5,
	4,
	3,
	2,
	1,
	0,
};

inline void mag_set_pwm(int32_t pwm_a, int32_t pwm_b);
int8_t mag_idle_detect(void);
void mag_drv_set(bool mode);
void mag_hall_calibrate(void);
void mag_crtl_speed(void);
void mag_crtl_pos(void);
void led_process(void);
void mag_led_reflesh(void);

typedef enum
{
	MODE_STARTUP = 0,
	MODE_IDLE,
	MODE_CALIBRATE,
	MODE_RUN,
	MODE_TEST,
} sys_status_t;
sys_status_t System_Status = MODE_IDLE;

// when get rid of mag, the coil still working, then detection will fail.
// so the threthhold should be larger a little bit.
int8_t mag_idle_detect(void)
{
	if (Hall_Pos_lpf[1] < -200)
		return 1;
	else if (Hall_Pos_lpf[1] > 100)
		return -1;

	return 0;
}

void mag_set_pwm(int32_t pwm_a, int32_t pwm_b)
{
	uint16_t ccr1, ccr2, arr;

	if (pwm_a > 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		ccr1 = pwm_a;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		ccr1 = -pwm_a;
	}
	if (pwm_b > 0)
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		ccr2 = pwm_b;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
		ccr2 = -pwm_b;
	}

	arr = __HAL_TIM_GET_AUTORELOAD(&htim3) - 1;
	if (ccr1 > arr)
		ccr1 = arr;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr1);
	if (ccr2 > arr)
		ccr2 = arr;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr2);
}

// 1K (Flat Response 23kHz)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	float Hall_Pos_lpf2p_Temp[2];

	CtrlTick++;

	for (size_t i = 0; i < 2; i++)
	{
		// pos lpf
		// UTILS_LP_FAST(Hall_Pos_lpf[i], Hall_Pos[i], 0.1);
		// Hall_Pos_lpf2p_Temp[i] = lpf2_apply(&lpf2p_pos[i], Hall_Pos[i]);

		// pos average
		avg_pos[i][avg_indx[i]++] = ADC_Raw[i];
		avg_indx[i] %= AVG_WIN_SZ;
		int32_t temp = 0;
		for (size_t j = 0; j < AVG_WIN_SZ; j++)
			temp += avg_pos[i][j];
		ADC_RawAvg[i] = temp / AVG_WIN_SZ;

		// assemble problem
		if (i == 0)
			Hall_Pos[i] = -((int32_t)ADC_RawAvg[i] - ADC_Offset[i]);
		else
			Hall_Pos[i] = (int32_t)ADC_RawAvg[i] - ADC_Offset[i];

		// UTILS_LP_FAST(Hall_Speed_lpf[i], Hall_Pos_lpf2p_Temp[i] - Hall_Pos_lpf[i], 0.01);
		Hall_Speed_lpf[i] = lpf2_apply(&lpf2p_speed[i], Hall_Pos[i] - Hall_Pos_Last[i]);
		Hall_Pos_Last[i] = Hall_Pos[i];
	}

	if (System_Status == MODE_RUN)
	{
		mag_crtl_speed();
	}
}

void mag_position_calibrate(void)
{
	int32_t offset_sum;

	HAL_Delay(100);

	for (size_t i = 0; i < 2; i++)
	{
		offset_sum = 0;
		ADC_Offset[i] = 0;
		HAL_Delay(2);

		for (size_t j = 0; j < 32; j++)
		{
			HAL_Delay(2);
			offset_sum += ADC_Raw[i];
		}
		ADC_Offset[i] = utils_round_div(offset_sum, 32);
	}
}

void mag_crtl_pos(void)
{
	float error;

	for (size_t i = 0; i < 2; i++)
	{
		error = 0 - Hall_Pos[i];
		TargetSpeed[i] = pid_regulator(error, &PID_Pos[i]);
	}
}

void mag_crtl_speed(void)
{
	float v_set[2];
	float error;

	for (size_t i = 0; i < 2; i++)
	{
		error = TargetSpeed[i] - Hall_Speed_lpf[i];
		v_set[i] = pid_regulator(error, &PID_Speed[i]);
	}

	mag_set_pwm(-v_set[1], v_set[0]);
}

void user_loop(void)
{
	mag_drv_set(false);

	System_Status = MODE_CALIBRATE;

	PID_Param_Speed[0].kp = 4;
	PID_Param_Speed[0].ki = 0;
	PID_Param_Speed[0].kd = 0.01;
	PID_Param_Speed[0].limit_integral = 1;
	PID_Param_Speed[0].factor = 100;
	PID_Param_Speed[0].limit_output = 700;
	pid_init(&PID_Speed[0], &PID_Param_Speed[0]);

	PID_Param_Speed[1].kp = 4;
	PID_Param_Speed[1].ki = 0;
	PID_Param_Speed[1].kd = 0.01;
	PID_Param_Speed[1].limit_integral = 1;
	PID_Param_Speed[1].factor = 100;
	PID_Param_Speed[1].limit_output = 700;
	pid_init(&PID_Speed[1], &PID_Param_Speed[1]);

	PID_Param_Pos[0].kp = 0;
	PID_Param_Pos[0].ki = 0;
	PID_Param_Pos[0].kd = 0;
	PID_Param_Pos[0].limit_integral = 1;
	PID_Param_Pos[0].factor = 0.001;
	PID_Param_Pos[0].limit_output = 1;
	pid_init(&PID_Pos[0], &PID_Param_Pos[0]);

	PID_Param_Pos[1].kp = 0;
	PID_Param_Pos[1].ki = 0;
	PID_Param_Pos[1].kd = 0;
	PID_Param_Pos[1].limit_integral = 1;
	PID_Param_Pos[1].factor = 0.001;
	PID_Param_Pos[1].limit_output = 1;
	pid_init(&PID_Pos[1], &PID_Param_Pos[1]);

	lpf2_set_cutoff_frequency(&lpf2p_pos[0], 1000, 80);
	lpf2_set_cutoff_frequency(&lpf2p_pos[1], 1000, 80);
	lpf2_set_cutoff_frequency(&lpf2p_speed[0], 5000, 100);
	lpf2_set_cutoff_frequency(&lpf2p_speed[1], 5000, 100);

	System_Status = MODE_STARTUP;

	uint8_t i2c_buffer_tx[5], i2c_buffer_rx[5];
	bsp_aw9523b_reg_read(I2C_DEV_ID, i2c_buffer_rx, 1);
	if (i2c_buffer_rx[0] != 0x23)
		HAL_Delay(1000);
	// all pull high
	i2c_buffer_tx[0] = 0xFF;
	bsp_aw9523b_reg_write(0x02, i2c_buffer_tx, 1);
	bsp_aw9523b_reg_write(0x03, i2c_buffer_tx, 1);
	// led mode
	i2c_buffer_tx[0] = 0;
	bsp_aw9523b_reg_write(0x12, i2c_buffer_tx, 1);
	bsp_aw9523b_reg_write(0x13, i2c_buffer_tx, 1);
	// brightness
	for (size_t i = 0; i < 16; i++)
		LED_Brightness[i] = 0;
	mag_led_reflesh();
	// P0: push pull; max current: Imax x 1/4
	i2c_buffer_tx[0] = (0x01 << 4) | (0x03);
	bsp_aw9523b_reg_write(0x11, i2c_buffer_tx, 1);

	while (1)
	{
		switch (System_Status)
		{
		case MODE_STARTUP:
			LED_Status = LED_STARTUP;
			System_Status = MODE_RUN;
			mag_drv_set(true);
			break;
		default:
			break;
		}

		// 10 ms tasks
		if (HAL_GetTick() > BackgroundTick + 10)
		{
			BackgroundTick = HAL_GetTick();
			mag_crtl_pos();

			// 	led_process();
			// 	mag_led_reflesh();

			if (mag_btn_even())
			{
				System_Status = MODE_CALIBRATE;
				mag_position_calibrate();

				// save parameter

				System_Status = MODE_IDLE;
			}

			if (System_Status == MODE_RUN || System_Status == MODE_IDLE)
			{
				int8_t detect;

				detect = mag_idle_detect();

				if (detect == 1)
				{
					mag_drv_set(true);
					System_Status = MODE_RUN;
					LED_Status = LED_RUN;
					// LED_Status = LED_IDLE;
				}
				else
				{
					mag_drv_set(false);
					System_Status = MODE_RUN;
					LED_Status = LED_RUN;
				}
			}
		}
	}
}

// 1khz
void led_process(void)
{
	static uint32_t Tick;
	Tick++;

	switch (LED_Status)
	{
	case LED_RUN:
	{
		if (Tick % 20 == 0)
		{
			static bool LED_Invert;
			static uint32_t Brightness;

			Brightness++;
			if (Brightness > 50)
			{
				LED_Invert = !LED_Invert;
				Brightness = 0;
			}
			for (size_t i = 0; i < 16; i++)
			{
				if (LED_Invert)
					LED_Brightness[i] = 50 - Brightness;
				else
					LED_Brightness[i] = Brightness;
			}
		}
		break;
	}
		{
			static uint8_t LED_Head = 0;
			static uint8_t LED_Origin = 0;
			static bool LED_Invert = false;
			static bool LED_Carry = false;

			if (Tick % 100 == 0)
			{
				LED_Head++;
				if (LED_Head > 15)
					LED_Head = 0;

				if (LED_Head == LED_Origin && LED_Carry)
				{
					LED_Carry = false;
				}
				else if (LED_Head == LED_Origin && LED_Invert)
				{
					LED_Origin++;
					if (LED_Origin > 15)
						LED_Origin = 0;
					LED_Carry = true;
					LED_Invert = false;
				}
				else if (LED_Head == LED_Origin && !LED_Invert)
				{
					LED_Invert = true;
				}

				LED_Brightness[LED_Head] = LED_Invert ? 0 : 5;
			}
			break;
		}
	case LED_IDLE:
		if (Tick % 100 == 0)
		{
			for (size_t i = 0; i < 16; i++)
				LED_Brightness[i] = 0;
		}
		break;
	case LED_TEST:
#if 0
	{
		if (Tick % 100 == 0)
		{
			static uint8_t LED_Tail = 0;

			LED_Tail++;
			if (LED_Tail > 15)
				LED_Tail = 0;
			for (size_t i = 0; i < 16; i++)
				LED_Brightness[i] = 0;
			LED_Brightness[LED_Tail] = 1;
			LED_Brightness[(LED_Tail + 1) % 16] = 2;
			LED_Brightness[(LED_Tail + 2) % 16] = 5;
			LED_Brightness[(LED_Tail + 3) % 16] = 10;
		}
		break;
	}
	{
		static bool LED_Invert;
		if (Tick % 200 == 0)
		{
			srand(TIM3->CNT);
			LED_Brightness[rand() % 16] =LED_Invert? 0: 5;
		}
		if(LED_Invert)
		{
			LED_Invert = false;
			for (size_t i = 0; i < 16; i++)
				if(LED_Brightness[i] == 5)
					LED_Invert = true;			
		}
		else
		{
			LED_Invert = true;			
			for (size_t i = 0; i < 16; i++)
				if(LED_Brightness[i] != 5)
					LED_Invert = false;
		}
		break;
	}
#endif
	default:
		break;
	}

	// mag_led_reflesh();
}

void bsp_aw9523b_reg_read(uint8_t reg, uint8_t *buffer, uint8_t size)
{
	HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, &reg, 1, 1);
	HAL_I2C_Master_Receive(&hi2c1, I2C_DEV_ADDR | 0x01, buffer, size, 2);
}

void bsp_aw9523b_reg_write(uint8_t reg, uint8_t *buffer, uint8_t size)
{
	uint8_t buff[20];

	buff[0] = reg;
	memcpy(buff + 1, buffer, size);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, buff, size + 1, 2);
}

void mag_led_reflesh(void)
{
	bsp_aw9523b_reg_write(0x20, LED_Brightness, 16);
}

bool mag_btn_even(void)
{
	static uint32_t btn_cnt = 0;

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
	{
		if (btn_cnt++ > 50)
		{
			btn_cnt = 0;
			return true;
		}
	}
	else
		btn_cnt = 0;

	return false;
}

void mag_drv_set(bool mode)
{
	if (mode)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}
