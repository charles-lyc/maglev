/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "utils_ex.h"
#include "pid.h"
#include "lpf_2p.h"

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

volatile uint16_t ADC_Raw[3];
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
pid_param_t PID_Param_Pos[2],PID_Param_Speed[2];
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
	MODE_STARTUP = 0,
	MODE_IDLE,
	MODE_CALIBRATE,
	MODE_RUN,
	MODE_TEST,
} sys_status_t;
sys_status_t System_Status = MODE_IDLE;

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

// when get rid of mag, the coil still working, then detection will fail.
// so the threthhold should be larger a little bit.
int8_t mag_idle_detect(void)
{
	if (Hall_Pos_lpf[1] < -200)
		return -1;
	else if (Hall_Pos_lpf[1] > 100)
		return 1;

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
		if(i==0)		
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

	HAL_Delay(500);

	for (size_t i = 0; i < 2; i++)
	{
		offset_sum = 0;
		for (size_t j = 0; j < 32; j++)
		{
			offset_sum += ADC_Raw[i];
			HAL_Delay(1);
		}
		ADC_Offset[i] = utils_round_div(offset_sum, 32);
	}

	HAL_Delay(100);
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
	// mag_set_pwm(-0, v_set[0]);
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	__HAL_TIM_SET_AUTORELOAD(&htim3, 1000);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 200 - 1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADC_Raw, 3);
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
	
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
			// 	if (mag_btn_even())
			// 	{
			// 		System_Status = MODE_CALIBRATE;
			// 	}
			// 	if (System_Status == MODE_CALIBRATE)
			// 	{
			// 		mag_position_calibrate();
			// 		// save parameter
			// 		System_Status = MODE_IDLE;
			// 	}
			// }

			// 	if (System_Status == MODE_RUN || System_Status == MODE_IDLE)
			// 	{
			// 		int8_t detect;

			// 		detect = mag_idle_detect();
			// 		detect = -1;

			// 		if (detect == 0)
			// 		{
			// 			mag_drv_set(false);
			// 			System_Status = MODE_IDLE;
			// 			LED_Status = LED_IDLE;
			// 		}
			// 		else if (detect == 1)
			// 		{
			// 			mag_drv_set(false);
			// 			System_Status = MODE_IDLE;
			// 			LED_Status = LED_ERROR;
			// 		}
			// 		else if (detect == -1)
			// 		{
			// 			mag_drv_set(true);
			// 			System_Status = MODE_RUN;
			// 			// LED_Status = LED_RUN;
			// 			LED_Status = LED_IDLE;
			// 		}
			// 	}
		}

	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks 
  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks 
  */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC4;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted. 
  */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted. 
  */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted. 
  */
	sConfig.Channel = ADC_CHANNEL_2;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00300208;
	hi2c1.Init.OwnAddress1 = 176;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter 
  */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter 
  */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 48 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 100 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 3 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pins : PF0 PF1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : PA3 PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM14)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
