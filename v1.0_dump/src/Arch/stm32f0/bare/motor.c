#include "gpio.h"
#include "common use.h"
#include "systick.h"
#include "configure.h"

#define MOTOR_CNT_FREQ			1000000	// 1us
#define MOTOR_OUTPUT_RATE_HZ	1000	// 1ms

#define COIL_A1_PORT    		GPIOA
#define COIL_A1_PIN     		GPIO_Pin_4
#define COIL_A2_PORT    		GPIOA
#define COIL_A2_PIN     		GPIO_Pin_3
#define COIL_B1_PORT    		GPIOA
#define COIL_B1_PIN     		GPIO_Pin_6
#define COIL_B2_PORT    		GPIOA
#define COIL_B2_PIN     		GPIO_Pin_7
#define COIL_POWER_PORT			GPIOA
#define COIL_POWER_Pin 			GPIO_Pin_5
#define COIL_PWMA_PORT    		GPIOB
#define COIL_PWMA_Pin     		GPIO_Pin_0
#define COIL_PWMB_PORT			GPIOB
#define COIL_PWMB_Pin 			GPIO_Pin_1

void Motor_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_DeInit(TIM3);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = MOTOR_CNT_FREQ/MOTOR_OUTPUT_RATE_HZ-1;
	TIM_TimeBaseStructure.TIM_Prescaler = (APB1CLK)/MOTOR_CNT_FREQ-1;
	TIM_TimeBaseStructure.TIM_ClockDivision =0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// Channel 3/4, output
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	GPIO_InitSimple(COIL_A1_PORT, COIL_A1_PIN, GPIO_MODE_OUTPUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
	GPIO_InitSimple(COIL_A2_PORT, COIL_A2_PIN, GPIO_MODE_OUTPUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
	GPIO_InitSimple(COIL_B1_PORT, COIL_B1_PIN, GPIO_MODE_OUTPUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
	GPIO_InitSimple(COIL_B2_PORT, COIL_B2_PIN, GPIO_MODE_OUTPUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
	GPIO_InitSimple(COIL_POWER_PORT, COIL_POWER_Pin, GPIO_MODE_OUTPUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
	GPIO_InitSimple(COIL_PWMA_PORT, COIL_PWMA_Pin, GPIO_AF_1, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
	GPIO_InitSimple(COIL_PWMB_PORT, COIL_PWMB_Pin, GPIO_AF_1, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0);
}

void Motor_SetThrottle(int16_t dutyA, int16_t dutyB)
{
	if(dutyA>0){
		GPIO_SetBits(COIL_A1_PORT, COIL_A1_PIN);
		GPIO_ResetBits(COIL_A2_PORT, COIL_A2_PIN);
	}
	else{
		GPIO_ResetBits(COIL_A1_PORT, COIL_A1_PIN);
		GPIO_SetBits(COIL_A2_PORT, COIL_A2_PIN);
	}
	if(dutyB>0){
		GPIO_ResetBits(COIL_B1_PORT, COIL_B1_PIN);
		GPIO_SetBits(COIL_B2_PORT, COIL_B2_PIN);
	}
	else{
		GPIO_SetBits(COIL_B1_PORT, COIL_B1_PIN);
		GPIO_ResetBits(COIL_B2_PORT, COIL_B2_PIN);
	}
	
	dutyA=ABS(dutyA);
	dutyB=ABS(dutyB);
	if(dutyB>1000-1)
		dutyB=1000-1;
	if(dutyB>1000-1)
		dutyB=1000-1;

	TIM_SetCompare3(TIM3, dutyA);
	TIM_SetCompare4(TIM3, dutyB);
}

