/**
 * STM32 HAL Mock Layer
 * Provides minimal type definitions and function stubs to compile and run
 * the maglev application logic on a Linux host for simulation/testing.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- GPIO Mock ---------- */
typedef struct {
    uint32_t dummy;
} GPIO_TypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_3  ((uint16_t)0x0008)
#define GPIO_PIN_4  ((uint16_t)0x0010)
#define GPIO_PIN_5  ((uint16_t)0x0020)

static GPIO_TypeDef gpioa_mock, gpiob_mock, gpiof_mock;
#define GPIOA (&gpioa_mock)
#define GPIOB (&gpiob_mock)
#define GPIOF (&gpiof_mock)

static inline void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    (void)GPIOx; (void)GPIO_Pin; (void)PinState;
}

/* Simulated button state: default not pressed (SET = not pressed) */
static GPIO_PinState mock_button_state = GPIO_PIN_SET;

static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    (void)GPIOx; (void)GPIO_Pin;
    return mock_button_state;
}

/* ---------- Timer Mock ---------- */
typedef struct {
    uint32_t AutoReload;
    uint32_t Compare[4]; /* channels 1-4 */
} TIM_Instance_Mock;

typedef struct {
    TIM_Instance_Mock *Instance;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1  0
#define TIM_CHANNEL_2  1
#define TIM_CHANNEL_3  2
#define TIM_CHANNEL_4  3

#define __HAL_TIM_SET_AUTORELOAD(htim, val) ((htim)->Instance->AutoReload = (val))
#define __HAL_TIM_GET_AUTORELOAD(htim)      ((htim)->Instance->AutoReload)
#define __HAL_TIM_SET_COMPARE(htim, ch, val) ((htim)->Instance->Compare[ch] = (val))
#define __HAL_TIM_GET_COMPARE(htim, ch)      ((htim)->Instance->Compare[ch])

typedef int HAL_StatusTypeDef;
#define HAL_OK 0

static inline HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel) {
    (void)htim; (void)Channel; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel) {
    (void)htim; (void)Channel; return HAL_OK;
}

/* ---------- ADC / DMA Mock ---------- */
typedef struct { uint32_t dummy; } ADC_HandleTypeDef;
typedef struct { uint32_t dummy; } DMA_HandleTypeDef;
typedef struct { uint32_t dummy; } I2C_HandleTypeDef;

static inline HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length) {
    (void)hadc; (void)pData; (void)Length; return HAL_OK;
}

/* ---------- Delay Mock ---------- */
static inline void HAL_Delay(uint32_t Delay) {
    (void)Delay;
    /* In simulation we don't actually delay */
}

/* ---------- HAL_Init and Error_Handler ---------- */
static inline HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
static inline void Error_Handler(void) {}

#ifdef __cplusplus
}
#endif
