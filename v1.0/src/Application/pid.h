#ifndef __PID_H
#define __PID_H
#include "common use.h"

typedef int32_t PID_Value_t;

typedef enum{
	PID_Positional,
	PID_Incremental,
} ePID_Mode_t;

typedef struct {
	ePID_Mode_t Mode;
	// Target system output unit eg. motor rps or position
	PID_Value_t ExpectValue;
	PID_Value_t RealValue;
	// Target system input unit eg. motor pwm duty cycle
	PID_Value_t ThrottleCalc;
	PID_Value_t ThrottleMin, ThrottleMax;
	// Parameter
	float Kp;
    float Ki;
	float Kd;
	bool Enable;
	bool Invert;
	// Inside
	PID_Value_t Error, ErrorLast;
	PID_Value_t ErrorIntegral;
	PID_Value_t ErrorDerivative, ErrorDerivativeLast;
	uint32_t SampleTime;
} sPID_t, *spPID_t;

spPID_t PID_Init(spPID_t pPID, PID_Value_t ThrottleMin, PID_Value_t ThrottleMax, ePID_Mode_t Mode);
PID_Value_t PID_Process(spPID_t pPID, PID_Value_t ExpectValue, PID_Value_t RealValue);
void PID_Tunning(spPID_t pPID, float Kp, float Ki, float Kd);

#endif
