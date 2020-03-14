#include "common use.h"
#include "pid.h"

spPID_t PID_Init(spPID_t pPID, PID_Value_t ThrottleMin, PID_Value_t ThrottleMax, ePID_Mode_t Mode)
{
	pPID->Mode=Mode;
	pPID->ExpectValue=0;
	pPID->RealValue=0;
	pPID->ThrottleCalc=0;
	pPID->ThrottleMin=ThrottleMin;
	pPID->ThrottleMax=ThrottleMax;
	pPID->Kp=DEFAULT_Kp;
	pPID->Ki=DEFAULT_Ki;
	pPID->Kd=DEFAULT_Kd;
	pPID->Enable=false;
	pPID->Error=pPID->ErrorLast=pPID->ErrorIntegral=pPID->ErrorDerivative=pPID->ErrorDerivativeLast=0;
	pPID->SampleTime=1000/DEFAULT_SAMPLE_RATE_HZ;
	return pPID;
}
	
PID_Value_t PID_Process(spPID_t pPID, PID_Value_t ExpectValue, PID_Value_t RealValue)
{
	pPID->ExpectValue=ExpectValue;
	pPID->RealValue=RealValue;
	
	pPID->Error=ExpectValue-RealValue;
	pPID->ErrorIntegral+=pPID->Error;
	if(pPID->ErrorIntegral<-INTERGRAL_CONSTAIN)
		pPID->ErrorIntegral=-INTERGRAL_CONSTAIN;
	else if(pPID->ErrorIntegral>INTERGRAL_CONSTAIN)
		pPID->ErrorIntegral=INTERGRAL_CONSTAIN;
	pPID->ErrorDerivative=SYS_DivRound((pPID->Error-pPID->ErrorLast)*1000, pPID->SampleTime);

	if(pPID->Mode==PID_Positional)
	{
		pPID->ThrottleCalc = PID_OUTPUT_FACTOR*(
			pPID->Kp * pPID->Error +
			pPID->Ki * pPID->ErrorIntegral +
			pPID->Kd * pPID->ErrorDerivative);
	}
	else if(pPID->Mode==PID_Incremental)	// just the derivative of Positional
	{
		pPID->ThrottleCalc = PID_OUTPUT_FACTOR*(
			pPID->Kp * (pPID->Error-pPID->ErrorLast) +
			pPID->Ki * (pPID->Error) +
			pPID->Kd * (pPID->ErrorDerivative-pPID->ErrorDerivativeLast));
	}
	
	if(pPID->ThrottleCalc<pPID->ThrottleMin)
		pPID->ThrottleCalc=pPID->ThrottleMin;
	if(pPID->ThrottleCalc>pPID->ThrottleMax)
		pPID->ThrottleCalc=pPID->ThrottleMax;

	pPID->ErrorLast=pPID->Error;
	pPID->ErrorDerivativeLast=pPID->ErrorDerivative;
	
	return pPID->ThrottleCalc;
}
	
void PID_Tunning(spPID_t pPID, float Kp, float Ki, float Kd)
{
	pPID->Kp=Kp;
	pPID->Ki=Ki;
	pPID->Kd=Kd;
	
	pPID->Error=pPID->ErrorLast=
	pPID->ErrorIntegral=
	pPID->ErrorDerivative=pPID->ErrorDerivativeLast=0;
}

void PID_TunningAuto(spPID_t pPID)
{

}

