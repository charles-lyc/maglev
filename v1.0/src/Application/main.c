#include "common use.h"
#include "serial.h"
#include "button.h"
#include "motor.h"
#include "gpio.h"
#include "adc.h"
#include "systick.h"
#include "pid.h"

static sPID_t Controler[2];
spPID_t pController[2];
PID_Value_t Position[2], ExpectPostion[2];
PID_Value_t Throttle[2];

void MotorController(void)
{
	uint16_t Buffer[ADC_CHANNEL_NUM];

	// 500hz
	static int Cnt=0;
	Cnt++;
	if(Cnt==2) Cnt=0;
	else return;

	ADC_ReadValueLPF(Buffer, ADC_CHANNEL_NUM);
	printf("$%d %d %d;", Buffer[0], Buffer[1], Buffer[2]);
	
	for(uint16_t i=0; i<2; i++)
	{
		Position[i]=Buffer[0];
		Throttle[i]=PID_Process(pController[i], ExpectPostion[i], Position[i]);
		Motor_SetThrottle(Throttle[i]);
	}
}

void MsgProcess()
{
	eButton_Msg_t Msg;
	
	Msg=Button_GetValue();
	if(Msg==BTN_1_SHORT)
	{
		
	}
}

int main(void)
{
	static uint32_t ReportTick, DeviceTick;

	Serial_Init(SerialPort_Debug, 256000);
	Button_Init();
	SysTick_Init();
	GPIO_InitSimple(GPIOA, GPIO_Pin_11, GPIO_MODE_OUTPUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_2MHz, 0);
	Motor_Init();
	ADC_DMA_Init();
	for(uint16_t i=0; i<2; i++)
	{
		pController[i]=&Controler[i];
		PID_Init(pController[i], -1000, 1000, PID_Positional);
	}
	SysTick_CBRegister(MotorController);
	printf("Hello world.\n");
	
	PID_Tunning(pController[0], 0.3, 0.01, 0.01);
	PID_Tunning(pController[1], 0.3, 0.01, 0.01);
	
	while(1)
	{
		// device
		if(DeviceTick+40<SysTick_GetTick())
		{
			DeviceTick=SysTick_GetTick();
			Button_Handle();
			MsgProcess();
		}
		// Plot
		if(ReportTick+5<SysTick_GetTick())
		{
			ReportTick=SysTick_GetTick();
			//printf("$%d %d;", Position[0], Position[1]);
		}
	}
}

