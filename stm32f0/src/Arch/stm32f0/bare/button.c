#include "gpio.h"
#include "systick.h"
#include "button.h"
#include "configure.h"
#include "common use.h"

uint64_t BTN_BitIndicate=0x0000000000000000;
eButton_Msg_t BTNMsgQueue;

typedef struct {
	eButton_Msg_t Msg;
	sGPIO_t IO;
} sButton_t;

#define BTN1_GPIO_PORT      		GPIOA
#define BTN1_GPIO_PIN       		GPIO_Pin_15
#define BTN2_GPIO_PORT      		GPIOB
#define BTN2_GPIO_PIN       		GPIO_Pin_6
#define BTN3_GPIO_PORT      		GPIOB
#define BTN3_GPIO_PIN       		GPIO_Pin_7
sButton_t BTN_GPIO_List[]=
{
	{BTN_1, BTN1_GPIO_PORT, BTN1_GPIO_PIN},
	{BTN_2,	BTN2_GPIO_PORT, BTN2_GPIO_PIN},
	{BTN_3,	BTN3_GPIO_PORT, BTN3_GPIO_PIN},
};

/* static status */
typedef struct {
	bool Level;
	enum{
		eNone,
		eRising,
		eFalling,
	}Edge;	// 1:rise, 2:fall
	bool LongPressed;
	uint32_t FirstPressTime;
	// uint32_t PulseCnt;
} sBTN_Status_t;
static sBTN_Status_t BTN_Status[SIZEOF_ELE(BTN_GPIO_List)];

void Button_Init(void)
{
	for(uint16_t i=0;i<SIZEOF_ELE(BTN_GPIO_List);i++)
	{
		GPIO_InitSimple(BTN_GPIO_List[i].IO.Port, BTN_GPIO_List[i].IO.Pin, GPIO_MODE_INPUT, GPIO_OType_OD, GPIO_PuPd_DOWN, GPIO_Speed_2MHz,true);
		
		BTN_Status[i].Level=true;	// default button-up, high electric level
		BTN_Status[i].Edge=eNone;
		BTN_Status[i].LongPressed=false;
		BTN_Status[i].FirstPressTime=SysTick_GetTick();
	}
}
	
void Button_Handle(void)
{
	static uint32_t Tick;	
	uint32_t PressTime;
	eButton_Msg_t Msg;
	
	Msg=BTN_NONE;
	Tick = SysTick_GetTick();
	
	for(uint16_t i=0;i<SIZEOF_ELE(BTN_GPIO_List);i++)
	{
		if(Tick > BTN_Status[i].FirstPressTime)
			PressTime = Tick - BTN_Status[i].FirstPressTime;
		else
			PressTime = UINT32_MAX - BTN_Status[i].FirstPressTime + Tick+1;

		if(!GPIO_ReadInputDataBit(BTN_GPIO_List[i].IO.Port, BTN_GPIO_List[i].IO.Pin))
		{
			// Up
			if(BTN_Status[i].Level==false)
			{
				Msg=(eButton_Msg_t)(BTN_GPIO_List[i].Msg+3);
				BTNMsgQueue=Msg;
				if(!BTN_Status[i].LongPressed)
				{
					Msg=(eButton_Msg_t)(BTN_GPIO_List[i].Msg+1);
					BTNMsgQueue=Msg;
				}
			}
			BTN_Status[i].Level=true;
			BTN_Status[i].FirstPressTime=Tick;
			BTN_Status[i].LongPressed=false;
		}
		else
		{
			// down
			if(BTN_Status[i].Level==true)
			{
				Msg=BTN_GPIO_List[i].Msg;
				BTNMsgQueue=Msg;
			}

			// Long
			if(BTN_Status[i].LongPressed==false && PressTime>BTN_LONG_PRESS_TIME)
			{
				BTN_Status[i].LongPressed=true;
				Msg=(eButton_Msg_t)(BTN_GPIO_List[i].Msg+2);
				BTNMsgQueue=Msg;
			}
			BTN_Status[i].Level=false;
		}
	}
}

eButton_Msg_t Button_GetValue(void)
{
	eButton_Msg_t Msg;
	Msg=BTNMsgQueue;
	BTNMsgQueue=BTN_NONE;
	return Msg;
}
