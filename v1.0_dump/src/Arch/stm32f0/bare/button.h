#ifndef BUTTON_H
#define BUTTON_H

#include "common use.h"

// Long press should placed NEXT to the short press
// "Combine" considered to be an indepenent ID
typedef enum{
	BTN_NONE=0,
	
	BTN_1,
	BTN_1_SHORT,
	BTN_1_LONG,
	BTN_1_UP,

	BTN_2,
	BTN_2_SHORT,
	BTN_2_LONG,
	BTN_2_UP,

	BTN_3,
	BTN_3_SHORT,
	BTN_3_LONG,
	BTN_3_UP,
	
	BTN_MSG_NUM,
} eButton_Msg_t;

#if 0
#define BTN_COMBINE_1_IO			(BTN_WHEAL_R || )
#define BTN_COMBINE_IO(a,b,c,d)		(!BTN_Status[BTN_WHEAL_R/2].Level && !BTN_Status[BTN_WHEAL_L/2].Level && !BTN_Status[BTN_WHEAL_L/2].Level)
#define BTN_LONGPRESS_MASK			(1UL<<31)
#define BTN_GET_MSG_ID(msg)			((msg)&BTN_LONGPRESS_MASK)
#define BTN_IS_LONGPRESS(msg)		((msg)&BTN_LONGPRESS_MASK)
#endif

void Button_Init(void);
void Button_Handle(void);
eButton_Msg_t Button_GetValue(void);

#endif
