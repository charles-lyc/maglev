#include "serial.h"
#include "gpio.h"

#define UART1_DMA_CHANNEL				DMA_Channel_4
#define UART1_RX_DMA_CHANNEL			DMA1_Channel3
static uint8_t UART1_TXBuffer[UART1_TX_BUFFER_SIZE];
static uint8_t UART1_RXBuffer[UART1_RX_BUFFER_SIZE];
static uint32_t UART1_RXBufferFirst;
static bool UART1_Initialized=false;

static bool		UART2_Initialized=false;

static bool 	UART3_Initialized=false;

void Serial_Init(uint8_t id, uint32_t Baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	switch(id)
	{
		case 1:	// uart #1
			if(UART1_Initialized)
				return;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

			USART_StructInit(&USART_InitStructure);
			USART_InitStructure.USART_BaudRate = Baudrate;
			USART_Init(USART1, &USART_InitStructure);
			USART_ClearFlag(USART1, USART_FLAG_TC|USART_FLAG_RXNE);
			USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
			USART_Cmd(USART1, ENABLE);
			// DMA Rx
			DMA_StructInit(&DMA_InitStructure);
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->RDR);
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART1_RXBuffer;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
			DMA_InitStructure.DMA_BufferSize = sizeof(UART1_RXBuffer);
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
			DMA_Init(UART1_RX_DMA_CHANNEL, &DMA_InitStructure);
			DMA_Cmd(UART1_RX_DMA_CHANNEL, ENABLE);
			// Var
			UART1_RXBufferFirst=0;
			
			GPIO_InitSimple(GPIOA,GPIO_Pin_9,GPIO_AF_1,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIO_Speed_50MHz,true);
			GPIO_InitSimple(GPIOA,GPIO_Pin_10,GPIO_AF_1,GPIO_OType_OD,GPIO_PuPd_UP,GPIO_Speed_50MHz,true);
			UART1_Initialized=true;
			return;
		case 2:	// uart #2
			if(UART2_Initialized)
				return;
			UART2_Initialized=true;
			return;
		case 3:	// uart #3
			if(UART3_Initialized)
				return;
			UART3_Initialized=true;
			return;
	}
}

// Can't be used in INT!!
bool Serial_Transmit(uint8_t id, const void *datain, uint32_t data_size)
{
	uint8_t *pByte;
	
	switch(id)
	{
	case 1:
		if(!UART1_Initialized)
			return false;

		{
			if(data_size>UART1_TX_BUFFER_SIZE)
				return false;
			memcpy(UART1_TXBuffer,datain,data_size);
			pByte=UART1_TXBuffer;
			while(data_size-->0)
			{
				USART_SendData(USART1, *pByte++);
				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			}
			return true;
		}
	case 2:
	break;
	case 3:
	break;
	}
	return false;
}

#ifdef SERIAL_USE_DMA_INT
static void Serial_Handle(uint8_t id, bool isINT)
{
	uint32_t TXBufferNext;
	uint32_t TXSize;
	static bool UART_Handling=false;
	
	switch(id)
	{
	case 1:
		if(!UART1_Initialized)
			return;
		if(UART_Handling)
			return;
		UART_Handling=true;
		
		// End of a transmission
		if(DMA_GetCmdStatus(UART1_TX_DMA_STREAM)==DISABLE || isINT)
		{
			UART1_TXBufferFirst=UART1_TXBufferNextFirst;

			// Start a new transmission
			TXBufferNext=UART1_TXBufferNext;
			if(UART1_TXBufferFirst!=TXBufferNext)
			{
				if(UART1_TXBufferFirst<TXBufferNext)
				{
					TXSize=TXBufferNext-UART1_TXBufferFirst;
					UART1_TXBufferNextFirst=TXBufferNext;
				}
				else
				{
					TXSize=UART1_TX_BUFFER_SIZE-UART1_TXBufferFirst;
					UART1_TXBufferNextFirst=0;
				}
				UART1_TX_DMA_STREAM->M0AR=(uint32_t)(UART1_TXBuffer+UART1_TXBufferFirst);
				UART1_TX_DMA_STREAM->NDTR=TXSize;
				DMA_Cmd(UART1_TX_DMA_STREAM, ENABLE);
			}
		}
		UART_Handling=false;
		return;
	case 2:
		if(!UART2_Initialized)
			return;

		return;
	}
}
#endif

// Without any mutex anyhow.
uint32_t Serial_Receive(uint8_t id, void *dataout, uint32_t buff_size)
{
	uint32_t RxBufferNext;
	switch(id)
	{
	case 1:
		if(!UART1_Initialized)
			return 0;
		RxBufferNext=UART1_RX_BUFFER_SIZE-DMA_GetCurrDataCounter(UART1_RX_DMA_CHANNEL);	// Down count
		if (UART1_RXBufferFirst==RxBufferNext)
			return 0;
		else if (UART1_RXBufferFirst<RxBufferNext)
		{
			uint32_t RxSize;
			
			RxSize=RxBufferNext-UART1_RXBufferFirst;
			if (RxSize>buff_size)
				RxSize=buff_size;
			memcpy(dataout,UART1_RXBuffer+UART1_RXBufferFirst,RxSize);
			UART1_RXBufferFirst+=RxSize;
			return RxSize;
		}
		else
		{
			uint32_t RxSize;
			uint32_t RXSize2;

			RxSize=UART1_RX_BUFFER_SIZE-(UART1_RXBufferFirst-RxBufferNext);
			if (RxSize>buff_size)
				RxSize=buff_size;
			RXSize2=UART1_RX_BUFFER_SIZE-UART1_RXBufferFirst;
			if (RXSize2>buff_size)
				RXSize2=buff_size;
			memcpy(dataout,UART1_RXBuffer+UART1_RXBufferFirst,RXSize2);
			RxSize-=RXSize2;
			dataout=(uint8_t *)dataout+RXSize2;
			if (RxSize==0)
			{
				UART1_RXBufferFirst+=RXSize2;
				if (UART1_RXBufferFirst>=UART1_RX_BUFFER_SIZE)
					UART1_RXBufferFirst=0;
				return RXSize2;
			}
			else
			{
				memcpy(dataout,UART1_RXBuffer,RxSize);
				UART1_RXBufferFirst=RxSize;
				return RXSize2+RxSize;
			}
		}
	case 2:
		if(!UART2_Initialized)
			return 0;
	break;
	case 3:
		if(!UART3_Initialized)
			return 0;
	break;
	}
	return 0;
}

// Tx DMA complete IRQ
#ifdef SERIAL_USE_DMA_T_INT
void UART1_TX_DMA_IRQ_HANDLER(void)
{
    if(DMA_GetITStatus(UART1_TX_DMA_STREAM,UART1_TX_DMA_STREAM_INT_TCIF)!=RESET)
    {
        DMA_ClearITPendingBit(UART1_TX_DMA_STREAM,UART1_TX_DMA_STREAM_INT_TCIF);
		Serial_Handle(1, true);
    }
}

void UART2_TX_DMA_IRQ_HANDLER(void)
{
    if(DMA_GetITStatus(UART2_TX_DMA_STREAM,UART2_TX_DMA_STREAM_INT_TCIF)!=RESET)
    {
        DMA_ClearITPendingBit(UART2_TX_DMA_STREAM,UART2_TX_DMA_STREAM_INT_TCIF);
		xSemaphoreGiveFromISR(xMutexUART[1], NULL);
    }
}

#endif

void Serial_Flush(uint8_t id)
{
	uint8_t tmp; 
	while(Serial_Receive(id, &tmp, 1));
}

void Serial_TestSuilt(uint8_t id)
{
	static uint8_t tmp[1000];
	uint32_t Size;

	while(1)
	{
		Size=Serial_Receive(id, tmp, sizeof tmp);
		Serial_Transmit(id, tmp, Size);

		Delay_Ms(100);
	}
}
