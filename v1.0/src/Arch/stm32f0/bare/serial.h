#ifndef SERIAL_H
#define SERIAL_H
#include "common use.h"
#include "configure.h"

void Serial_Init(uint8_t id, uint32_t Baudrate);
bool Serial_Transmit(uint8_t id, const void *datain, uint32_t size);
uint32_t Serial_Receive(uint8_t id, void *dataout, uint32_t buff_size);
uint32_t Serial_GetFreeSpace(uint8_t id, bool isTx);
void Serial_Flush(uint8_t id);
void Serial_TestSuilt(uint8_t id);

#endif
