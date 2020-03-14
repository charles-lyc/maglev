#ifndef __FLASH_H
#define __FLASH_H

#include "common use.h"

bool Flash_SavePage(uint32_t Addr, void *pData, uint32_t Size);
bool Flash_Load(uint32_t Addr, void *pData, uint32_t Size);

#endif
