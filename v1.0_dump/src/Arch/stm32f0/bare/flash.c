#include "flash.h"
#include "configure.h"

// reference manual: Table 4. Flash memory organization
#ifdef STM32F072
 #define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
 #define FLASH_USER_START_ADDR   ((uint32_t)0x08009000)   /* Start @ of user Flash area */
 #define FLASH_USER_END_ADDR     ((uint32_t)0x08020000)   /* End @ of user Flash area */
#elif defined (STM32F091)
 #define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
 #define FLASH_USER_START_ADDR   ((uint32_t)0x08009000)   /* Start @ of user Flash area */
 #define FLASH_USER_END_ADDR     ((uint32_t)0x08040000)   /* End @ of user Flash area */
#else
 #define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
 #define FLASH_USER_START_ADDR   ((uint32_t)0x08000000)   /* Start @ of user Flash area */
 #define FLASH_USER_END_ADDR     ((uint32_t)0x08007000)   /* End @ of user Flash area */
#endif

// BE CAREFULL! ADDR must be PAGE ALIGNED!
bool Flash_SavePage(uint32_t Addr, void *pData, uint32_t Size)
{
	uint32_t Buffer[FLASH_PAGE_SIZE/4];
	uint32_t Pages, PageAddr;
	bool Result;
	
	if(Addr%FLASH_PAGE_SIZE)
		return false;
	
	DISABLE_INT();
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);	
	Pages=Size/FLASH_PAGE_SIZE+1;
	PageAddr=Addr;
	for(uint32_t i=0;i<Pages;i++)
	{
		memcpy(Buffer, (uint32_t*)PageAddr, FLASH_PAGE_SIZE);
		memcpy(Buffer, (uint32_t*)pData, Size%FLASH_PAGE_SIZE);		
		if(FLASH_ErasePage(PageAddr) != FLASH_COMPLETE)
			Result=false;
		
		for(uint32_t j=0;j<FLASH_PAGE_SIZE/4;j++)
		{
			if(FLASH_ProgramWord(PageAddr, Buffer[j]) != FLASH_COMPLETE)
				Result=false;
			PageAddr += 4;
		}
	}
	FLASH_Lock();
	ENABLE_INT();
	
	return Result;
}

bool Flash_Load(uint32_t Addr, void *pData, uint32_t Size)
{
	DISABLE_INT();
	memcpy(pData, (uint8_t*)Addr, Size);
	ENABLE_INT();
	return true;
}

