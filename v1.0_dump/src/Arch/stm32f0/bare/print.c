#include "serial.h"
#include "stdio.h"

#pragma import(__use_no_semihosting)
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}
struct __FILE 
{
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
FILE __stdout;       
void _sys_exit(int x) 
{
	x = x; 
}
int fputc(int ch, FILE *f)
{
	uint8_t c=ch;
	#if 1
	Serial_Transmit(SerialPort_Debug,&c,1);
	#else
	VCP_send_uint8(&c,1);
	#endif
    return ch;
}
