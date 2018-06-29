#include "delay.h"
#define CLOCK 9 // 72/8
void delay_us(unsigned int us)
{
	u8 n;		    
	while(us--)for(n=0;n<CLOCK;n++); 	 
}


void delay_ms(unsigned int ms)
{
	while(ms--)delay_us(1000);
}


