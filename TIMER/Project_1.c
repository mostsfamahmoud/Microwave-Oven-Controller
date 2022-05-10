#include "systic.h"
//Systick setup
void systic_wait(unsigned long delay){
    NVIC_ST_CTRL_R = 0x00;
    NVIC_ST_RELOAD_R = delay -1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x05;
    while((NVIC_ST_CTRL_R & 0x00010000)==0){}
}

// 1 milli_second delay
void Generic_delay_m_sec(unsigned long m_sec){
unsigned long i;
	for(i=0; i< m_sec; i++)
	{
		systic_wait(16000);
	}
}

// Second delay 
void Generic_delay_sec(unsigned long sec){
unsigned long i;
	for(i=0; i< sec; i++)
	{
		Generic_delay_m_sec(1000);
	}
}

// min delay
void Generic_delay_min(unsigned long mins){
unsigned long i;
	for(i=0; i< mins; i++)
	{
		Generic_delay_sec(60);
	}
}
