#include "systic.h"
#include "Io.h"


void SystemInit(void){

SYSCTL_RCGCGPIO_R |= 0x20; // PortF clock enable
while ((SYSCTL_PRGPIO_R & 0x20)==0); //Delay
GPIO_PORTF_LOCK_R = 0x4C4F434B; // Unlock PortF Commit register
GPIO_PORTF_CR_R |= 0x0E; // Allow changes to PF1
GPIO_PORTF_AMSEL_R &= ~0x0E; // Disable analog function
GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // GPIO clear bit PCTL
GPIO_PORTF_AFSEL_R &= ~0x0E; // No alternate function
GPIO_PORTF_DIR_R |= 0x0E; // PF321 output
GPIO_PORTF_DEN_R |= 0x0E; // Enable digital pins PF3-PF1
GPIO_PORTF_DATA_R |= 0x0E; // Initialize LEDs to be On
}

//
void systic_wait(unsigned long delay){
    NVIC_ST_CTRL_R = 0x00;
    NVIC_ST_RELOAD_R = delay -1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x05;
    while((NVIC_ST_CTRL_R & 0x00010000)==0){}
}
//_______________________________________________________
// 1 milli_second delay
void Generic_delay_m_sec(unsigned long m_sec){
unsigned long i;
	for(i=0; i< m_sec; i++)
	{
		systic_wait(16000);
	}
}
//_______________________________________________________
// Second delay 
void Generic_delay_sec(unsigned long sec){
unsigned long i;
	for(i=0; i< sec; i++)
	{
		Generic_delay_m_sec(1000);
	}
}
//________________________________________________________

void Generic_delay_min(unsigned long mins){
unsigned long i;
	for(i=0; i< mins; i++)
	{
		Generic_delay_sec(60);
	}
}
//________________________________________________________

int main(){
	SystemInit();
	Generic_delay_min(1);
	GPIO_PORTF_DATA_R &= ~0x0E;
	
}
