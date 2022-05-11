<<<<<<< HEAD
#include "systic.h"
//Systick setup
void systic_wait(unsigned long delay){
=======
/***********************
 *
 * Module: systick timer
 *
 * File Name: timer.c
 *
 * Description: Source file for timer Driver.
 * 
 * Authors: Sherin Sameh Abd El-Samad Ali 
 *          Esraa Amr Abdelmoneam Hassan Kandil
 ***************************/
#include "timer.h"
#include "std_types.h"
/***************************
 *                       Functions Definitions                             *
 ***************************/

/*
 * Description : 
 * initialising the systick timer and the delay function 
 * Register(s) : NVIC_ST_CTRL_R , NVIC_ST_RELOAD_R ,NVIC_ST_CURRENT_R
 */
void systic_wait(uint32_t delay){
>>>>>>> 7070377226ff5c4bdb9cc92ea4deef75e28f0de3
    NVIC_ST_CTRL_R = 0x00;
    NVIC_ST_RELOAD_R = delay -1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x05;
    while((NVIC_ST_CTRL_R & 0x00010000)==0){}
}
<<<<<<< HEAD

// 1 milli_second delay
void Generic_delay_m_sec(unsigned long m_sec){
unsigned long i;
=======
/*
 * Description : 
 * delay for one millisecond
 */

void Generic_delay_m_sec(uint32_t m_sec){
uint32_t i;
>>>>>>> 7070377226ff5c4bdb9cc92ea4deef75e28f0de3
	for(i=0; i< m_sec; i++)
	{
		systic_wait(16000);
	}
}

<<<<<<< HEAD
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
=======
>>>>>>> 7070377226ff5c4bdb9cc92ea4deef75e28f0de3
