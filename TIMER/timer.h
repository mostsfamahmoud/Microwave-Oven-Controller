/******************************************************************************
 *
 * Module: systick timer
 *
 * File Name: timer.h
 *
 * Description: Header file for The timer Driver.
 *
 * Author: Sherin Sameh Abd El-Samad Ali 
 *         Esraa Amr Abdelmoneam Hassan Kandil
 *******************************************************************************/
#ifndef TIMER_H_
#define TIMER_H_
#include "std_types.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

#define NVIC_ST_CTRL_R       (*((volatile uint32_t *)0xE000E010))


#define NVIC_ST_RELOAD_R         (*((volatile uint32_t *)0xE000E014))


#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Description : 
 * initialising the systick timer and the delay function 
 * Register(s) : NVIC_ST_CTRL_R , NVIC_ST_RELOAD_R ,NVIC_ST_CURRENT_R
 */

void systic_wait(uint32_t delay)

/*
 * Description : 
 * delay for one millisecond
 */

void Generic_delay_m_sec(uint32_t m_sec);

void Generic_delay_sec(uint32_t sec);

void Generic_delay_min(uint32_t mins);