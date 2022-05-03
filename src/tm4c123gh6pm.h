tm4c123gh6pm.h
Who has access
H
System properties
Type
Text
Size
644 KB
Storage used
644 KB
Location
TivaC GPIO Driver Template
Owner
Hussam Wael
Modified
Mar 20, 2022 by Hussam Wael
Opened
10:56 PM by me
Created
Mar 20, 2022
No description
Viewers can download
//*****************************************************************************
//
// tm4c123gh6pm.h - TM4C123GH6PM Register Definitions
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#ifndef __TM4C123GH6PM_H__
#define __TM4C123GH6PM_H__


//*****************************************************************************
//
// Interrupt assignments
//
//*****************************************************************************
#define INT_GPIOA               16          // GPIO Port A
#define INT_GPIOB               17          // GPIO Port B
#define INT_GPIOC               18          // GPIO Port C
#define INT_GPIOD               19          // GPIO Port D
#define INT_GPIOE               20          // GPIO Port E
#define INT_UART0               21          // UART0
#define INT_UART1               22          // UART1
#define INT_SSI0                23          // SSI0
#define INT_I2C0                24          // I2C0
#define INT_PWM0_FAULT          25          // PWM0 Fault
#define INT_PWM0_0              26          // PWM0 Generator 0
#define INT_PWM0_1              27          // PWM0 Generator 1
#define INT_PWM0_2              28          // PWM0 Generator 2
#define INT_QEI0                29          // QEI0
#define INT_ADC0SS0             30          // ADC0 Sequence 0
#define INT_ADC0SS1             31          // ADC0 Sequence 1
#define INT_ADC0SS2             32          // ADC0 Sequence 2
#define INT_ADC0SS3             33          // ADC0 Sequence 3
#define INT_WATCHDOG            34          // Watchdog Timers 0 and 1
#define INT_TIMER0A             35          // 16/32-Bit Timer 0A
#define INT_TIMER0B             36          // 16/32-Bit Timer 0B
#define INT_TIMER1A             37          // 16/32-Bit Timer 1A
#define INT_TIMER1B             38          // 16/32-Bit Timer 1B
#define INT_TIMER2A             39          // 16/32-Bit Timer 2A
#define INT_TIMER2B             40          // 16/32-Bit Timer 2B
#define INT_COMP0               41          // Analog Comparator 0
#define INT_COMP1               42          // Analog Comparator 1
#define INT_SYSCTL              44          // System Control
#define INT_FLASH               45          // Flash Memory Control and EEPROM
// Control
#define INT_GPIOF               46          // GPIO Port F
#define INT_UART2               49          // UART2
#define INT_SSI1                50          // SSI1
#define INT_TIMER3A             51          // 16/32-Bit Timer 3A
#define INT_TIMER3B             52          // Timer 3B
#define INT_I2C1                53          // I2C1
#define INT_QEI1                54          // QEI1
#define INT_CAN0                55          // CAN0
#define INT_CAN1                56          // CAN1
#define INT_HIBERNATE           59          // Hibernation Module
#define INT_USB0                60          // USB
#define INT_PWM0_3              61          // PWM Generator 3
#define INT_UDMA                62          // uDMA Software
#define INT_UDMAERR             63          // uDMA Error
#define INT_ADC1SS0             64          // ADC1 Sequence 0
#define INT_ADC1SS1             65          // ADC1 Sequence 1
#define INT_ADC1SS2             66          // ADC1 Sequence 2
#define INT_ADC1SS3             67          // ADC1 Sequence 3
#define INT_SSI2                73          // SSI2
#define INT_SSI3                74          // SSI3
#define INT_UART3               75          // UART3
#define INT_UART4               76          // UART4
#define INT_UART5               77          // UART5
#define INT_UART6               78          // UART6
#define INT_UART7               79          // UART7
#define INT_I2C2                84          // I2C2
#define INT_I2C3                85          // I2C3
#define INT_TIMER4A             86          // 16/32-Bit Timer 4A
#define INT_TIMER4B             87          // 16/32-Bit Timer 4B
#define INT_TIMER5A             108         // 16/32-Bit Timer 5A
#define INT_TIMER5B             109         // 16/32-Bit Timer 5B
#define INT_WTIMER0A            110         // 32/64-Bit Timer 0A
#define INT_WTIMER0B            111         // 32/64-Bit Timer 0B
#define INT_WTIMER1A            112         // 32/64-Bit Timer 1A
#define INT_WTIMER1B            113         // 32/64-Bit Timer 1B
#define INT_WTIMER2A            114         // 32/64-Bit Timer 2A
#define INT_WTIMER2B            115         // 32/64-Bit Timer 2B
#define INT_WTIMER3A            116         // 32/64-Bit Timer 3A
#define INT_WTIMER3B            117         // 32/64-Bit Timer 3B
#define INT_WTIMER4A            118         // 32/64-Bit Timer 4A
#define INT_WTIMER4B            119         // 32/64-Bit Timer 4B
#define INT_WTIMER5A            120         // 32/64-Bit Timer 5A
#define INT_WTIMER5B            121         // 32/64-Bit Timer 5B
#define INT_SYSEXC              122         // System Exception (imprecise)
#define INT_PWM1_0              150         // PWM1 Generator 0
#define INT_PWM1_1              151         // PWM1 Generator 1
#define INT_PWM1_2              152         // PWM1 Generator 2
#define INT_PWM1_3              153         // PWM1 Generator 3
#define INT_PWM1_FAULT          154         // PWM1 Fault

//*****************************************************************************
//
// Watchdog Timer registers (WATCHDOG0)
//
//*****************************************************************************
#define WATCHDOG0_LOAD_R        (*((volatile uint32 *)0x40000000))
#define WATCHDOG0_VALUE_R       (*((volatile uint32 *)0x40000004))
#define WATCHDOG0_CTL_R         (*((volatile uint32 *)0x40000008))
#define WATCHDOG0_ICR_R         (*((volatile uint32 *)0x4000000C))
#define WATCHDOG0_RIS_R         (*((volatile uint32 *)0x40000010))
#define WATCHDOG0_MIS_R         (*((volatile uint32 *)0x40000014))
#define WATCHDOG0_TEST_R        (*((volatile uint32 *)0x40000418))
#define WATCHDOG0_LOCK_R        (*((volatile uint32 *)0x40000C00))

//*****************************************************************************
//
// Watchdog Timer registers (WATCHDOG1)
//
//*****************************************************************************
#define WATCHDOG1_LOAD_R        (*((volatile uint32 *)0x40001000))
#define WATCHDOG1_VALUE_R       (*((volatile uint32 *)0x40001004))
#define WATCHDOG1_CTL_R         (*((volatile uint32 *)0x40001008))
#define WATCHDOG1_ICR_R         (*((volatile uint32 *)0x4000100C))
#define WATCHDOG1_RIS_R         (*((volatile uint32 *)0x40001010))
#define WATCHDOG1_MIS_R         (*((volatile uint32 *)0x40001014))
#define WATCHDOG1_TEST_R        (*((volatile uint32 *)0x40001418))
#define WATCHDOG1_LOCK_R        (*((volatile uint32 *)0x40001C00))

//*****************************************************************************
//
// GPIO registers (PORTA)
//
//*****************************************************************************
#define GPIO_PORTA_DATA_BITS_R  ((volatile uint32 *)0x40004000)
#define GPIO_PORTA_DATA_R       (*((volatile uint32 *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile uint32 *)0x40004400))
#define GPIO_PORTA_IS_R         (*((volatile uint32 *)0x40004404))
#define GPIO_PORTA_IBE_R        (*((volatile uint32 *)0x40004408))
#define GPIO_PORTA_IEV_R        (*((volatile uint32 *)0x4000440C))
#define GPIO_PORTA_IM_R         (*((volatile uint32 *)0x40004410))
#define GPIO_PORTA_RIS_R        (*((volatile uint32 *)0x40004414))
#define GPIO_PORTA_MIS_R        (*((volatile uint32 *)0x40004418))
#define GPIO_PORTA_ICR_R        (*((volatile uint32 *)0x4000441C))
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32 *)0x40004420))
#define GPIO_PORTA_DR2R_R       (*((volatile uint32 *)0x40004500))
#define GPIO_PORTA_DR4R_R       (*((volatile uint32 *)0x40004504))
#define GPIO_PORTA_DR8R_R       (*((volatile uint32 *)0x40004508))
#define GPIO_PORTA_ODR_R        (*((volatile uint32 *)0x4000450C))
#define GPIO_PORTA_PUR_R        (*((volatile uint32 *)0x40004510))
#define GPIO_PORTA_PDR_R        (*((volatile uint32 *)0x40004514))
#define GPIO_PORTA_SLR_R        (*((volatile uint32 *)0x40004518))
#define GPIO_PORTA_DEN_R        (*((volatile uint32 *)0x4000451C))
#define GPIO_PORTA_LOCK_R       (*((volatile uint32 *)0x40004520))
#define GPIO_PORTA_CR_R         (*((volatile uint32 *)0x40004524))
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32 *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile uint32 *)0x4000452C))
#define GPIO_PORTA_ADCCTL_R     (*((volatile uint32 *)0x40004530))
#define GPIO_PORTA_DMACTL_R     (*((volatile uint32 *)0x40004534))

//*****************************************************************************
//
// GPIO registers (PORTB)
//
//*****************************************************************************
#define GPIO_PORTB_DATA_BITS_R  ((volatile uint32 *)0x40005000)
#define GPIO_PORTB_DATA_R       (*((volatile uint32 *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile uint32 *)0x40005400))
#define GPIO_PORTB_IS_R         (*((volatile uint32 *)0x40005404))
#define GPIO_PORTB_IBE_R        (*((volatile uint32 *)0x40005408))
#define GPIO_PORTB_IEV_R        (*((volatile uint32 *)0x4000540C))
#define GPIO_PORTB_IM_R         (*((volatile uint32 *)0x40005410))
#define GPIO_PORTB_RIS_R        (*((volatile uint32 *)0x40005414))
#define GPIO_PORTB_MIS_R        (*((volatile uint32 *)0x40005418))
#define GPIO_PORTB_ICR_R        (*((volatile uint32 *)0x4000541C))
#define GPIO_PORTB_AFSEL_R      (*((volatile uint32 *)0x40005420))
#define GPIO_PORTB_DR2R_R       (*((volatile uint32 *)0x40005500))
#define GPIO_PORTB_DR4R_R       (*((volatile uint32 *)0x40005504))
#define GPIO_PORTB_DR8R_R       (*((volatile uint32 *)0x40005508))
#define GPIO_PORTB_ODR_R        (*((volatile uint32 *)0x4000550C))
#define GPIO_PORTB_PUR_R        (*((volatile uint32 *)0x40005510))
#define GPIO_PORTB_PDR_R        (*((volatile uint32 *)0x40005514))
#define GPIO_PORTB_SLR_R        (*((volatile uint32 *)0x40005518))
#define GPIO_PORTB_DEN_R        (*((volatile uint32 *)0x4000551C))
#define GPIO_PORTB_LOCK_R       (*((volatile uint32 *)0x40005520))
#define GPIO_PORTB_CR_R         (*((volatile uint32 *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile uint32 *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile uint32 *)0x4000552C))
#define GPIO_PORTB_ADCCTL_R     (*((volatile uint32 *)0x40005530))
#define GPIO_PORTB_DMACTL_R     (*((volatile uint32 *)0x40005534))

//*****************************************************************************
//
// GPIO registers (PORTC)
//
//*****************************************************************************
#define GPIO_PORTC_DATA_BITS_R  ((volatile uint32 *)0x40006000)
#define GPIO_PORTC_DATA_R       (*((volatile uint32 *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile uint32 *)0x40006400))
#define GPIO_PORTC_IS_R         (*((volatile uint32 *)0x40006404))
#define GPIO_PORTC_IBE_R        (*((volatile uint32 *)0x40006408))
#define GPIO_PORTC_IEV_R        (*((volatile uint32 *)0x4000640C))
#define GPIO_PORTC_IM_R         (*((volatile uint32 *)0x40006410))
#define GPIO_PORTC_RIS_R        (*((volatile uint32 *)0x40006414))
#define GPIO_PORTC_MIS_R        (*((volatile uint32 *)0x40006418))
#define GPIO_PORTC_ICR_R        (*((volatile uint32 *)0x4000641C))
#define GPIO_PORTC_AFSEL_R      (*((volatile uint32 *)0x40006420))
#define GPIO_PORTC_DR2R_R       (*((volatile uint32 *)0x40006500))
#define GPIO_PORTC_DR4R_R       (*((volatile uint32 *)0x40006504))
#define GPIO_PORTC_DR8R_R       (*((volatile uint32 *)0x40006508))
#define GPIO_PORTC_ODR_R        (*((volatile uint32 *)0x4000650C))
#define GPIO_PORTC_PUR_R        (*((volatile uint32 *)0x40006510))
#define GPIO_PORTC_PDR_R        (*((volatile uint32 *)0x40006514))
#define GPIO_PORTC_SLR_R        (*((volatile uint32 *)0x40006518))
#define GPIO_PORTC_DEN_R        (*((volatile uint32 *)0x4000651C))
#define GPIO_PORTC_LOCK_R       (*((volatile uint32 *)0x40006520))
#define GPIO_PORTC_CR_R         (*((volatile uint32 *)0x40006524))
#define GPIO_PORTC_AMSEL_R      (*((volatile uint32 *)0x40006528))
#define GPIO_PORTC_PCTL_R       (*((volatile uint32 *)0x4000652C))
#define GPIO_PORTC_ADCCTL_R     (*((volatile uint32 *)0x40006530))
#define GPIO_PORTC_DMACTL_R     (*((volatile uint32 *)0x40006534))

//*****************************************************************************
//
// GPIO registers (PORTD)
//
//*****************************************************************************
#define GPIO_PORTD_DATA_BITS_R  ((volatile uint32 *)0x40007000)
#define GPIO_PORTD_DATA_R       (*((volatile uint32 *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile uint32 *)0x40007400))
#define GPIO_PORTD_IS_R         (*((volatile uint32 *)0x40007404))
#define GPIO_PORTD_IBE_R        (*((volatile uint32 *)0x40007408))
#define GPIO_PORTD_IEV_R        (*((volatile uint32 *)0x4000740C))
#define GPIO_PORTD_IM_R         (*((volatile uint32 *)0x40007410))
#define GPIO_PORTD_RIS_R        (*((volatile uint32 *)0x40007414))
#define GPIO_PORTD_MIS_R        (*((volatile uint32 *)0x40007418))
#define GPIO_PORTD_ICR_R        (*((volatile uint32 *)0x4000741C))
#define GPIO_PORTD_AFSEL_R      (*((volatile uint32 *)0x40007420))
#define GPIO_PORTD_DR2R_R       (*((volatile uint32 *)0x40007500))
#define GPIO_PORTD_DR4R_R       (*((volatile uint32 *)0x40007504))
#define GPIO_PORTD_DR8R_R       (*((volatile uint32 *)0x40007508))
#define GPIO_PORTD_ODR_R        (*((volatile uint32 *)0x4000750C))
#define GPIO_PORTD_PUR_R        (*((volatile uint32 *)0x40007510))
#define GPIO_PORTD_PDR_R        (*((volatile uint32 *)0x40007514))
#define GPIO_PORTD_SLR_R        (*((volatile uint32 *)0x40007518))
#define GPIO_PORTD_DEN_R        (*((volatile uint32 *)0x4000751C))
#define GPIO_PORTD_LOCK_R       (*((volatile uint32 *)0x40007520))
#define GPIO_PORTD_CR_R         (*((volatile uint32 *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile uint32 *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile uint32 *)0x4000752C))
#define GPIO_PORTD_ADCCTL_R     (*((volatile uint32 *)0x40007530))
#define GPIO_PORTD_DMACTL_R     (*((volatile uint32 *)0x40007534))

//*****************************************************************************
//
// SSI registers (SSI0)
//
//*****************************************************************************
#define SSI0_CR0_R              (*((volatile uint32 *)0x40008000))
#define SSI0_CR1_R              (*((volatile uint32 *)0x40008004))
#define SSI0_DR_R               (*((volatile uint32 *)0x40008008))
#define SSI0_SR_R               (*((volatile uint32 *)0x4000800C))
#define SSI0_CPSR_R             (*((volatile uint32 *)0x40008010))
#define SSI0_IM_R               (*((volatile uint32 *)0x40008014))
#define SSI0_RIS_R              (*((volatile uint32 *)0x40008018))
#define SSI0_MIS_R              (*((volatile uint32 *)0x4000801C))
#define SSI0_ICR_R              (*((volatile uint32 *)0x40008020))
#define SSI0_DMACTL_R           (*((volatile uint32 *)0x40008024))
#define SSI0_CC_R               (*((volatile uint32 *)0x40008FC8))

//*****************************************************************************
//
// SSI registers (SSI1)
//
//*****************************************************************************
#define SSI1_CR0_R              (*((volatile uint32 *)0x40009000))
#define SSI1_CR1_R              (*((volatile uint32 *)0x40009004))
#define SSI1_DR_R               (*((volatile uint32 *)0x40009008))
#define SSI1_SR_R               (*((volatile uint32 *)0x4000900C))
#define SSI1_CPSR_R             (*((volatile uint32 *)0x40009010))
#define SSI1_IM_R               (*((volatile uint32 *)0x40009014))
#define SSI1_RIS_R              (*((volatile uint32 *)0x40009018))
#define SSI1_MIS_R              (*((volatile uint32 *)0x4000901C))
#define SSI1_ICR_R              (*((volatile uint32 *)0x40009020))
#define SSI1_DMACTL_R           (*((volatile uint32 *)0x40009024))
#define SSI1_CC_R               (*((volatile uint32 *)0x40009FC8))

//*****************************************************************************
//
// SSI registers (SSI2)
//
//*****************************************************************************
#define SSI2_CR0_R              (*((volatile uint32 *)0x4000A000))
#define SSI2_CR1_R              (*((volatile uint32 *)0x4000A004))
#define SSI2_DR_R               (*((volatile uint32 *)0x4000A008))
#define SSI2_SR_R               (*((volatile uint32 *)0x4000A00C))
#define SSI2_CPSR_R             (*((volatile uint32 *)0x4000A010))
#define SSI2_IM_R               (*((volatile uint32 *)0x4000A014))
#define SSI2_RIS_R              (*((volatile uint32 *)0x4000A018))
#define SSI2_MIS_R              (*((volatile uint32 *)0x4000A01C))
#define SSI2_ICR_R              (*((volatile uint32 *)0x4000A020))
#define SSI2_DMACTL_R           (*((volatile uint32 *)0x4000A024))
#define SSI2_CC_R               (*((volatile uint32 *)0x4000AFC8))

//*****************************************************************************
//
// SSI registers (SSI3)
//
//*****************************************************************************
#define SSI3_CR0_R              (*((volatile uint32 *)0x4000B000))
#define SSI3_CR1_R              (*((volatile uint32 *)0x4000B004))
#define SSI3_DR_R               (*((volatile uint32 *)0x4000B008))
#define SSI3_SR_R               (*((volatile uint32 *)0x4000B00C))
#define SSI3_CPSR_R             (*((volatile uint32 *)0x4000B010))
#define SSI3_IM_R               (*((volatile uint32 *)0x4000B014))
#define SSI3_RIS_R              (*((volatile uint32 *)0x4000B018))
#define SSI3_MIS_R              (*((volatile uint32 *)0x4000B01C))
#define SSI3_ICR_R              (*((volatile uint32 *)0x4000B020))
#define SSI3_DMACTL_R           (*((volatile uint32 *)0x4000B024))
#define SSI3_CC_R               (*((volatile uint32 *)0x4000BFC8))

//*****************************************************************************
//
// UART registers (UART0)
//
//*****************************************************************************
#define UART0_DR_R              (*((volatile uint32 *)0x4000C000))
#define UART0_RSR_R             (*((volatile uint32 *)0x4000C004))
#define UART0_ECR_R             (*((volatile uint32 *)0x4000C004))
#define UART0_FR_R              (*((volatile uint32 *)0x4000C018))
#define UART0_ILPR_R            (*((volatile uint32 *)0x4000C020))
#define UART0_IBRD_R            (*((volatile uint32 *)0x4000C024))
#define UART0_FBRD_R            (*((volatile uint32 *)0x4000C028))
#define UART0_LCRH_R            (*((volatile uint32 *)0x4000C02C))
#define UART0_CTL_R             (*((volatile uint32 *)0x4000C030))
#define UART0_IFLS_R            (*((volatile uint32 *)0x4000C034))
#define UART0_IM_R              (*((volatile uint32 *)0x4000C038))
#define UART0_RIS_R             (*((volatile uint32 *)0x4000C03C))
#define UART0_MIS_R             (*((volatile uint32 *)0x4000C040))
#define UART0_ICR_R             (*((volatile uint32 *)0x4000C044))
#define UART0_DMACTL_R          (*((volatile uint32 *)0x4000C048))
#define UART0_9BITADDR_R        (*((volatile uint32 *)0x4000C0A4))
#define UART0_9BITAMASK_R       (*((volatile uint32 *)0x4000C0A8))
#define UART0_PP_R              (*((volatile uint32 *)0x4000CFC0))
#define UART0_CC_R              (*((volatile uint32 *)0x4000CFC8))

//*****************************************************************************
//
// UART registers (UART1)
//
//*****************************************************************************
#define UART1_DR_R              (*((volatile uint32 *)0x4000D000))
#define UART1_RSR_R             (*((volatile uint32 *)0x4000D004))
#define UART1_ECR_R             (*((volatile uint32 *)0x4000D004))
#define UART1_FR_R              (*((volatile uint32 *)0x4000D018))
#define UART1_ILPR_R            (*((volatile uint32 *)0x4000D020))
#define UART1_IBRD_R            (*((volatile uint32 *)0x4000D024))
#define UART1_FBRD_R            (*((volatile uint32 *)0x4000D028))
#define UART1_LCRH_R            (*((volatile uint32 *)0x4000D02C))
#define UART1_CTL_R             (*((volatile uint32 *)0x4000D030))
#define UART1_IFLS_R            (*((volatile uint32 *)0x4000D034))
#define UART1_IM_R              (*((volatile uint32 *)0x4000D038))
#define UART1_RIS_R             (*((volatile uint32 *)0x4000D03C))
#define UART1_MIS_R             (*((volatile uint32 *)0x4000D040))
#define UART1_ICR_R             (*((volatile uint32 *)0x4000D044))
#define UART1_DMACTL_R          (*((volatile uint32 *)0x4000D048))
#define UART1_9BITADDR_R        (*((volatile uint32 *)0x4000D0A4))
#define UART1_9BITAMASK_R       (*((volatile uint32 *)0x4000D0A8))
#define UART1_PP_R              (*((volatile uint32 *)0x4000DFC0))
#define UART1_CC_R              (*((volatile uint32 *)0x4000DFC8))

//*****************************************************************************
//
// UART registers (UART2)
//
//*****************************************************************************
#define UART2_DR_R              (*((volatile uint32 *)0x4000E000))
#define UART2_RSR_R             (*((volatile uint32 *)0x4000E004))
#define UART2_ECR_R             (*((volatile uint32 *)0x4000E004))
#define UART2_FR_R              (*((volatile uint32 *)0x4000E018))
#define UART2_ILPR_R            (*((volatile uint32 *)0x4000E020))
#define UART2_IBRD_R            (*((volatile uint32 *)0x4000E024))
#define UART2_FBRD_R            (*((volatile uint32 *)0x4000E028))
#define UART2_LCRH_R            (*((volatile uint32 *)0x4000E02C))
#define UART2_CTL_R             (*((volatile uint32 *)0x4000E030))
#define UART2_IFLS_R            (*((volatile uint32 *)0x4000E034))
#define UART2_IM_R              (*((volatile uint32 *)0x4000E038))
#define UART2_RIS_R             (*((volatile uint32 *)0x4000E03C))
#define UART2_MIS_R             (*((volatile uint32 *)0x4000E040))
#define UART2_ICR_R             (*((volatile uint32 *)0x4000E044))
#define UART2_DMACTL_R          (*((volatile uint32 *)0x4000E048))
#define UART2_9BITADDR_R        (*((volatile uint32 *)0x4000E0A4))
#define UART2_9BITAMASK_R       (*((volatile uint32 *)0x4000E0A8))
#define UART2_PP_R              (*((volatile uint32 *)0x4000EFC0))
#define UART2_CC_R              (*((volatile uint32 *)0x4000EFC8))

//*****************************************************************************
//
// UART registers (UART3)
//
//*****************************************************************************
#define UART3_DR_R              (*((volatile uint32 *)0x4000F000))
#define UART3_RSR_R             (*((volatile uint32 *)0x4000F004))
#define UART3_ECR_R             (*((volatile uint32 *)0x4000F004))
#define UART3_FR_R              (*((volatile uint32 *)0x4000F018))
#define UART3_ILPR_R            (*((volatile uint32 *)0x4000F020))
#define UART3_IBRD_R            (*((volatile uint32 *)0x4000F024))
#define UART3_FBRD_R            (*((volatile uint32 *)0x4000F028))
#define UART3_LCRH_R            (*((volatile uint32 *)0x4000F02C))
#define UART3_CTL_R             (*((volatile uint32 *)0x4000F030))
#define UART3_IFLS_R            (*((volatile uint32 *)0x4000F034))
#define UART3_IM_R              (*((volatile uint32 *)0x4000F038))
#define UART3_RIS_R             (*((volatile uint32 *)0x4000F03C))
#define UART3_MIS_R             (*((volatile uint32 *)0x4000F040))
#define UART3_ICR_R             (*((volatile uint32 *)0x4000F044))
#define UART3_DMACTL_R          (*((volatile uint32 *)0x4000F048))
#define UART3_9BITADDR_R        (*((volatile uint32 *)0x4000F0A4))
#define UART3_9BITAMASK_R       (*((volatile uint32 *)0x4000F0A8))
#define UART3_PP_R              (*((volatile uint32 *)0x4000FFC0))
#define UART3_CC_R              (*((volatile uint32 *)0x4000FFC8))

//*****************************************************************************
//
// UART registers (UART4)
//
//*****************************************************************************
#define UART4_DR_R              (*((volatile uint32 *)0x40010000))
#define UART4_RSR_R             (*((volatile uint32 *)0x40010004))
#define UART4_ECR_R             (*((volatile uint32 *)0x40010004))
#define UART4_FR_R              (*((volatile uint32 *)0x40010018))
#define UART4_ILPR_R            (*((volatile uint32 *)0x40010020))
#define UART4_IBRD_R            (*((volatile uint32 *)0x40010024))
#define UART4_FBRD_R            (*((volatile uint32 *)0x40010028))
#define UART4_LCRH_R            (*((volatile uint32 *)0x4001002C))
#define UART4_CTL_R             (*((volatile uint32 *)0x40010030))
#define UART4_IFLS_R            (*((volatile uint32 *)0x40010034))
#define UART4_IM_R              (*((volatile uint32 *)0x40010038))
#define UART4_RIS_R             (*((volatile uint32 *)0x4001003C))
#define UART4_MIS_R             (*((volatile uint32 *)0x40010040))
#define UART4_ICR_R             (*((volatile uint32 *)0x40010044))
#define UART4_DMACTL_R          (*((volatile uint32 *)0x40010048))
#define UART4_9BITADDR_R        (*((volatile uint32 *)0x400100A4))
#define UART4_9BITAMASK_R       (*((volatile uint32 *)0x400100A8))
#define UART4_PP_R              (*((volatile uint32 *)0x40010FC0))
#define UART4_CC_R              (*((volatile uint32 *)0x40010FC8))

//*****************************************************************************
//
// UART registers (UART5)
//
//*****************************************************************************
#define UART5_DR_R              (*((volatile uint32 *)0x40011000))
#define UART5_RSR_R             (*((volatile uint32 *)0x40011004))
#define UART5_ECR_R             (*((volatile uint32 *)0x40011004))
#define UART5_FR_R              (*((volatile uint32 *)0x40011018))
#define UART5_ILPR_R            (*((volatile uint32 *)0x40011020))
#define UART5_IBRD_R            (*((volatile uint32 *)0x40011024))
#define UART5_FBRD_R            (*((volatile uint32 *)0x40011028))
#define UART5_LCRH_R            (*((volatile uint32 *)0x4001102C))
#define UART5_CTL_R             (*((volatile uint32 *)0x40011030))
#define UART5_IFLS_R            (*((volatile uint32 *)0x40011034))
#define UART5_IM_R              (*((volatile uint32 *)0x40011038))
#define UART5_RIS_R             (*((volatile uint32 *)0x4001103C))
#define UART5_MIS_R             (*((volatile uint32 *)0x40011040))
#define UART5_ICR_R             (*((volatile uint32 *)0x40011044))
#define UART5_DMACTL_R          (*((volatile uint32 *)0x40011048))
#define UART5_9BITADDR_R        (*((volatile uint32 *)0x400110A4))
#define UART5_9BITAMASK_R       (*((volatile uint32 *)0x400110A8))
#define UART5_PP_R              (*((volatile uint32 *)0x40011FC0))
#define UART5_CC_R              (*((volatile uint32 *)0x40011FC8))

//*****************************************************************************
//
// UART registers (UART6)
//
//*****************************************************************************
#define UART6_DR_R              (*((volatile uint32 *)0x40012000))
#define UART6_RSR_R             (*((volatile uint32 *)0x40012004))
#define UART6_ECR_R             (*((volatile uint32 *)0x40012004))
#define UART6_FR_R              (*((volatile uint32 *)0x40012018))
#define UART6_ILPR_R            (*((volatile uint32 *)0x40012020))
#define UART6_IBRD_R            (*((volatile uint32 *)0x40012024))
#define UART6_FBRD_R            (*((volatile uint32 *)0x40012028))
#define UART6_LCRH_R            (*((volatile uint32 *)0x4001202C))
#define UART6_CTL_R             (*((volatile uint32 *)0x40012030))
#define UART6_IFLS_R            (*((volatile uint32 *)0x40012034))
#define UART6_IM_R              (*((volatile uint32 *)0x40012038))
#define UART6_RIS_R             (*((volatile uint32 *)0x4001203C))
#define UART6_MIS_R             (*((volatile uint32 *)0x40012040))
#define UART6_ICR_R             (*((volatile uint32 *)0x40012044))
#define UART6_DMACTL_R          (*((volatile uint32 *)0x40012048))
#define UART6_9BITADDR_R        (*((volatile uint32 *)0x400120A4))
#define UART6_9BITAMASK_R       (*((volatile uint32 *)0x400120A8))
#define UART6_PP_R              (*((volatile uint32 *)0x40012FC0))
#define UART6_CC_R              (*((volatile uint32 *)0x40012FC8))

//*****************************************************************************
//
// UART registers (UART7)
//
//*****************************************************************************
#define UART7_DR_R              (*((volatile uint32 *)0x40013000))
#define UART7_RSR_R             (*((volatile uint32 *)0x40013004))
#define UART7_ECR_R             (*((volatile uint32 *)0x40013004))
#define UART7_FR_R              (*((volatile uint32 *)0x40013018))
#define UART7_ILPR_R            (*((volatile uint32 *)0x40013020))
#define UART7_IBRD_R            (*((volatile uint32 *)0x40013024))
#define UART7_FBRD_R            (*((volatile uint32 *)0x40013028))
#define UART7_LCRH_R            (*((volatile uint32 *)0x4001302C))
#define UART7_CTL_R             (*((volatile uint32 *)0x40013030))
#define UART7_IFLS_R            (*((volatile uint32 *)0x40013034))
#define UART7_IM_R              (*((volatile uint32 *)0x40013038))
#define UART7_RIS_R             (*((volatile uint32 *)0x4001303C))
#define UART7_MIS_R             (*((volatile uint32 *)0x40013040))
#define UART7_ICR_R             (*((volatile uint32 *)0x40013044))
#define UART7_DMACTL_R          (*((volatile uint32 *)0x40013048))
#define UART7_9BITADDR_R        (*((volatile uint32 *)0x400130A4))
#define UART7_9BITAMASK_R       (*((volatile uint32 *)0x400130A8))
#define UART7_PP_R              (*((volatile uint32 *)0x40013FC0))
#define UART7_CC_R              (*((volatile uint32 *)0x40013FC8))

//*****************************************************************************
//
// I2C registers (I2C0)
//
//*****************************************************************************
#define I2C0_MSA_R              (*((volatile uint32 *)0x40020000))
#define I2C0_MCS_R              (*((volatile uint32 *)0x40020004))
#define I2C0_MDR_R              (*((volatile uint32 *)0x40020008))
#define I2C0_MTPR_R             (*((volatile uint32 *)0x4002000C))
#define I2C0_MIMR_R             (*((volatile uint32 *)0x40020010))
#define I2C0_MRIS_R             (*((volatile uint32 *)0x40020014))
#define I2C0_MMIS_R             (*((volatile uint32 *)0x40020018))
#define I2C0_MICR_R             (*((volatile uint32 *)0x4002001C))
#define I2C0_MCR_R              (*((volatile uint32 *)0x40020020))
#define I2C0_MCLKOCNT_R         (*((volatile uint32 *)0x40020024))
#define I2C0_MBMON_R            (*((volatile uint32 *)0x4002002C))
#define I2C0_MCR2_R             (*((volatile uint32 *)0x40020038))
#define I2C0_SOAR_R             (*((volatile uint32 *)0x40020800))
#define I2C0_SCSR_R             (*((volatile uint32 *)0x40020804))
#define I2C0_SDR_R              (*((volatile uint32 *)0x40020808))
#define I2C0_SIMR_R             (*((volatile uint32 *)0x4002080C))
#define I2C0_SRIS_R             (*((volatile uint32 *)0x40020810))
#define I2C0_SMIS_R             (*((volatile uint32 *)0x40020814))
#define I2C0_SICR_R             (*((volatile uint32 *)0x40020818))
#define I2C0_SOAR2_R            (*((volatile uint32 *)0x4002081C))
#define I2C0_SACKCTL_R          (*((volatile uint32 *)0x40020820))
#define I2C0_PP_R               (*((volatile uint32 *)0x40020FC0))
#define I2C0_PC_R               (*((volatile uint32 *)0x40020FC4))

//*****************************************************************************
//
// I2C registers (I2C1)
//
//*****************************************************************************
#define I2C1_MSA_R              (*((volatile uint32 *)0x40021000))
#define I2C1_MCS_R              (*((volatile uint32 *)0x40021004))
#define I2C1_MDR_R              (*((volatile uint32 *)0x40021008))
#define I2C1_MTPR_R             (*((volatile uint32 *)0x4002100C))
#define I2C1_MIMR_R             (*((volatile uint32 *)0x40021010))
#define I2C1_MRIS_R             (*((volatile uint32 *)0x40021014))
#define I2C1_MMIS_R             (*((volatile uint32 *)0x40021018))
#define I2C1_MICR_R             (*((volatile uint32 *)0x4002101C))
#define I2C1_MCR_R              (*((volatile uint32 *)0x40021020))
#define I2C1_MCLKOCNT_R         (*((volatile uint32 *)0x40021024))
#define I2C1_MBMON_R            (*((volatile uint32 *)0x4002102C))
#define I2C1_MCR2_R             (*((volatile uint32 *)0x40021038))
#define I2C1_SOAR_R             (*((volatile uint32 *)0x40021800))
#define I2C1_SCSR_R             (*((volatile uint32 *)0x40021804))
#define I2C1_SDR_R              (*((volatile uint32 *)0x40021808))
#define I2C1_SIMR_R             (*((volatile uint32 *)0x4002180C))
#define I2C1_SRIS_R             (*((volatile uint32 *)0x40021810))
#define I2C1_SMIS_R             (*((volatile uint32 *)0x40021814))
#define I2C1_SICR_R             (*((volatile uint32 *)0x40021818))
#define I2C1_SOAR2_R            (*((volatile uint32 *)0x4002181C))
#define I2C1_SACKCTL_R          (*((volatile uint32 *)0x40021820))
#define I2C1_PP_R               (*((volatile uint32 *)0x40021FC0))
#define I2C1_PC_R               (*((volatile uint32 *)0x40021FC4))

//*****************************************************************************
//
// I2C registers (I2C2)
//
//*****************************************************************************
#define I2C2_MSA_R              (*((volatile uint32 *)0x40022000))
#define I2C2_MCS_R              (*((volatile uint32 *)0x40022004))
#define I2C2_MDR_R              (*((volatile uint32 *)0x40022008))
#define I2C2_MTPR_R             (*((volatile uint32 *)0x4002200C))
#define I2C2_MIMR_R             (*((volatile uint32 *)0x40022010))
#define I2C2_MRIS_R             (*((volatile uint32 *)0x40022014))
#define I2C2_MMIS_R             (*((volatile uint32 *)0x40022018))
#define I2C2_MICR_R             (*((volatile uint32 *)0x4002201C))
#define I2C2_MCR_R              (*((volatile uint32 *)0x40022020))
#define I2C2_MCLKOCNT_R         (*((volatile uint32 *)0x40022024))
#define I2C2_MBMON_R            (*((volatile uint32 *)0x4002202C))
#define I2C2_MCR2_R             (*((volatile uint32 *)0x40022038))
#define I2C2_SOAR_R             (*((volatile uint32 *)0x40022800))
#define I2C2_SCSR_R             (*((volatile uint32 *)0x40022804))
#define I2C2_SDR_R              (*((volatile uint32 *)0x40022808))
#define I2C2_SIMR_R             (*((volatile uint32 *)0x4002280C))
#define I2C2_SRIS_R             (*((volatile uint32 *)0x40022810))
#define I2C2_SMIS_R             (*((volatile uint32 *)0x40022814))
#define I2C2_SICR_R             (*((volatile uint32 *)0x40022818))
#define I2C2_SOAR2_R            (*((volatile uint32 *)0x4002281C))
#define I2C2_SACKCTL_R          (*((volatile uint32 *)0x40022820))
#define I2C2_PP_R               (*((volatile uint32 *)0x40022FC0))
#define I2C2_PC_R               (*((volatile uint32 *)0x40022FC4))

//*****************************************************************************
//
// I2C registers (I2C3)
//
//*****************************************************************************
#define I2C3_MSA_R              (*((volatile uint32 *)0x40023000))
#define I2C3_MCS_R              (*((volatile uint32 *)0x40023004))
#define I2C3_MDR_R              (*((volatile uint32 *)0x40023008))
#define I2C3_MTPR_R             (*((volatile uint32 *)0x4002300C))
#define I2C3_MIMR_R             (*((volatile uint32 *)0x40023010))
#define I2C3_MRIS_R             (*((volatile uint32 *)0x40023014))
#define I2C3_MMIS_R             (*((volatile uint32 *)0x40023018))
#define I2C3_MICR_R             (*((volatile uint32 *)0x4002301C))
#define I2C3_MCR_R              (*((volatile uint32 *)0x40023020))
#define I2C3_MCLKOCNT_R         (*((volatile uint32 *)0x40023024))
#define I2C3_MBMON_R            (*((volatile uint32 *)0x4002302C))
#define I2C3_MCR2_R             (*((volatile uint32 *)0x40023038))
#define I2C3_SOAR_R             (*((volatile uint32 *)0x40023800))
#define I2C3_SCSR_R             (*((volatile uint32 *)0x40023804))
#define I2C3_SDR_R              (*((volatile uint32 *)0x40023808))
#define I2C3_SIMR_R             (*((volatile uint32 *)0x4002380C))
#define I2C3_SRIS_R             (*((volatile uint32 *)0x40023810))
#define I2C3_SMIS_R             (*((volatile uint32 *)0x40023814))
#define I2C3_SICR_R             (*((volatile uint32 *)0x40023818))
#define I2C3_SOAR2_R            (*((volatile uint32 *)0x4002381C))
#define I2C3_SACKCTL_R          (*((volatile uint32 *)0x40023820))
#define I2C3_PP_R               (*((volatile uint32 *)0x40023FC0))
#define I2C3_PC_R               (*((volatile uint32 *)0x40023FC4))

//*****************************************************************************
//
// GPIO registers (PORTE)
//
//*****************************************************************************
#define GPIO_PORTE_DATA_BITS_R  ((volatile uint32 *)0x40024000)
#define GPIO_PORTE_DATA_R       (*((volatile uint32 *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile uint32 *)0x40024400))
#define GPIO_PORTE_IS_R         (*((volatile uint32 *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile uint32 *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile uint32 *)0x4002440C))
#define GPIO_PORTE_IM_R         (*((volatile uint32 *)0x40024410))
#define GPIO_PORTE_RIS_R        (*((volatile uint32 *)0x40024414))
#define GPIO_PORTE_MIS_R        (*((volatile uint32 *)0x40024418))
#define GPIO_PORTE_ICR_R        (*((volatile uint32 *)0x4002441C))
#define GPIO_PORTE_AFSEL_R      (*((volatile uint32 *)0x40024420))
#define GPIO_PORTE_DR2R_R       (*((volatile uint32 *)0x40024500))
#define GPIO_PORTE_DR4R_R       (*((volatile uint32 *)0x40024504))
#define GPIO_PORTE_DR8R_R       (*((volatile uint32 *)0x40024508))
#define GPIO_PORTE_ODR_R        (*((volatile uint32 *)0x4002450C))
#define GPIO_PORTE_PUR_R        (*((volatile uint32 *)0x40024510))
#define GPIO_PORTE_PDR_R        (*((volatile uint32 *)0x40024514))
#define GPIO_PORTE_SLR_R        (*((volatile uint32 *)0x40024518))
#define GPIO_PORTE_DEN_R        (*((volatile uint32 *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile uint32 *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile uint32 *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile uint32 *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile uint32 *)0x4002452C))
#define GPIO_PORTE_ADCCTL_R     (*((volatile uint32 *)0x40024530))
#define GPIO_PORTE_DMACTL_R     (*((volatile uint32 *)0x40024534))

//*****************************************************************************
//
// GPIO registers (PORTF)
//
//*****************************************************************************
#define GPIO_PORTF_DATA_BITS_R  ((volatile uint32 *)0x40025000)
#define GPIO_PORTF_DATA_R       (*((volatile uint32 *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile uint32 *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile uint32 *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile uint32 *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile uint32 *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile uint32 *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile uint32 *)0x40025414))
#define GPIO_PORTF_MIS_R        (*((volatile uint32 *)0x40025418))
#define GPIO_PORTF_ICR_R        (*((volatile uint32 *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32 *)0x40025420))
#define GPIO_PORTF_DR2R_R       (*((volatile uint32 *)0x40025500))
#define GPIO_PORTF_DR4R_R       (*((volatile uint32 *)0x40025504))
#define GPIO_PORTF_DR8R_R       (*((volatile uint32 *)0x40025508))
#define GPIO_PORTF_ODR_R        (*((volatile uint32 *)0x4002550C))
#define GPIO_PORTF_PUR_R        (*((volatile uint32 *)0x40025510))
#define GPIO_PORTF_PDR_R        (*((volatile uint32 *)0x40025514))
#define GPIO_PORTF_SLR_R        (*((volatile uint32 *)0x40025518))
#define GPIO_PORTF_DEN_R        (*((volatile uint32 *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile uint32 *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile uint32 *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32 *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile uint32 *)0x4002552C))
#define GPIO_PORTF_ADCCTL_R     (*((volatile uint32 *)0x40025530))
#define GPIO_PORTF_DMACTL_R     (*((volatile uint32 *)0x40025534))

//*****************************************************************************
//
// PWM registers (PWM0)
//
//*****************************************************************************
#define PWM0_CTL_R              (*((volatile uint32 *)0x40028000))
#define PWM0_SYNC_R             (*((volatile uint32 *)0x40028004))
#define PWM0_ENABLE_R           (*((volatile uint32 *)0x40028008))
#define PWM0_INVERT_R           (*((volatile uint32 *)0x4002800C))
#define PWM0_FAULT_R            (*((volatile uint32 *)0x40028010))
#define PWM0_INTEN_R            (*((volatile uint32 *)0x40028014))
#define PWM0_RIS_R              (*((volatile uint32 *)0x40028018))
#define PWM0_ISC_R              (*((volatile uint32 *)0x4002801C))
#define PWM0_STATUS_R           (*((volatile uint32 *)0x40028020))
#define PWM0_FAULTVAL_R         (*((volatile uint32 *)0x40028024))
#define PWM0_ENUPD_R            (*((volatile uint32 *)0x40028028))
#define PWM0_0_CTL_R            (*((volatile uint32 *)0x40028040))
#define PWM0_0_INTEN_R          (*((volatile uint32 *)0x40028044))
#define PWM0_0_RIS_R            (*((volatile uint32 *)0x40028048))
#define PWM0_0_ISC_R            (*((volatile uint32 *)0x4002804C))
#define PWM0_0_LOAD_R           (*((volatile uint32 *)0x40028050))
#define PWM0_0_COUNT_R          (*((volatile uint32 *)0x40028054))
#define PWM0_0_CMPA_R           (*((volatile uint32 *)0x40028058))
#define PWM0_0_CMPB_R           (*((volatile uint32 *)0x4002805C))
#define PWM0_0_GENA_R           (*((volatile uint32 *)0x40028060))
#define PWM0_0_GENB_R           (*((volatile uint32 *)0x40028064))
#define PWM0_0_DBCTL_R          (*((volatile uint32 *)0x40028068))
#define PWM0_0_DBRISE_R         (*((volatile uint32 *)0x4002806C))
#define PWM0_0_DBFALL_R         (*((volatile uint32 *)0x40028070))
#define PWM0_0_FLTSRC0_R        (*((volatile uint32 *)0x40028074))
#define PWM0_0_FLTSRC1_R        (*((volatile uint32 *)0x40028078))
#define PWM0_0_MINFLTPER_R      (*((volatile uint32 *)0x4002807C))
#define PWM0_1_CTL_R            (*((volatile uint32 *)0x40028080))
#define PWM0_1_INTEN_R          (*((volatile uint32 *)0x40028084))
#define PWM0_1_RIS_R            (*((volatile uint32 *)0x40028088))
#define PWM0_1_ISC_R            (*((volatile uint32 *)0x4002808C))
#define PWM0_1_LOAD_R           (*((volatile uint32 *)0x40028090))
#define PWM0_1_COUNT_R          (*((volatile uint32 *)0x40028094))
#define PWM0_1_CMPA_R           (*((volatile uint32 *)0x40028098))
#define PWM0_1_CMPB_R           (*((volatile uint32 *)0x4002809C))
#define PWM0_1_GENA_R           (*((volatile uint32 *)0x400280A0))
#define PWM0_1_GENB_R           (*((volatile uint32 *)0x400280A4))
#define PWM0_1_DBCTL_R          (*((volatile uint32 *)0x400280A8))
#define PWM0_1_DBRISE_R         (*((volatile uint32 *)0x400280AC))
#define PWM0_1_DBFALL_R         (*((volatile uint32 *)0x400280B0))
#define PWM0_1_FLTSRC0_R        (*((volatile uint32 *)0x400280B4))
#define PWM0_1_FLTSRC1_R        (*((volatile uint32 *)0x400280B8))
#define PWM0_1_MINFLTPER_R      (*((volatile uint32 *)0x400280BC))
#define PWM0_2_CTL_R            (*((volatile uint32 *)0x400280C0))
#define PWM0_2_INTEN_R          (*((volatile uint32 *)0x400280C4))
#define PWM0_2_RIS_R            (*((volatile uint32 *)0x400280C8))
#define PWM0_2_ISC_R            (*((volatile uint32 *)0x400280CC))
#define PWM0_2_LOAD_R           (*((volatile uint32 *)0x400280D0))
#define PWM0_2_COUNT_R          (*((volatile uint32 *)0x400280D4))
#define PWM0_2_CMPA_R           (*((volatile uint32 *)0x400280D8))
#define PWM0_2_CMPB_R           (*((volatile uint32 *)0x400280DC))
#define PWM0_2_GENA_R           (*((volatile uint32 *)0x400280E0))
#define PWM0_2_GENB_R           (*((volatile uint32 *)0x400280E4))
#define PWM0_2_DBCTL_R          (*((volatile uint32 *)0x400280E8))
#define PWM0_2_DBRISE_R         (*((volatile uint32 *)0x400280EC))
#define PWM0_2_DBFALL_R         (*((volatile uint32 *)0x400280F0))
#define PWM0_2_FLTSRC0_R        (*((volatile uint32 *)0x400280F4))
#define PWM0_2_FLTSRC1_R        (*((volatile uint32 *)0x400280F8))
#define PWM0_2_MINFLTPER_R      (*((volatile uint32 *)0x400280FC))
#define PWM0_3_CTL_R            (*((volatile uint32 *)0x40028100))
#define PWM0_3_INTEN_R          (*((volatile uint32 *)0x40028104))
#define PWM0_3_RIS_R            (*((volatile uint32 *)0x40028108))
#define PWM0_3_ISC_R            (*((volatile uint32 *)0x4002810C))
#define PWM0_3_LOAD_R           (*((volatile uint32 *)0x40028110))
#define PWM0_3_COUNT_R          (*((volatile uint32 *)0x40028114))
#define PWM0_3_CMPA_R           (*((volatile uint32 *)0x40028118))
#define PWM0_3_CMPB_R           (*((volatile uint32 *)0x4002811C))
#define PWM0_3_GENA_R           (*((volatile uint32 *)0x40028120))
#define PWM0_3_GENB_R           (*((volatile uint32 *)0x40028124))
#define PWM0_3_DBCTL_R          (*((volatile uint32 *)0x40028128))
#define PWM0_3_DBRISE_R         (*((volatile uint32 *)0x4002812C))
#define PWM0_3_DBFALL_R         (*((volatile uint32 *)0x40028130))
#define PWM0_3_FLTSRC0_R        (*((volatile uint32 *)0x40028134))
#define PWM0_3_FLTSRC1_R        (*((volatile uint32 *)0x40028138))
#define PWM0_3_MINFLTPER_R      (*((volatile uint32 *)0x4002813C))
#define PWM0_0_FLTSEN_R         (*((volatile uint32 *)0x40028800))
#define PWM0_0_FLTSTAT0_R       (*((volatile uint32 *)0x40028804))
#define PWM0_0_FLTSTAT1_R       (*((volatile uint32 *)0x40028808))
#define PWM0_1_FLTSEN_R         (*((volatile uint32 *)0x40028880))
#define PWM0_1_FLTSTAT0_R       (*((volatile uint32 *)0x40028884))
#define PWM0_1_FLTSTAT1_R       (*((volatile uint32 *)0x40028888))
#define PWM0_2_FLTSTAT0_R       (*((volatile uint32 *)0x40028904))
#define PWM0_2_FLTSTAT1_R       (*((volatile uint32 *)0x40028908))
#define PWM0_3_FLTSTAT0_R       (*((volatile uint32 *)0x40028984))
#define PWM0_3_FLTSTAT1_R       (*((volatile uint32 *)0x40028988))
#define PWM0_PP_R               (*((volatile uint32 *)0x40028FC0))

//*****************************************************************************
//
// PWM registers (PWM1)
//
//*****************************************************************************
#define PWM1_CTL_R              (*((volatile uint32 *)0x40029000))
#define PWM1_SYNC_R             (*((volatile uint32 *)0x40029004))
#define PWM1_ENABLE_R           (*((volatile uint32 *)0x40029008))
#define PWM1_INVERT_R           (*((volatile uint32 *)0x4002900C))
#define PWM1_FAULT_R            (*((volatile uint32 *)0x40029010))
#define PWM1_INTEN_R            (*((volatile uint32 *)0x40029014))
#define PWM1_RIS_R              (*((volatile uint32 *)0x40029018))
#define PWM1_ISC_R              (*((volatile uint32 *)0x4002901C))
#define PWM1_STATUS_R           (*((volatile uint32 *)0x40029020))
#define PWM1_FAULTVAL_R         (*((volatile uint32 *)0x40029024))
#define PWM1_ENUPD_R            (*((volatile uint32 *)0x40029028))
#define PWM1_0_CTL_R            (*((volatile uint32 *)0x40029040))
#define PWM1_0_INTEN_R          (*((volatile uint32 *)0x40029044))
#define PWM1_0_RIS_R            (*((volatile uint32 *)0x40029048))
#define PWM1_0_ISC_R            (*((volatile uint32 *)0x4002904C))
#define PWM1_0_LOAD_R           (*((volatile uint32 *)0x40029050))
#define PWM1_0_COUNT_R          (*((volatile uint32 *)0x40029054))
#define PWM1_0_CMPA_R           (*((volatile uint32 *)0x40029058))
#define PWM1_0_CMPB_R           (*((volatile uint32 *)0x4002905C))
#define PWM1_0_GENA_R           (*((volatile uint32 *)0x40029060))
#define PWM1_0_GENB_R           (*((volatile uint32 *)0x40029064))
#define PWM1_0_DBCTL_R          (*((volatile uint32 *)0x40029068))
#define PWM1_0_DBRISE_R         (*((volatile uint32 *)0x4002906C))
#define PWM1_0_DBFALL_R         (*((volatile uint32 *)0x40029070))
#define PWM1_0_FLTSRC0_R        (*((volatile uint32 *)0x40029074))
#define PWM1_0_FLTSRC1_R        (*((volatile uint32 *)0x40029078))
#define PWM1_0_MINFLTPER_R      (*((volatile uint32 *)0x4002907C))
#define PWM1_1_CTL_R            (*((volatile uint32 *)0x40029080))
#define PWM1_1_INTEN_R          (*((volatile uint32 *)0x40029084))
#define PWM1_1_RIS_R            (*((volatile uint32 *)0x40029088))
#define PWM1_1_ISC_R            (*((volatile uint32 *)0x4002908C))
#define PWM1_1_LOAD_R           (*((volatile uint32 *)0x40029090))
#define PWM1_1_COUNT_R          (*((volatile uint32 *)0x40029094))
#define PWM1_1_CMPA_R           (*((volatile uint32 *)0x40029098))
#define PWM1_1_CMPB_R           (*((volatile uint32 *)0x4002909C))
#define PWM1_1_GENA_R           (*((volatile uint32 *)0x400290A0))
#define PWM1_1_GENB_R           (*((volatile uint32 *)0x400290A4))
#define PWM1_1_DBCTL_R          (*((volatile uint32 *)0x400290A8))
#define PWM1_1_DBRISE_R         (*((volatile uint32 *)0x400290AC))
#define PWM1_1_DBFALL_R         (*((volatile uint32 *)0x400290B0))
#define PWM1_1_FLTSRC0_R        (*((volatile uint32 *)0x400290B4))
#define PWM1_1_FLTSRC1_R        (*((volatile uint32 *)0x400290B8))
#define PWM1_1_MINFLTPER_R      (*((volatile uint32 *)0x400290BC))
#define PWM1_2_CTL_R            (*((volatile uint32 *)0x400290C0))
#define PWM1_2_INTEN_R          (*((volatile uint32 *)0x400290C4))
#define PWM1_2_RIS_R            (*((volatile uint32 *)0x400290C8))
#define PWM1_2_ISC_R            (*((volatile uint32 *)0x400290CC))
#define PWM1_2_LOAD_R           (*((volatile uint32 *)0x400290D0))
#define PWM1_2_COUNT_R          (*((volatile uint32 *)0x400290D4))
#define PWM1_2_CMPA_R           (*((volatile uint32 *)0x400290D8))
#define PWM1_2_CMPB_R           (*((volatile uint32 *)0x400290DC))
#define PWM1_2_GENA_R           (*((volatile uint32 *)0x400290E0))
#define PWM1_2_GENB_R           (*((volatile uint32 *)0x400290E4))
#define PWM1_2_DBCTL_R          (*((volatile uint32 *)0x400290E8))
#define PWM1_2_DBRISE_R         (*((volatile uint32 *)0x400290EC))
#define PWM1_2_DBFALL_R         (*((volatile uint32 *)0x400290F0))
#define PWM1_2_FLTSRC0_R        (*((volatile uint32 *)0x400290F4))
#define PWM1_2_FLTSRC1_R        (*((volatile uint32 *)0x400290F8))
#define PWM1_2_MINFLTPER_R      (*((volatile uint32 *)0x400290FC))
#define PWM1_3_CTL_R            (*((volatile uint32 *)0x40029100))
#define PWM1_3_INTEN_R          (*((volatile uint32 *)0x40029104))
#define PWM1_3_RIS_R            (*((volatile uint32 *)0x40029108))
#define PWM1_3_ISC_R            (*((volatile uint32 *)0x4002910C))
#define PWM1_3_LOAD_R           (*((volatile uint32 *)0x40029110))
#define PWM1_3_COUNT_R          (*((volatile uint32 *)0x40029114))
#define PWM1_3_CMPA_R           (*((volatile uint32 *)0x40029118))
#define PWM1_3_CMPB_R           (*((volatile uint32 *)0x4002911C))
#define PWM1_3_GENA_R           (*((volatile uint32 *)0x40029120))
#define PWM1_3_GENB_R           (*((volatile uint32 *)0x40029124))
#define PWM1_3_DBCTL_R          (*((volatile uint32 *)0x40029128))
#define PWM1_3_DBRISE_R         (*((volatile uint32 *)0x4002912C))
#define PWM1_3_DBFALL_R         (*((volatile uint32 *)0x40029130))
#define PWM1_3_FLTSRC0_R        (*((volatile uint32 *)0x40029134))
#define PWM1_3_FLTSRC1_R        (*((volatile uint32 *)0x40029138))
#define PWM1_3_MINFLTPER_R      (*((volatile uint32 *)0x4002913C))
#define PWM1_0_FLTSEN_R         (*((volatile uint32 *)0x40029800))
#define PWM1_0_FLTSTAT0_R       (*((volatile uint32 *)0x40029804))
#define PWM1_0_FLTSTAT1_R       (*((volatile uint32 *)0x40029808))
#define PWM1_1_FLTSEN_R         (*((volatile uint32 *)0x40029880))
#define PWM1_1_FLTSTAT0_R       (*((volatile uint32 *)0x40029884))
#define PWM1_1_FLTSTAT1_R       (*((volatile uint32 *)0x40029888))
#define PWM1_2_FLTSTAT0_R       (*((volatile uint32 *)0x40029904))
#define PWM1_2_FLTSTAT1_R       (*((volatile uint32 *)0x40029908))
#define PWM1_3_FLTSTAT0_R       (*((volatile uint32 *)0x40029984))
#define PWM1_3_FLTSTAT1_R       (*((volatile uint32 *)0x40029988))
#define PWM1_PP_R               (*((volatile uint32 *)0x40029FC0))

//*****************************************************************************
//
// QEI registers (QEI0)
//
//*****************************************************************************
#define QEI0_CTL_R              (*((volatile uint32 *)0x4002C000))
#define QEI0_STAT_R             (*((volatile uint32 *)0x4002C004))
#define QEI0_POS_R              (*((volatile uint32 *)0x4002C008))
#define QEI0_MAXPOS_R           (*((volatile uint32 *)0x4002C00C))
#define QEI0_LOAD_R             (*((volatile uint32 *)0x4002C010))
#define QEI0_TIME_R             (*((volatile uint32 *)0x4002C014))
#define QEI0_COUNT_R            (*((volatile uint32 *)0x4002C018))
#define QEI0_SPEED_R            (*((volatile uint32 *)0x4002C01C))
#define QEI0_INTEN_R            (*((volatile uint32 *)0x4002C020))
#define QEI0_RIS_R              (*((volatile uint32 *)0x4002C024))
#define QEI0_ISC_R              (*((volatile uint32 *)0x4002C028))

//*****************************************************************************
//
// QEI registers (QEI1)
//
//*****************************************************************************
#define QEI1_CTL_R              (*((volatile uint32 *)0x4002D000))
#define QEI1_STAT_R             (*((volatile uint32 *)0x4002D004))
#define QEI1_POS_R              (*((volatile uint32 *)0x4002D008))
#define QEI1_MAXPOS_R           (*((volatile uint32 *)0x4002D00C))
#define QEI1_LOAD_R             (*((volatile uint32 *)0x4002D010))
#define QEI1_TIME_R             (*((volatile uint32 *)0x4002D014))
#define QEI1_COUNT_R            (*((volatile uint32 *)0x4002D018))
#define QEI1_SPEED_R            (*((volatile uint32 *)0x4002D01C))
#define QEI1_INTEN_R            (*((volatile uint32 *)0x4002D020))
#define QEI1_RIS_R              (*((volatile uint32 *)0x4002D024))
#define QEI1_ISC_R              (*((volatile uint32 *)0x4002D028))

//*****************************************************************************
//
// Timer registers (TIMER0)
//
//*****************************************************************************
#define TIMER0_CFG_R            (*((volatile uint32 *)0x40030000))
#define TIMER0_TAMR_R           (*((volatile uint32 *)0x40030004))
#define TIMER0_TBMR_R           (*((volatile uint32 *)0x40030008))
#define TIMER0_CTL_R            (*((volatile uint32 *)0x4003000C))
#define TIMER0_SYNC_R           (*((volatile uint32 *)0x40030010))
#define TIMER0_IMR_R            (*((volatile uint32 *)0x40030018))
#define TIMER0_RIS_R            (*((volatile uint32 *)0x4003001C))
#define TIMER0_MIS_R            (*((volatile uint32 *)0x40030020))
#define TIMER0_ICR_R            (*((volatile uint32 *)0x40030024))
#define TIMER0_TAILR_R          (*((volatile uint32 *)0x40030028))
#define TIMER0_TBILR_R          (*((volatile uint32 *)0x4003002C))
#define TIMER0_TAMATCHR_R       (*((volatile uint32 *)0x40030030))
#define TIMER0_TBMATCHR_R       (*((volatile uint32 *)0x40030034))
#define TIMER0_TAPR_R           (*((volatile uint32 *)0x40030038))
#define TIMER0_TBPR_R           (*((volatile uint32 *)0x4003003C))
#define TIMER0_TAPMR_R          (*((volatile uint32 *)0x40030040))
#define TIMER0_TBPMR_R          (*((volatile uint32 *)0x40030044))
#define TIMER0_TAR_R            (*((volatile uint32 *)0x40030048))
#define TIMER0_TBR_R            (*((volatile uint32 *)0x4003004C))
#define TIMER0_TAV_R            (*((volatile uint32 *)0x40030050))
#define TIMER0_TBV_R            (*((volatile uint32 *)0x40030054))
#define TIMER0_RTCPD_R          (*((volatile uint32 *)0x40030058))
#define TIMER0_TAPS_R           (*((volatile uint32 *)0x4003005C))
#define TIMER0_TBPS_R           (*((volatile uint32 *)0x40030060))
#define TIMER0_TAPV_R           (*((volatile uint32 *)0x40030064))
#define TIMER0_TBPV_R           (*((volatile uint32 *)0x40030068))
#define TIMER0_PP_R             (*((volatile uint32 *)0x40030FC0))

//*****************************************************************************
//
// Timer registers (TIMER1)
//
//*****************************************************************************
#define TIMER1_CFG_R            (*((volatile uint32 *)0x40031000))
#define TIMER1_TAMR_R           (*((volatile uint32 *)0x40031004))
#define TIMER1_TBMR_R           (*((volatile uint32 *)0x40031008))
#define TIMER1_CTL_R            (*((volatile uint32 *)0x4003100C))
#define TIMER1_SYNC_R           (*((volatile uint32 *)0x40031010))
#define TIMER1_IMR_R            (*((volatile uint32 *)0x40031018))
#define TIMER1_RIS_R            (*((volatile uint32 *)0x4003101C))
#define TIMER1_MIS_R            (*((volatile uint32 *)0x40031020))
#define TIMER1_ICR_R            (*((volatile uint32 *)0x40031024))
#define TIMER1_TAILR_R          (*((volatile uint32 *)0x40031028))
#define TIMER1_TBILR_R          (*((volatile uint32 *)0x4003102C))
#define TIMER1_TAMATCHR_R       (*((volatile uint32 *)0x40031030))
#define TIMER1_TBMATCHR_R       (*((volatile uint32 *)0x40031034))
#define TIMER1_TAPR_R           (*((volatile uint32 *)0x40031038))
#define TIMER1_TBPR_R           (*((volatile uint32 *)0x4003103C))
#define TIMER1_TAPMR_R          (*((volatile uint32 *)0x40031040))
#define TIMER1_TBPMR_R          (*((volatile uint32 *)0x40031044))
#define TIMER1_TAR_R            (*((volatile uint32 *)0x40031048))
#define TIMER1_TBR_R            (*((volatile uint32 *)0x4003104C))
#define TIMER1_TAV_R            (*((volatile uint32 *)0x40031050))
#define TIMER1_TBV_R            (*((volatile uint32 *)0x40031054))
#define TIMER1_RTCPD_R          (*((volatile uint32 *)0x40031058))
#define TIMER1_TAPS_R           (*((volatile uint32 *)0x4003105C))
#define TIMER1_TBPS_R           (*((volatile uint32 *)0x40031060))
#define TIMER1_TAPV_R           (*((volatile uint32 *)0x40031064))
#define TIMER1_TBPV_R           (*((volatile uint32 *)0x40031068))
#define TIMER1_PP_R             (*((volatile uint32 *)0x40031FC0))

//*****************************************************************************
//
// Timer registers (TIMER2)
//
//*****************************************************************************
#define TIMER2_CFG_R            (*((volatile uint32 *)0x40032000))
#define TIMER2_TAMR_R           (*((volatile uint32 *)0x40032004))
#define TIMER2_TBMR_R           (*((volatile uint32 *)0x40032008))
#define TIMER2_CTL_R            (*((volatile uint32 *)0x4003200C))
#define TIMER2_SYNC_R           (*((volatile uint32 *)0x40032010))
#define TIMER2_IMR_R            (*((volatile uint32 *)0x40032018))
#define TIMER2_RIS_R            (*((volatile uint32 *)0x4003201C))
#define TIMER2_MIS_R            (*((volatile uint32 *)0x40032020))
#define TIMER2_ICR_R            (*((volatile uint32 *)0x40032024))
#define TIMER2_TAILR_R          (*((volatile uint32 *)0x40032028))
#define TIMER2_TBILR_R          (*((volatile uint32 *)0x4003202C))
#define TIMER2_TAMATCHR_R       (*((volatile uint32 *)0x40032030))
#define TIMER2_TBMATCHR_R       (*((volatile uint32 *)0x40032034))
#define TIMER2_TAPR_R           (*((volatile uint32 *)0x40032038))
#define TIMER2_TBPR_R           (*((volatile uint32 *)0x4003203C))
#define TIMER2_TAPMR_R          (*((volatile uint32 *)0x40032040))
#define TIMER2_TBPMR_R          (*((volatile uint32 *)0x40032044))
#define TIMER2_TAR_R            (*((volatile uint32 *)0x40032048))
#define TIMER2_TBR_R            (*((volatile uint32 *)0x4003204C))
#define TIMER2_TAV_R            (*((volatile uint32 *)0x40032050))
#define TIMER2_TBV_R            (*((volatile uint32 *)0x40032054))
#define TIMER2_RTCPD_R          (*((volatile uint32 *)0x40032058))
#define TIMER2_TAPS_R           (*((volatile uint32 *)0x4003205C))
#define TIMER2_TBPS_R           (*((volatile uint32 *)0x40032060))
#define TIMER2_TAPV_R           (*((volatile uint32 *)0x40032064))
#define TIMER2_TBPV_R           (*((volatile uint32 *)0x40032068))
#define TIMER2_PP_R             (*((volatile uint32 *)0x40032FC0))

//*****************************************************************************
//
// Timer registers (TIMER3)
//
//*****************************************************************************
#define TIMER3_CFG_R            (*((volatile uint32 *)0x40033000))
#define TIMER3_TAMR_R           (*((volatile uint32 *)0x40033004))
#define TIMER3_TBMR_R           (*((volatile uint32 *)0x40033008))
#define TIMER3_CTL_R            (*((volatile uint32 *)0x4003300C))
#define TIMER3_SYNC_R           (*((volatile uint32 *)0x40033010))
#define TIMER3_IMR_R            (*((volatile uint32 *)0x40033018))
#define TIMER3_RIS_R            (*((volatile uint32 *)0x4003301C))
#define TIMER3_MIS_R            (*((volatile uint32 *)0x40033020))
#define TIMER3_ICR_R            (*((volatile uint32 *)0x40033024))
#define TIMER3_TAILR_R          (*((volatile uint32 *)0x40033028))
#define TIMER3_TBILR_R          (*((volatile uint32 *)0x4003302C))
#define TIMER3_TAMATCHR_R       (*((volatile uint32 *)0x40033030))
#define TIMER3_TBMATCHR_R       (*((volatile uint32 *)0x40033034))
#define TIMER3_TAPR_R           (*((volatile uint32 *)0x40033038))
#define TIMER3_TBPR_R           (*((volatile uint32 *)0x4003303C))
#define TIMER3_TAPMR_R          (*((volatile uint32 *)0x40033040))
#define TIMER3_TBPMR_R          (*((volatile uint32 *)0x40033044))
#define TIMER3_TAR_R            (*((volatile uint32 *)0x40033048))
#define TIMER3_TBR_R            (*((volatile uint32 *)0x4003304C))
#define TIMER3_TAV_R            (*((volatile uint32 *)0x40033050))
#define TIMER3_TBV_R            (*((volatile uint32 *)0x40033054))
#define TIMER3_RTCPD_R          (*((volatile uint32 *)0x40033058))
#define TIMER3_TAPS_R           (*((volatile uint32 *)0x4003305C))
#define TIMER3_TBPS_R           (*((volatile uint32 *)0x40033060))
#define TIMER3_TAPV_R           (*((volatile uint32 *)0x40033064))
#define TIMER3_TBPV_R           (*((volatile uint32 *)0x40033068))
#define TIMER3_PP_R             (*((volatile uint32 *)0x40033FC0))

//*****************************************************************************
//
// Timer registers (TIMER4)
//
//*****************************************************************************
#define TIMER4_CFG_R            (*((volatile uint32 *)0x40034000))
#define TIMER4_TAMR_R           (*((volatile uint32 *)0x40034004))
#define TIMER4_TBMR_R           (*((volatile uint32 *)0x40034008))
#define TIMER4_CTL_R            (*((volatile uint32 *)0x4003400C))
#define TIMER4_SYNC_R           (*((volatile uint32 *)0x40034010))
#define TIMER4_IMR_R            (*((volatile uint32 *)0x40034018))
#define TIMER4_RIS_R            (*((volatile uint32 *)0x4003401C))
#define TIMER4_MIS_R            (*((volatile uint32 *)0x40034020))
#define TIMER4_ICR_R            (*((volatile uint32 *)0x40034024))
#define TIMER4_TAILR_R          (*((volatile uint32 *)0x40034028))
#define TIMER4_TBILR_R          (*((volatile uint32 *)0x4003402C))
#define TIMER4_TAMATCHR_R       (*((volatile uint32 *)0x40034030))
#define TIMER4_TBMATCHR_R       (*((volatile uint32 *)0x40034034))
#define TIMER4_TAPR_R           (*((volatile uint32 *)0x40034038))
#define TIMER4_TBPR_R           (*((volatile uint32 *)0x4003403C))
#define TIMER4_TAPMR_R          (*((volatile uint32 *)0x40034040))
#define TIMER4_TBPMR_R          (*((volatile uint32 *)0x40034044))
#define TIMER4_TAR_R            (*((volatile uint32 *)0x40034048))
#define TIMER4_TBR_R            (*((volatile uint32 *)0x4003404C))
#define TIMER4_TAV_R            (*((volatile uint32 *)0x40034050))
#define TIMER4_TBV_R            (*((volatile uint32 *)0x40034054))
#define TIMER4_RTCPD_R          (*((volatile uint32 *)0x40034058))
#define TIMER4_TAPS_R           (*((volatile uint32 *)0x4003405C))
#define TIMER4_TBPS_R           (*((volatile uint32 *)0x40034060))
#define TIMER4_TAPV_R           (*((volatile uint32 *)0x40034064))
#define TIMER4_TBPV_R           (*((volatile uint32 *)0x40034068))
#define TIMER4_PP_R             (*((volatile uint32 *)0x40034FC0))

//*****************************************************************************
//
// Timer registers (TIMER5)
//
//*****************************************************************************
#define TIMER5_CFG_R            (*((volatile uint32 *)0x40035000))
#define TIMER5_TAMR_R           (*((volatile uint32 *)0x40035004))
#define TIMER5_TBMR_R           (*((volatile uint32 *)0x40035008))
#define TIMER5_CTL_R            (*((volatile uint32 *)0x4003500C))
#define TIMER5_SYNC_R           (*((volatile uint32 *)0x40035010))
#define TIMER5_IMR_R            (*((volatile uint32 *)0x40035018))
#define TIMER5_RIS_R            (*((volatile uint32 *)0x4003501C))
#define TIMER5_MIS_R            (*((volatile uint32 *)0x40035020))
#define TIMER5_ICR_R            (*((volatile uint32 *)0x40035024))
#define TIMER5_TAILR_R          (*((volatile uint32 *)0x40035028))
#define TIMER5_TBILR_R          (*((volatile uint32 *)0x4003502C))
#define TIMER5_TAMATCHR_R       (*((volatile uint32 *)0x40035030))
#define TIMER5_TBMATCHR_R       (*((volatile uint32 *)0x40035034))
#define TIMER5_TAPR_R           (*((volatile uint32 *)0x40035038))
#define TIMER5_TBPR_R           (*((volatile uint32 *)0x4003503C))
#define TIMER5_TAPMR_R          (*((volatile uint32 *)0x40035040))
#define TIMER5_TBPMR_R          (*((volatile uint32 *)0x40035044))
#define TIMER5_TAR_R            (*((volatile uint32 *)0x40035048))
#define TIMER5_TBR_R            (*((volatile uint32 *)0x4003504C))
#define TIMER5_TAV_R            (*((volatile uint32 *)0x40035050))
#define TIMER5_TBV_R            (*((volatile uint32 *)0x40035054))
#define TIMER5_RTCPD_R          (*((volatile uint32 *)0x40035058))
#define TIMER5_TAPS_R           (*((volatile uint32 *)0x4003505C))
#define TIMER5_TBPS_R           (*((volatile uint32 *)0x40035060))
#define TIMER5_TAPV_R           (*((volatile uint32 *)0x40035064))
#define TIMER5_TBPV_R           (*((volatile uint32 *)0x40035068))
#define TIMER5_PP_R             (*((volatile uint32 *)0x40035FC0))

//*****************************************************************************
//
// Timer registers (WTIMER0)
//
//*****************************************************************************
#define WTIMER0_CFG_R           (*((volatile uint32 *)0x40036000))
#define WTIMER0_TAMR_R          (*((volatile uint32 *)0x40036004))
#define WTIMER0_TBMR_R          (*((volatile uint32 *)0x40036008))
#define WTIMER0_CTL_R           (*((volatile uint32 *)0x4003600C))
#define WTIMER0_SYNC_R          (*((volatile uint32 *)0x40036010))
#define WTIMER0_IMR_R           (*((volatile uint32 *)0x40036018))
#define WTIMER0_RIS_R           (*((volatile uint32 *)0x4003601C))
#define WTIMER0_MIS_R           (*((volatile uint32 *)0x40036020))
#define WTIMER0_ICR_R           (*((volatile uint32 *)0x40036024))
#define WTIMER0_TAILR_R         (*((volatile uint32 *)0x40036028))
#define WTIMER0_TBILR_R         (*((volatile uint32 *)0x4003602C))
#define WTIMER0_TAMATCHR_R      (*((volatile uint32 *)0x40036030))
#define WTIMER0_TBMATCHR_R      (*((volatile uint32 *)0x40036034))
#define WTIMER0_TAPR_R          (*((volatile uint32 *)0x40036038))
#define WTIMER0_TBPR_R          (*((volatile uint32 *)0x4003603C))
#define WTIMER0_TAPMR_R         (*((volatile uint32 *)0x40036040))
#define WTIMER0_TBPMR_R         (*((volatile uint32 *)0x40036044))
#define WTIMER0_TAR_R           (*((volatile uint32 *)0x40036048))
#define WTIMER0_TBR_R           (*((volatile uint32 *)0x4003604C))
#define WTIMER0_TAV_R           (*((volatile uint32 *)0x40036050))
#define WTIMER0_TBV_R           (*((volatile uint32 *)0x40036054))
#define WTIMER0_RTCPD_R         (*((volatile uint32 *)0x40036058))
#define WTIMER0_TAPS_R          (*((volatile uint32 *)0x4003605C))
#define WTIMER0_TBPS_R          (*((volatile uint32 *)0x40036060))
#define WTIMER0_TAPV_R          (*((volatile uint32 *)0x40036064))
#define WTIMER0_TBPV_R          (*((volatile uint32 *)0x40036068))
#define WTIMER0_PP_R            (*((volatile uint32 *)0x40036FC0))

//*****************************************************************************
//
// Timer registers (WTIMER1)
//
//*****************************************************************************
#define WTIMER1_CFG_R           (*((volatile uint32 *)0x40037000))
#define WTIMER1_TAMR_R          (*((volatile uint32 *)0x40037004))
#define WTIMER1_TBMR_R          (*((volatile uint32 *)0x40037008))
#define WTIMER1_CTL_R           (*((volatile uint32 *)0x4003700C))
#define WTIMER1_SYNC_R          (*((volatile uint32 *)0x40037010))
#define WTIMER1_IMR_R           (*((volatile uint32 *)0x40037018))
#define WTIMER1_RIS_R           (*((volatile uint32 *)0x4003701C))
#define WTIMER1_MIS_R           (*((volatile uint32 *)0x40037020))
#define WTIMER1_ICR_R           (*((volatile uint32 *)0x40037024))
#define WTIMER1_TAILR_R         (*((volatile uint32 *)0x40037028))
#define WTIMER1_TBILR_R         (*((volatile uint32 *)0x4003702C))
#define WTIMER1_TAMATCHR_R      (*((volatile uint32 *)0x40037030))
#define WTIMER1_TBMATCHR_R      (*((volatile uint32 *)0x40037034))
#define WTIMER1_TAPR_R          (*((volatile uint32 *)0x40037038))
#define WTIMER1_TBPR_R          (*((volatile uint32 *)0x4003703C))
#define WTIMER1_TAPMR_R         (*((volatile uint32 *)0x40037040))
#define WTIMER1_TBPMR_R         (*((volatile uint32 *)0x40037044))
#define WTIMER1_TAR_R           (*((volatile uint32 *)0x40037048))
#define WTIMER1_TBR_R           (*((volatile uint32 *)0x4003704C))
#define WTIMER1_TAV_R           (*((volatile uint32 *)0x40037050))
#define WTIMER1_TBV_R           (*((volatile uint32 *)0x40037054))
#define WTIMER1_RTCPD_R         (*((volatile uint32 *)0x40037058))
#define WTIMER1_TAPS_R          (*((volatile uint32 *)0x4003705C))
#define WTIMER1_TBPS_R          (*((volatile uint32 *)0x40037060))
#define WTIMER1_TAPV_R          (*((volatile uint32 *)0x40037064))
#define WTIMER1_TBPV_R          (*((volatile uint32 *)0x40037068))
#define WTIMER1_PP_R            (*((volatile uint32 *)0x40037FC0))

//*****************************************************************************
//
// ADC registers (ADC0)
//
//*****************************************************************************
#define ADC0_ACTSS_R            (*((volatile uint32 *)0x40038000))
#define ADC0_RIS_R              (*((volatile uint32 *)0x40038004))
#define ADC0_IM_R               (*((volatile uint32 *)0x40038008))
#define ADC0_ISC_R              (*((volatile uint32 *)0x4003800C))
#define ADC0_OSTAT_R            (*((volatile uint32 *)0x40038010))
#define ADC0_EMUX_R             (*((volatile uint32 *)0x40038014))
#define ADC0_USTAT_R            (*((volatile uint32 *)0x40038018))
#define ADC0_TSSEL_R            (*((volatile uint32 *)0x4003801C))
#define ADC0_SSPRI_R            (*((volatile uint32 *)0x40038020))
#define ADC0_SPC_R              (*((volatile uint32 *)0x40038024))
#define ADC0_PSSI_R             (*((volatile uint32 *)0x40038028))
#define ADC0_SAC_R              (*((volatile uint32 *)0x40038030))
#define ADC0_DCISC_R            (*((volatile uint32 *)0x40038034))
#define ADC0_CTL_R              (*((volatile uint32 *)0x40038038))
#define ADC0_SSMUX0_R           (*((volatile uint32 *)0x40038040))
#define ADC0_SSCTL0_R           (*((volatile uint32 *)0x40038044))
#define ADC0_SSFIFO0_R          (*((volatile uint32 *)0x40038048))
#define ADC0_SSFSTAT0_R         (*((volatile uint32 *)0x4003804C))
#define ADC0_SSOP0_R            (*((volatile uint32 *)0x40038050))
#define ADC0_SSDC0_R            (*((volatile uint32 *)0x40038054))
#define ADC0_SSMUX1_R           (*((volatile uint32 *)0x40038060))
#define ADC0_SSCTL1_R           (*((volatile uint32 *)0x40038064))
#define ADC0_SSFIFO1_R          (*((volatile uint32 *)0x40038068))
#define ADC0_SSFSTAT1_R         (*((volatile uint32 *)0x4003806C))
#define ADC0_SSOP1_R            (*((volatile uint32 *)0x40038070))
#define ADC0_SSDC1_R            (*((volatile uint32 *)0x40038074))
#define ADC0_SSMUX2_R           (*((volatile uint32 *)0x40038080))
#define ADC0_SSCTL2_R           (*((volatile uint32 *)0x40038084))
#define ADC0_SSFIFO2_R          (*((volatile uint32 *)0x40038088))
#define ADC0_SSFSTAT2_R         (*((volatile uint32 *)0x4003808C))
#define ADC0_SSOP2_R            (*((volatile uint32 *)0x40038090))
#define ADC0_SSDC2_R            (*((volatile uint32 *)0x40038094))
#define ADC0_SSMUX3_R           (*((volatile uint32 *)0x400380A0))
#define ADC0_SSCTL3_R           (*((volatile uint32 *)0x400380A4))
#define ADC0_SSFIFO3_R          (*((volatile uint32 *)0x400380A8))
#define ADC0_SSFSTAT3_R         (*((volatile uint32 *)0x400380AC))
#define ADC0_SSOP3_R            (*((volatile uint32 *)0x400380B0))
#define ADC0_SSDC3_R            (*((volatile uint32 *)0x400380B4))
#define ADC0_DCRIC_R            (*((volatile uint32 *)0x40038D00))
#define ADC0_DCCTL0_R           (*((volatile uint32 *)0x40038E00))
#define ADC0_DCCTL1_R           (*((volatile uint32 *)0x40038E04))
#define ADC0_DCCTL2_R           (*((volatile uint32 *)0x40038E08))
#define ADC0_DCCTL3_R           (*((volatile uint32 *)0x40038E0C))
#define ADC0_DCCTL4_R           (*((volatile uint32 *)0x40038E10))
#define ADC0_DCCTL5_R           (*((volatile uint32 *)0x40038E14))
#define ADC0_DCCTL6_R           (*((volatile uint32 *)0x40038E18))
#define ADC0_DCCTL7_R           (*((volatile uint32 *)0x40038E1C))
#define ADC0_DCCMP0_R           (*((volatile uint32 *)0x40038E40))
#define ADC0_DCCMP1_R           (*((volatile uint32 *)0x40038E44))
#define ADC0_DCCMP2_R           (*((volatile uint32 *)0x40038E48))
#define ADC0_DCCMP3_R           (*((volatile uint32 *)0x40038E4C))
#define ADC0_DCCMP4_R           (*((volatile uint32 *)0x40038E50))
#define ADC0_DCCMP5_R           (*((volatile uint32 *)0x40038E54))
#define ADC0_DCCMP6_R           (*((volatile uint32 *)0x40038E58))
#define ADC0_DCCMP7_R           (*((volatile uint32 *)0x40038E5C))
#define ADC0_PP_R               (*((volatile uint32 *)0x40038FC0))
#define ADC0_PC_R               (*((volatile uint32 *)0x40038FC4))
#define ADC0_CC_R               (*((volatile uint32 *)0x40038FC8))

//*****************************************************************************
//
// ADC registers (ADC1)
//
//*****************************************************************************
#define ADC1_ACTSS_R            (*((volatile uint32 *)0x40039000))
#define ADC1_RIS_R              (*((volatile uint32 *)0x40039004))
#define ADC1_IM_R               (*((volatile uint32 *)0x40039008))
#define ADC1_ISC_R              (*((volatile uint32 *)0x4003900C))
#define ADC1_OSTAT_R            (*((volatile uint32 *)0x40039010))
#define ADC1_EMUX_R             (*((volatile uint32 *)0x40039014))
#define ADC1_USTAT_R            (*((volatile uint32 *)0x40039018))
#define ADC1_TSSEL_R            (*((volatile uint32 *)0x4003901C))
#define ADC1_SSPRI_R            (*((volatile uint32 *)0x40039020))
#define ADC1_SPC_R              (*((volatile uint32 *)0x40039024))
#define ADC1_PSSI_R             (*((volatile uint32 *)0x40039028))
#define ADC1_SAC_R              (*((volatile uint32 *)0x40039030))
#define ADC1_DCISC_R            (*((volatile uint32 *)0x40039034))
#define ADC1_CTL_R              (*((volatile uint32 *)0x40039038))
#define ADC1_SSMUX0_R           (*((volatile uint32 *)0x40039040))
#define ADC1_SSCTL0_R           (*((volatile uint32 *)0x40039044))
#define ADC1_SSFIFO0_R          (*((volatile uint32 *)0x40039048))
#define ADC1_SSFSTAT0_R         (*((volatile uint32 *)0x4003904C))
#define ADC1_SSOP0_R            (*((volatile uint32 *)0x40039050))
#define ADC1_SSDC0_R            (*((volatile uint32 *)0x40039054))
#define ADC1_SSMUX1_R           (*((volatile uint32 *)0x40039060))
#define ADC1_SSCTL1_R           (*((volatile uint32 *)0x40039064))
#define ADC1_SSFIFO1_R          (*((volatile uint32 *)0x40039068))
#define ADC1_SSFSTAT1_R         (*((volatile uint32 *)0x4003906C))
#define ADC1_SSOP1_R            (*((volatile uint32 *)0x40039070))
#define ADC1_SSDC1_R            (*((volatile uint32 *)0x40039074))
#define ADC1_SSMUX2_R           (*((volatile uint32 *)0x40039080))
#define ADC1_SSCTL2_R           (*((volatile uint32 *)0x40039084))
#define ADC1_SSFIFO2_R          (*((volatile uint32 *)0x40039088))
#define ADC1_SSFSTAT2_R         (*((volatile uint32 *)0x4003908C))
#define ADC1_SSOP2_R            (*((volatile uint32 *)0x40039090))
#define ADC1_SSDC2_R            (*((volatile uint32 *)0x40039094))
#define ADC1_SSMUX3_R           (*((volatile uint32 *)0x400390A0))
#define ADC1_SSCTL3_R           (*((volatile uint32 *)0x400390A4))
#define ADC1_SSFIFO3_R          (*((volatile uint32 *)0x400390A8))
#define ADC1_SSFSTAT3_R         (*((volatile uint32 *)0x400390AC))
#define ADC1_SSOP3_R            (*((volatile uint32 *)0x400390B0))
#define ADC1_SSDC3_R            (*((volatile uint32 *)0x400390B4))
#define ADC1_DCRIC_R            (*((volatile uint32 *)0x40039D00))
#define ADC1_DCCTL0_R           (*((volatile uint32 *)0x40039E00))
#define ADC1_DCCTL1_R           (*((volatile uint32 *)0x40039E04))
#define ADC1_DCCTL2_R           (*((volatile uint32 *)0x40039E08))
#define ADC1_DCCTL3_R           (*((volatile uint32 *)0x40039E0C))
#define ADC1_DCCTL4_R           (*((volatile uint32 *)0x40039E10))
#define ADC1_DCCTL5_R           (*((volatile uint32 *)0x40039E14))
#define ADC1_DCCTL6_R           (*((volatile uint32 *)0x40039E18))
#define ADC1_DCCTL7_R           (*((volatile uint32 *)0x40039E1C))
#define ADC1_DCCMP0_R           (*((volatile uint32 *)0x40039E40))
#define ADC1_DCCMP1_R           (*((volatile uint32 *)0x40039E44))
#define ADC1_DCCMP2_R           (*((volatile uint32 *)0x40039E48))
#define ADC1_DCCMP3_R           (*((volatile uint32 *)0x40039E4C))
#define ADC1_DCCMP4_R           (*((volatile uint32 *)0x40039E50))
#define ADC1_DCCMP5_R           (*((volatile uint32 *)0x40039E54))
#define ADC1_DCCMP6_R           (*((volatile uint32 *)0x40039E58))
#define ADC1_DCCMP7_R           (*((volatile uint32 *)0x40039E5C))
#define ADC1_PP_R               (*((volatile uint32 *)0x40039FC0))
#define ADC1_PC_R               (*((volatile uint32 *)0x40039FC4))
#define ADC1_CC_R               (*((volatile uint32 *)0x40039FC8))

//*****************************************************************************
//
// Comparator registers (COMP)
//
//*****************************************************************************
#define COMP_ACMIS_R            (*((volatile uint32 *)0x4003C000))
#define COMP_ACRIS_R            (*((volatile uint32 *)0x4003C004))
#define COMP_ACINTEN_R          (*((volatile uint32 *)0x4003C008))
#define COMP_ACREFCTL_R         (*((volatile uint32 *)0x4003C010))
#define COMP_ACSTAT0_R          (*((volatile uint32 *)0x4003C020))
#define COMP_ACCTL0_R           (*((volatile uint32 *)0x4003C024))
#define COMP_ACSTAT1_R          (*((volatile uint32 *)0x4003C040))
#define COMP_ACCTL1_R           (*((volatile uint32 *)0x4003C044))
#define COMP_PP_R               (*((volatile uint32 *)0x4003CFC0))

//*****************************************************************************
//
// CAN registers (CAN0)
//
//*****************************************************************************
#define CAN0_CTL_R              (*((volatile uint32 *)0x40040000))
#define CAN0_STS_R              (*((volatile uint32 *)0x40040004))
#define CAN0_ERR_R              (*((volatile uint32 *)0x40040008))
#define CAN0_BIT_R              (*((volatile uint32 *)0x4004000C))
#define CAN0_INT_R              (*((volatile uint32 *)0x40040010))
#define CAN0_TST_R              (*((volatile uint32 *)0x40040014))
#define CAN0_BRPE_R             (*((volatile uint32 *)0x40040018))
#define CAN0_IF1CRQ_R           (*((volatile uint32 *)0x40040020))
#define CAN0_IF1CMSK_R          (*((volatile uint32 *)0x40040024))
#define CAN0_IF1MSK1_R          (*((volatile uint32 *)0x40040028))
#define CAN0_IF1MSK2_R          (*((volatile uint32 *)0x4004002C))
#define CAN0_IF1ARB1_R          (*((volatile uint32 *)0x40040030))
#define CAN0_IF1ARB2_R          (*((volatile uint32 *)0x40040034))
#define CAN0_IF1MCTL_R          (*((volatile uint32 *)0x40040038))
#define CAN0_IF1DA1_R           (*((volatile uint32 *)0x4004003C))
#define CAN0_IF1DA2_R           (*((volatile uint32 *)0x40040040))
#define CAN0_IF1DB1_R           (*((volatile uint32 *)0x40040044))
#define CAN0_IF1DB2_R           (*((volatile uint32 *)0x40040048))
#define CAN0_IF2CRQ_R           (*((volatile uint32 *)0x40040080))
#define CAN0_IF2CMSK_R          (*((volatile uint32 *)0x40040084))
#define CAN0_IF2MSK1_R          (*((volatile uint32 *)0x40040088))
#define CAN0_IF2MSK2_R          (*((volatile uint32 *)0x4004008C))
#define CAN0_IF2ARB1_R          (*((volatile uint32 *)0x40040090))
#define CAN0_IF2ARB2_R          (*((volatile uint32 *)0x40040094))
#define CAN0_IF2MCTL_R          (*((volatile uint32 *)0x40040098))
#define CAN0_IF2DA1_R           (*((volatile uint32 *)0x4004009C))
#define CAN0_IF2DA2_R           (*((volatile uint32 *)0x400400A0))
#define CAN0_IF2DB1_R           (*((volatile uint32 *)0x400400A4))
#define CAN0_IF2DB2_R           (*((volatile uint32 *)0x400400A8))
#define CAN0_TXRQ1_R            (*((volatile uint32 *)0x40040100))
#define CAN0_TXRQ2_R            (*((volatile uint32 *)0x40040104))
#define CAN0_NWDA1_R            (*((volatile uint32 *)0x40040120))
#define CAN0_NWDA2_R            (*((volatile uint32 *)0x40040124))
#define CAN0_MSG1INT_R          (*((volatile uint32 *)0x40040140))
#define CAN0_MSG2INT_R          (*((volatile uint32 *)0x40040144))
#define CAN0_MSG1VAL_R          (*((volatile uint32 *)0x40040160))
#define CAN0_MSG2VAL_R          (*((volatile uint32 *)0x40040164))

//*****************************************************************************
//
// CAN registers (CAN1)
//
//*****************************************************************************
#define CAN1_CTL_R              (*((volatile uint32 *)0x40041000))
#define CAN1_STS_R              (*((volatile uint32 *)0x40041004))
#define CAN1_ERR_R              (*((volatile uint32 *)0x40041008))
#define CAN1_BIT_R              (*((volatile uint32 *)0x4004100C))
#define CAN1_INT_R              (*((volatile uint32 *)0x40041010))
#define CAN1_TST_R              (*((volatile uint32 *)0x40041014))
#define CAN1_BRPE_R             (*((volatile uint32 *)0x40041018))
#define CAN1_IF1CRQ_R           (*((volatile uint32 *)0x40041020))
#define CAN1_IF1CMSK_R          (*((volatile uint32 *)0x40041024))
#define CAN1_IF1MSK1_R          (*((volatile uint32 *)0x40041028))
#define CAN1_IF1MSK2_R          (*((volatile uint32 *)0x4004102C))
#define CAN1_IF1ARB1_R          (*((volatile uint32 *)0x40041030))
#define CAN1_IF1ARB2_R          (*((volatile uint32 *)0x40041034))
#define CAN1_IF1MCTL_R          (*((volatile uint32 *)0x40041038))
#define CAN1_IF1DA1_R           (*((volatile uint32 *)0x4004103C))
#define CAN1_IF1DA2_R           (*((volatile uint32 *)0x40041040))
#define CAN1_IF1DB1_R           (*((volatile uint32 *)0x40041044))
#define CAN1_IF1DB2_R           (*((volatile uint32 *)0x40041048))
#define CAN1_IF2CRQ_R           (*((volatile uint32 *)0x40041080))
#define CAN1_IF2CMSK_R          (*((volatile uint32 *)0x40041084))
#define CAN1_IF2MSK1_R          (*((volatile uint32 *)0x40041088))
#define CAN1_IF2MSK2_R          (*((volatile uint32 *)0x4004108C))
#define CAN1_IF2ARB1_R          (*((volatile uint32 *)0x40041090))
#define CAN1_IF2ARB2_R          (*((volatile uint32 *)0x40041094))
#define CAN1_IF2MCTL_R          (*((volatile uint32 *)0x40041098))
#define CAN1_IF2DA1_R           (*((volatile uint32 *)0x4004109C))
#define CAN1_IF2DA2_R           (*((volatile uint32 *)0x400410A0))
#define CAN1_IF2DB1_R           (*((volatile uint32 *)0x400410A4))
#define CAN1_IF2DB2_R           (*((volatile uint32 *)0x400410A8))
#define CAN1_TXRQ1_R            (*((volatile uint32 *)0x40041100))
#define CAN1_TXRQ2_R            (*((volatile uint32 *)0x40041104))
#define CAN1_NWDA1_R            (*((volatile uint32 *)0x40041120))
#define CAN1_NWDA2_R            (*((volatile uint32 *)0x40041124))
#define CAN1_MSG1INT_R          (*((volatile uint32 *)0x40041140))
#define CAN1_MSG2INT_R          (*((volatile uint32 *)0x40041144))
#define CAN1_MSG1VAL_R          (*((volatile uint32 *)0x40041160))
#define CAN1_MSG2VAL_R          (*((volatile uint32 *)0x40041164))

//*****************************************************************************
//
// Timer registers (WTIMER2)
//
//*****************************************************************************
#define WTIMER2_CFG_R           (*((volatile uint32 *)0x4004C000))
#define WTIMER2_TAMR_R          (*((volatile uint32 *)0x4004C004))
#define WTIMER2_TBMR_R          (*((volatile uint32 *)0x4004C008))
#define WTIMER2_CTL_R           (*((volatile uint32 *)0x4004C00C))
#define WTIMER2_SYNC_R          (*((volatile uint32 *)0x4004C010))
#define WTIMER2_IMR_R           (*((volatile uint32 *)0x4004C018))
#define WTIMER2_RIS_R           (*((volatile uint32 *)0x4004C01C))
#define WTIMER2_MIS_R           (*((volatile uint32 *)0x4004C020))
#define WTIMER2_ICR_R           (*((volatile uint32 *)0x4004C024))
#define WTIMER2_TAILR_R         (*((volatile uint32 *)0x4004C028))
#define WTIMER2_TBILR_R         (*((volatile uint32 *)0x4004C02C))
#define WTIMER2_TAMATCHR_R      (*((volatile uint32 *)0x4004C030))
#define WTIMER2_TBMATCHR_R      (*((volatile uint32 *)0x4004C034))
#define WTIMER2_TAPR_R          (*((volatile uint32 *)0x4004C038))
#define WTIMER2_TBPR_R          (*((volatile uint32 *)0x4004C03C))
#define WTIMER2_TAPMR_R         (*((volatile uint32 *)0x4004C040))
#define WTIMER2_TBPMR_R         (*((volatile uint32 *)0x4004C044))
#define WTIMER2_TAR_R           (*((volatile uint32 *)0x4004C048))
#define WTIMER2_TBR_R           (*((volatile uint32 *)0x4004C04C))
#define WTIMER2_TAV_R           (*((volatile uint32 *)0x4004C050))
#define WTIMER2_TBV_R           (*((volatile uint32 *)0x4004C054))
#define WTIMER2_RTCPD_R         (*((volatile uint32 *)0x4004C058))
#define WTIMER2_TAPS_R          (*((volatile uint32 *)0x4004C05C))
#define WTIMER2_TBPS_R          (*((volatile uint32 *)0x4004C060))
#define WTIMER2_TAPV_R          (*((volatile uint32 *)0x4004C064))
#define WTIMER2_TBPV_R          (*((volatile uint32 *)0x4004C068))
#define WTIMER2_PP_R            (*((volatile uint32 *)0x4004CFC0))

//*****************************************************************************
//
// Timer registers (WTIMER3)
//
//*****************************************************************************
#define WTIMER3_CFG_R           (*((volatile uint32 *)0x4004D000))
#define WTIMER3_TAMR_R          (*((volatile uint32 *)0x4004D004))
#define WTIMER3_TBMR_R          (*((volatile uint32 *)0x4004D008))
#define WTIMER3_CTL_R           (*((volatile uint32 *)0x4004D00C))
#define WTIMER3_SYNC_R          (*((volatile uint32 *)0x4004D010))
#define WTIMER3_IMR_R           (*((volatile uint32 *)0x4004D018))
#define WTIMER3_RIS_R           (*((volatile uint32 *)0x4004D01C))
#define WTIMER3_MIS_R           (*((volatile uint32 *)0x4004D020))
#define WTIMER3_ICR_R           (*((volatile uint32 *)0x4004D024))
#define WTIMER3_TAILR_R         (*((volatile uint32 *)0x4004D028))
#define WTIMER3_TBILR_R         (*((volatile uint32 *)0x4004D02C))
#define WTIMER3_TAMATCHR_R      (*((volatile uint32 *)0x4004D030))
#define WTIMER3_TBMATCHR_R      (*((volatile uint32 *)0x4004D034))
#define WTIMER3_TAPR_R          (*((volatile uint32 *)0x4004D038))
#define WTIMER3_TBPR_R          (*((volatile uint32 *)0x4004D03C))
#define WTIMER3_TAPMR_R         (*((volatile uint32 *)0x4004D040))
#define WTIMER3_TBPMR_R         (*((volatile uint32 *)0x4004D044))
#define WTIMER3_TAR_R           (*((volatile uint32 *)0x4004D048))
#define WTIMER3_TBR_R           (*((volatile uint32 *)0x4004D04C))
#define WTIMER3_TAV_R           (*((volatile uint32 *)0x4004D050))
#define WTIMER3_TBV_R           (*((volatile uint32 *)0x4004D054))
#define WTIMER3_RTCPD_R         (*((volatile uint32 *)0x4004D058))
#define WTIMER3_TAPS_R          (*((volatile uint32 *)0x4004D05C))
#define WTIMER3_TBPS_R          (*((volatile uint32 *)0x4004D060))
#define WTIMER3_TAPV_R          (*((volatile uint32 *)0x4004D064))
#define WTIMER3_TBPV_R          (*((volatile uint32 *)0x4004D068))
#define WTIMER3_PP_R            (*((volatile uint32 *)0x4004DFC0))

//*****************************************************************************
//
// Timer registers (WTIMER4)
//
//*****************************************************************************
#define WTIMER4_CFG_R           (*((volatile uint32 *)0x4004E000))
#define WTIMER4_TAMR_R          (*((volatile uint32 *)0x4004E004))
#define WTIMER4_TBMR_R          (*((volatile uint32 *)0x4004E008))
#define WTIMER4_CTL_R           (*((volatile uint32 *)0x4004E00C))
#define WTIMER4_SYNC_R          (*((volatile uint32 *)0x4004E010))
#define WTIMER4_IMR_R           (*((volatile uint32 *)0x4004E018))
#define WTIMER4_RIS_R           (*((volatile uint32 *)0x4004E01C))
#define WTIMER4_MIS_R           (*((volatile uint32 *)0x4004E020))
#define WTIMER4_ICR_R           (*((volatile uint32 *)0x4004E024))
#define WTIMER4_TAILR_R         (*((volatile uint32 *)0x4004E028))
#define WTIMER4_TBILR_R         (*((volatile uint32 *)0x4004E02C))
#define WTIMER4_TAMATCHR_R      (*((volatile uint32 *)0x4004E030))
#define WTIMER4_TBMATCHR_R      (*((volatile uint32 *)0x4004E034))
#define WTIMER4_TAPR_R          (*((volatile uint32 *)0x4004E038))
#define WTIMER4_TBPR_R          (*((volatile uint32 *)0x4004E03C))
#define WTIMER4_TAPMR_R         (*((volatile uint32 *)0x4004E040))
#define WTIMER4_TBPMR_R         (*((volatile uint32 *)0x4004E044))
#define WTIMER4_TAR_R           (*((volatile uint32 *)0x4004E048))
#define WTIMER4_TBR_R           (*((volatile uint32 *)0x4004E04C))
#define WTIMER4_TAV_R           (*((volatile uint32 *)0x4004E050))
#define WTIMER4_TBV_R           (*((volatile uint32 *)0x4004E054))
#define WTIMER4_RTCPD_R         (*((volatile uint32 *)0x4004E058))
#define WTIMER4_TAPS_R          (*((volatile uint32 *)0x4004E05C))
#define WTIMER4_TBPS_R          (*((volatile uint32 *)0x4004E060))
#define WTIMER4_TAPV_R          (*((volatile uint32 *)0x4004E064))
#define WTIMER4_TBPV_R          (*((volatile uint32 *)0x4004E068))
#define WTIMER4_PP_R            (*((volatile uint32 *)0x4004EFC0))

//*****************************************************************************
//
// Timer registers (WTIMER5)
//
//*****************************************************************************
#define WTIMER5_CFG_R           (*((volatile uint32 *)0x4004F000))
#define WTIMER5_TAMR_R          (*((volatile uint32 *)0x4004F004))
#define WTIMER5_TBMR_R          (*((volatile uint32 *)0x4004F008))
#define WTIMER5_CTL_R           (*((volatile uint32 *)0x4004F00C))
#define WTIMER5_SYNC_R          (*((volatile uint32 *)0x4004F010))
#define WTIMER5_IMR_R           (*((volatile uint32 *)0x4004F018))
#define WTIMER5_RIS_R           (*((volatile uint32 *)0x4004F01C))
#define WTIMER5_MIS_R           (*((volatile uint32 *)0x4004F020))
#define WTIMER5_ICR_R           (*((volatile uint32 *)0x4004F024))
#define WTIMER5_TAILR_R         (*((volatile uint32 *)0x4004F028))
#define WTIMER5_TBILR_R         (*((volatile uint32 *)0x4004F02C))
#define WTIMER5_TAMATCHR_R      (*((volatile uint32 *)0x4004F030))
#define WTIMER5_TBMATCHR_R      (*((volatile uint32 *)0x4004F034))
#define WTIMER5_TAPR_R          (*((volatile uint32 *)0x4004F038))
#define WTIMER5_TBPR_R          (*((volatile uint32 *)0x4004F03C))
#define WTIMER5_TAPMR_R         (*((volatile uint32 *)0x4004F040))
#define WTIMER5_TBPMR_R         (*((volatile uint32 *)0x4004F044))
#define WTIMER5_TAR_R           (*((volatile uint32 *)0x4004F048))
#define WTIMER5_TBR_R           (*((volatile uint32 *)0x4004F04C))
#define WTIMER5_TAV_R           (*((volatile uint32 *)0x4004F050))
#define WTIMER5_TBV_R           (*((volatile uint32 *)0x4004F054))
#define WTIMER5_RTCPD_R         (*((volatile uint32 *)0x4004F058))
#define WTIMER5_TAPS_R          (*((volatile uint32 *)0x4004F05C))
#define WTIMER5_TBPS_R          (*((volatile uint32 *)0x4004F060))
#define WTIMER5_TAPV_R          (*((volatile uint32 *)0x4004F064))
#define WTIMER5_TBPV_R          (*((volatile uint32 *)0x4004F068))
#define WTIMER5_PP_R            (*((volatile uint32 *)0x4004FFC0))

//*****************************************************************************
//
// Univeral Serial Bus registers (USB0)
//
//*****************************************************************************
#define USB0_FADDR_R            (*((volatile uint8 *)0x40050000))
#define USB0_POWER_R            (*((volatile uint8 *)0x40050001))
#define USB0_TXIS_R             (*((volatile uint16 *)0x40050002))
#define USB0_RXIS_R             (*((volatile uint16 *)0x40050004))
#define USB0_TXIE_R             (*((volatile uint16 *)0x40050006))
#define USB0_RXIE_R             (*((volatile uint16 *)0x40050008))
#define USB0_IS_R               (*((volatile uint8 *)0x4005000A))
#define USB0_IE_R               (*((volatile uint8 *)0x4005000B))
#define USB0_FRAME_R            (*((volatile uint16 *)0x4005000C))
#define USB0_EPIDX_R            (*((volatile uint8 *)0x4005000E))
#define USB0_TEST_R             (*((volatile uint8 *)0x4005000F))
#define USB0_FIFO0_R            (*((volatile uint32 *)0x40050020))
#define USB0_FIFO1_R            (*((volatile uint32 *)0x40050024))
#define USB0_FIFO2_R            (*((volatile uint32 *)0x40050028))
#define USB0_FIFO3_R            (*((volatile uint32 *)0x4005002C))
#define USB0_FIFO4_R            (*((volatile uint32 *)0x40050030))
#define USB0_FIFO5_R            (*((volatile uint32 *)0x40050034))
#define USB0_FIFO6_R            (*((volatile uint32 *)0x40050038))
#define USB0_FIFO7_R            (*((volatile uint32 *)0x4005003C))
#define USB0_DEVCTL_R           (*((volatile uint8 *)0x40050060))
#define USB0_TXFIFOSZ_R         (*((volatile uint8 *)0x40050062))
#define USB0_RXFIFOSZ_R         (*((volatile uint8 *)0x40050063))
#define USB0_TXFIFOADD_R        (*((volatile uint16 *)0x40050064))
#define USB0_RXFIFOADD_R        (*((volatile uint16 *)0x40050066))
#define USB0_CONTIM_R           (*((volatile uint8 *)0x4005007A))
#define USB0_VPLEN_R            (*((volatile uint8 *)0x4005007B))
#define USB0_FSEOF_R            (*((volatile uint8 *)0x4005007D))
#define USB0_LSEOF_R            (*((volatile uint8 *)0x4005007E))
#define USB0_TXFUNCADDR0_R      (*((volatile uint8 *)0x40050080))
#define USB0_TXHUBADDR0_R       (*((volatile uint8 *)0x40050082))
#define USB0_TXHUBPORT0_R       (*((volatile uint8 *)0x40050083))
#define USB0_TXFUNCADDR1_R      (*((volatile uint8 *)0x40050088))
#define USB0_TXHUBADDR1_R       (*((volatile uint8 *)0x4005008A))
#define USB0_TXHUBPORT1_R       (*((volatile uint8 *)0x4005008B))
#define USB0_RXFUNCADDR1_R      (*((volatile uint8 *)0x4005008C))
#define USB0_RXHUBADDR1_R       (*((volatile uint8 *)0x4005008E))
#define USB0_RXHUBPORT1_R       (*((volatile uint8 *)0x4005008F))
#define USB0_TXFUNCADDR2_R      (*((volatile uint8 *)0x40050090))
#define USB0_TXHUBADDR2_R       (*((volatile uint8 *)0x40050092))
#define USB0_TXHUBPORT2_R       (*((volatile uint8 *)0x40050093))
#define USB0_RXFUNCADDR2_R      (*((volatile uint8 *)0x40050094))
#define USB0_RXHUBADDR2_R       (*((volatile uint8 *)0x40050096))
#define USB0_RXHUBPORT2_R       (*((volatile uint8 *)0x40050097))
#define USB0_TXFUNCADDR3_R      (*((volatile uint8 *)0x40050098))
#define USB0_TXHUBADDR3_R       (*((volatile uint8 *)0x4005009A))
#define USB0_TXHUBPORT3_R       (*((volatile uint8 *)0x4005009B))
#define USB0_RXFUNCADDR3_R      (*((volatile uint8 *)0x4005009C))
#define USB0_RXHUBADDR3_R       (*((volatile uint8 *)0x4005009E))
#define USB0_RXHUBPORT3_R       (*((volatile uint8 *)0x4005009F))
#define USB0_TXFUNCADDR4_R      (*((volatile uint8 *)0x400500A0))
#define USB0_TXHUBADDR4_R       (*((volatile uint8 *)0x400500A2))
#define USB0_TXHUBPORT4_R       (*((volatile uint8 *)0x400500A3))
#define USB0_RXFUNCADDR4_R      (*((volatile uint8 *)0x400500A4))
#define USB0_RXHUBADDR4_R       (*((volatile uint8 *)0x400500A6))
#define USB0_RXHUBPORT4_R       (*((volatile uint8 *)0x400500A7))
#define USB0_TXFUNCADDR5_R      (*((volatile uint8 *)0x400500A8))
#define USB0_TXHUBADDR5_R       (*((volatile uint8 *)0x400500AA))
#define USB0_TXHUBPORT5_R       (*((volatile uint8 *)0x400500AB))
#define USB0_RXFUNCADDR5_R      (*((volatile uint8 *)0x400500AC))
#define USB0_RXHUBADDR5_R       (*((volatile uint8 *)0x400500AE))
#define USB0_RXHUBPORT5_R       (*((volatile uint8 *)0x400500AF))
#define USB0_TXFUNCADDR6_R      (*((volatile uint8 *)0x400500B0))
#define USB0_TXHUBADDR6_R       (*((volatile uint8 *)0x400500B2))
#define USB0_TXHUBPORT6_R       (*((volatile uint8 *)0x400500B3))
#define USB0_RXFUNCADDR6_R      (*((volatile uint8 *)0x400500B4))
#define USB0_RXHUBADDR6_R       (*((volatile uint8 *)0x400500B6))
#define USB0_RXHUBPORT6_R       (*((volatile uint8 *)0x400500B7))
#define USB0_TXFUNCADDR7_R      (*((volatile uint8 *)0x400500B8))
#define USB0_TXHUBADDR7_R       (*((volatile uint8 *)0x400500BA))
#define USB0_TXHUBPORT7_R       (*((volatile uint8 *)0x400500BB))
#define USB0_RXFUNCADDR7_R      (*((volatile uint8 *)0x400500BC))
#define USB0_RXHUBADDR7_R       (*((volatile uint8 *)0x400500BE))
#define USB0_RXHUBPORT7_R       (*((volatile uint8 *)0x400500BF))
#define USB0_CSRL0_R            (*((volatile uint8 *)0x40050102))
#define USB0_CSRH0_R            (*((volatile uint8 *)0x40050103))
#define USB0_COUNT0_R           (*((volatile uint8 *)0x40050108))
#define USB0_TYPE0_R            (*((volatile uint8 *)0x4005010A))
#define USB0_NAKLMT_R           (*((volatile uint8 *)0x4005010B))
#define USB0_TXMAXP1_R          (*((volatile uint16 *)0x40050110))
#define USB0_TXCSRL1_R          (*((volatile uint8 *)0x40050112))
#define USB0_TXCSRH1_R          (*((volatile uint8 *)0x40050113))
#define USB0_RXMAXP1_R          (*((volatile uint16 *)0x40050114))
#define USB0_RXCSRL1_R          (*((volatile uint8 *)0x40050116))
#define USB0_RXCSRH1_R          (*((volatile uint8 *)0x40050117))
#define USB0_RXCOUNT1_R         (*((volatile uint16 *)0x40050118))
#define USB0_TXTYPE1_R          (*((volatile uint8 *)0x4005011A))
#define USB0_TXINTERVAL1_R      (*((volatile uint8 *)0x4005011B))
#define USB0_RXTYPE1_R          (*((volatile uint8 *)0x4005011C))
#define USB0_RXINTERVAL1_R      (*((volatile uint8 *)0x4005011D))
#define USB0_TXMAXP2_R          (*((volatile uint16 *)0x40050120))
#define USB0_TXCSRL2_R          (*((volatile uint8 *)0x40050122))
#define USB0_TXCSRH2_R          (*((volatile uint8 *)0x40050123))
#define USB0_RXMAXP2_R          (*((volatile uint16 *)0x40050124))
#define USB0_RXCSRL2_R          (*((volatile uint8 *)0x40050126))
#define USB0_RXCSRH2_R          (*((volatile uint8 *)0x40050127))
#define USB0_RXCOUNT2_R         (*((volatile uint16 *)0x40050128))
#define USB0_TXTYPE2_R          (*((volatile uint8 *)0x4005012A))
#define USB0_TXINTERVAL2_R      (*((volatile uint8 *)0x4005012B))
#define USB0_RXTYPE2_R          (*((volatile uint8 *)0x4005012C))
#define USB0_RXINTERVAL2_R      (*((volatile uint8 *)0x4005012D))
#define USB0_TXMAXP3_R          (*((volatile uint16 *)0x40050130))
#define USB0_TXCSRL3_R          (*((volatile uint8 *)0x40050132))
#define USB0_TXCSRH3_R          (*((volatile uint8 *)0x40050133))
#define USB0_RXMAXP3_R          (*((volatile uint16 *)0x40050134))
#define USB0_RXCSRL3_R          (*((volatile uint8 *)0x40050136))
#define USB0_RXCSRH3_R          (*((volatile uint8 *)0x40050137))
#define USB0_RXCOUNT3_R         (*((volatile uint16 *)0x40050138))
#define USB0_TXTYPE3_R          (*((volatile uint8 *)0x4005013A))
#define USB0_TXINTERVAL3_R      (*((volatile uint8 *)0x4005013B))
#define USB0_RXTYPE3_R          (*((volatile uint8 *)0x4005013C))
#define USB0_RXINTERVAL3_R      (*((volatile uint8 *)0x4005013D))
#define USB0_TXMAXP4_R          (*((volatile uint16 *)0x40050140))
#define USB0_TXCSRL4_R          (*((volatile uint8 *)0x40050142))
#define USB0_TXCSRH4_R          (*((volatile uint8 *)0x40050143))
#define USB0_RXMAXP4_R          (*((volatile uint16 *)0x40050144))
#define USB0_RXCSRL4_R          (*((volatile uint8 *)0x40050146))
#define USB0_RXCSRH4_R          (*((volatile uint8 *)0x40050147))
#define USB0_RXCOUNT4_R         (*((volatile uint16 *)0x40050148))
#define USB0_TXTYPE4_R          (*((volatile uint8 *)0x4005014A))
#define USB0_TXINTERVAL4_R      (*((volatile uint8 *)0x4005014B))
#define USB0_RXTYPE4_R          (*((volatile uint8 *)0x4005014C))
#define USB0_RXINTERVAL4_R      (*((volatile uint8 *)0x4005014D))
#define USB0_TXMAXP5_R          (*((volatile uint16 *)0x40050150))
#define USB0_TXCSRL5_R          (*((volatile uint8 *)0x40050152))
#define USB0_TXCSRH5_R          (*((volatile uint8 *)0x40050153))
#define USB0_RXMAXP5_R          (*((volatile uint16 *)0x40050154))
#define USB0_RXCSRL5_R          (*((volatile uint8 *)0x40050156))
#define USB0_RXCSRH5_R          (*((volatile uint8 *)0x40050157))
#define USB0_RXCOUNT5_R         (*((volatile uint16 *)0x40050158))
#define USB0_TXTYPE5_R          (*((volatile uint8 *)0x4005015A))
#define USB0_TXINTERVAL5_R      (*((volatile uint8 *)0x4005015B))
#define USB0_RXTYPE5_R          (*((volatile uint8 *)0x4005015C))
#define USB0_RXINTERVAL5_R      (*((volatile uint8 *)0x4005015D))
#define USB0_TXMAXP6_R          (*((volatile uint16 *)0x40050160))
#define USB0_TXCSRL6_R          (*((volatile uint8 *)0x40050162))
#define USB0_TXCSRH6_R          (*((volatile uint8 *)0x40050163))
#define USB0_RXMAXP6_R          (*((volatile uint16 *)0x40050164))
#define USB0_RXCSRL6_R          (*((volatile uint8 *)0x40050166))
#define USB0_RXCSRH6_R          (*((volatile uint8 *)0x40050167))
#define USB0_RXCOUNT6_R         (*((volatile uint16 *)0x40050168))
#define USB0_TXTYPE6_R          (*((volatile uint8 *)0x4005016A))
#define USB0_TXINTERVAL6_R      (*((volatile uint8 *)0x4005016B))
#define USB0_RXTYPE6_R          (*((volatile uint8 *)0x4005016C))
#define USB0_RXINTERVAL6_R      (*((volatile uint8 *)0x4005016D))
#define USB0_TXMAXP7_R          (*((volatile uint16 *)0x40050170))
#define USB0_TXCSRL7_R          (*((volatile uint8 *)0x40050172))
#define USB0_TXCSRH7_R          (*((volatile uint8 *)0x40050173))
#define USB0_RXMAXP7_R          (*((volatile uint16 *)0x40050174))
#define USB0_RXCSRL7_R          (*((volatile uint8 *)0x40050176))
#define USB0_RXCSRH7_R          (*((volatile uint8 *)0x40050177))
#define USB0_RXCOUNT7_R         (*((volatile uint16 *)0x40050178))
#define USB0_TXTYPE7_R          (*((volatile uint8 *)0x4005017A))
#define USB0_TXINTERVAL7_R      (*((volatile uint8 *)0x4005017B))
#define USB0_RXTYPE7_R          (*((volatile uint8 *)0x4005017C))
#define USB0_RXINTERVAL7_R      (*((volatile uint8 *)0x4005017D))
#define USB0_RQPKTCOUNT1_R      (*((volatile uint16 *)0x40050304))
#define USB0_RQPKTCOUNT2_R      (*((volatile uint16 *)0x40050308))
#define USB0_RQPKTCOUNT3_R      (*((volatile uint16 *)0x4005030C))
#define USB0_RQPKTCOUNT4_R      (*((volatile uint16 *)0x40050310))
#define USB0_RQPKTCOUNT5_R      (*((volatile uint16 *)0x40050314))
#define USB0_RQPKTCOUNT6_R      (*((volatile uint16 *)0x40050318))
#define USB0_RQPKTCOUNT7_R      (*((volatile uint16 *)0x4005031C))
#define USB0_RXDPKTBUFDIS_R     (*((volatile uint16 *)0x40050340))
#define USB0_TXDPKTBUFDIS_R     (*((volatile uint16 *)0x40050342))
#define USB0_EPC_R              (*((volatile uint32 *)0x40050400))
#define USB0_EPCRIS_R           (*((volatile uint32 *)0x40050404))
#define USB0_EPCIM_R            (*((volatile uint32 *)0x40050408))
#define USB0_EPCISC_R           (*((volatile uint32 *)0x4005040C))
#define USB0_DRRIS_R            (*((volatile uint32 *)0x40050410))
#define USB0_DRIM_R             (*((volatile uint32 *)0x40050414))
#define USB0_DRISC_R            (*((volatile uint32 *)0x40050418))
#define USB0_GPCS_R             (*((volatile uint32 *)0x4005041C))
#define USB0_VDC_R              (*((volatile uint32 *)0x40050430))
#define USB0_VDCRIS_R           (*((volatile uint32 *)0x40050434))
#define USB0_VDCIM_R            (*((volatile uint32 *)0x40050438))
#define USB0_VDCISC_R           (*((volatile uint32 *)0x4005043C))
#define USB0_IDVRIS_R           (*((volatile uint32 *)0x40050444))
#define USB0_IDVIM_R            (*((volatile uint32 *)0x40050448))
#define USB0_IDVISC_R           (*((volatile uint32 *)0x4005044C))
#define USB0_DMASEL_R           (*((volatile uint32 *)0x40050450))
#define USB0_PP_R               (*((volatile uint32 *)0x40050FC0))

//*****************************************************************************
//
// GPIO registers (PORTA AHB)
//
//*****************************************************************************
#define GPIO_PORTA_AHB_DATA_BITS_R                                            \
		((volatile uint32 *)0x40058000)
#define GPIO_PORTA_AHB_DATA_R   (*((volatile uint32 *)0x400583FC))
#define GPIO_PORTA_AHB_DIR_R    (*((volatile uint32 *)0x40058400))
#define GPIO_PORTA_AHB_IS_R     (*((volatile uint32 *)0x40058404))
#define GPIO_PORTA_AHB_IBE_R    (*((volatile uint32 *)0x40058408))
#define GPIO_PORTA_AHB_IEV_R    (*((volatile uint32 *)0x4005840C))
#define GPIO_PORTA_AHB_IM_R     (*((volatile uint32 *)0x40058410))
#define GPIO_PORTA_AHB_RIS_R    (*((volatile uint32 *)0x40058414))
#define GPIO_PORTA_AHB_MIS_R    (*((volatile uint32 *)0x40058418))
#define GPIO_PORTA_AHB_ICR_R    (*((volatile uint32 *)0x4005841C))
#define GPIO_PORTA_AHB_AFSEL_R  (*((volatile uint32 *)0x40058420))
#define GPIO_PORTA_AHB_DR2R_R   (*((volatile uint32 *)0x40058500))
#define GPIO_PORTA_AHB_DR4R_R   (*((volatile uint32 *)0x40058504))
#define GPIO_PORTA_AHB_DR8R_R   (*((volatile uint32 *)0x40058508))
#define GPIO_PORTA_AHB_ODR_R    (*((volatile uint32 *)0x4005850C))
#define GPIO_PORTA_AHB_PUR_R    (*((volatile uint32 *)0x40058510))
#define GPIO_PORTA_AHB_PDR_R    (*((volatile uint32 *)0x40058514))
#define GPIO_PORTA_AHB_SLR_R    (*((volatile uint32 *)0x40058518))
#define GPIO_PORTA_AHB_DEN_R    (*((volatile uint32 *)0x4005851C))
#define GPIO_PORTA_AHB_LOCK_R   (*((volatile uint32 *)0x40058520))
#define GPIO_PORTA_AHB_CR_R     (*((volatile uint32 *)0x40058524))
#define GPIO_PORTA_AHB_AMSEL_R  (*((volatile uint32 *)0x40058528))
#define GPIO_PORTA_AHB_PCTL_R   (*((volatile uint32 *)0x4005852C))
#define GPIO_PORTA_AHB_ADCCTL_R (*((volatile uint32 *)0x40058530))
#define GPIO_PORTA_AHB_DMACTL_R (*((volatile uint32 *)0x40058534))

//*****************************************************************************
//
// GPIO registers (PORTB AHB)
//
//*****************************************************************************
#define GPIO_PORTB_AHB_DATA_BITS_R                                            \
		((volatile uint32 *)0x40059000)
#define GPIO_PORTB_AHB_DATA_R   (*((volatile uint32 *)0x400593FC))
#define GPIO_PORTB_AHB_DIR_R    (*((volatile uint32 *)0x40059400))
#define GPIO_PORTB_AHB_IS_R     (*((volatile uint32 *)0x40059404))
#define GPIO_PORTB_AHB_IBE_R    (*((volatile uint32 *)0x40059408))
#define GPIO_PORTB_AHB_IEV_R    (*((volatile uint32 *)0x4005940C))
#define GPIO_PORTB_AHB_IM_R     (*((volatile uint32 *)0x40059410))
#define GPIO_PORTB_AHB_RIS_R    (*((volatile uint32 *)0x40059414))
#define GPIO_PORTB_AHB_MIS_R    (*((volatile uint32 *)0x40059418))
#define GPIO_PORTB_AHB_ICR_R    (*((volatile uint32 *)0x4005941C))
#define GPIO_PORTB_AHB_AFSEL_R  (*((volatile uint32 *)0x40059420))
#define GPIO_PORTB_AHB_DR2R_R   (*((volatile uint32 *)0x40059500))
#define GPIO_PORTB_AHB_DR4R_R   (*((volatile uint32 *)0x40059504))
#define GPIO_PORTB_AHB_DR8R_R   (*((volatile uint32 *)0x40059508))
#define GPIO_PORTB_AHB_ODR_R    (*((volatile uint32 *)0x4005950C))
#define GPIO_PORTB_AHB_PUR_R    (*((volatile uint32 *)0x40059510))
#define GPIO_PORTB_AHB_PDR_R    (*((volatile uint32 *)0x40059514))
#define GPIO_PORTB_AHB_SLR_R    (*((volatile uint32 *)0x40059518))
#define GPIO_PORTB_AHB_DEN_R    (*((volatile uint32 *)0x4005951C))
#define GPIO_PORTB_AHB_LOCK_R   (*((volatile uint32 *)0x40059520))
#define GPIO_PORTB_AHB_CR_R     (*((volatile uint32 *)0x40059524))
#define GPIO_PORTB_AHB_AMSEL_R  (*((volatile uint32 *)0x40059528))
#define GPIO_PORTB_AHB_PCTL_R   (*((volatile uint32 *)0x4005952C))
#define GPIO_PORTB_AHB_ADCCTL_R (*((volatile uint32 *)0x40059530))
#define GPIO_PORTB_AHB_DMACTL_R (*((volatile uint32 *)0x40059534))

//*****************************************************************************
//
// GPIO registers (PORTC AHB)
//
//*****************************************************************************
#define GPIO_PORTC_AHB_DATA_BITS_R                                            \
		((volatile uint32 *)0x4005A000)
#define GPIO_PORTC_AHB_DATA_R   (*((volatile uint32 *)0x4005A3FC))
#define GPIO_PORTC_AHB_DIR_R    (*((volatile uint32 *)0x4005A400))
#define GPIO_PORTC_AHB_IS_R     (*((volatile uint32 *)0x4005A404))
#define GPIO_PORTC_AHB_IBE_R    (*((volatile uint32 *)0x4005A408))
#define GPIO_PORTC_AHB_IEV_R    (*((volatile uint32 *)0x4005A40C))
#define GPIO_PORTC_AHB_IM_R     (*((volatile uint32 *)0x4005A410))
#define GPIO_PORTC_AHB_RIS_R    (*((volatile uint32 *)0x4005A414))
#define GPIO_PORTC_AHB_MIS_R    (*((volatile uint32 *)0x4005A418))
#define GPIO_PORTC_AHB_ICR_R    (*((volatile uint32 *)0x4005A41C))
#define GPIO_PORTC_AHB_AFSEL_R  (*((volatile uint32 *)0x4005A420))
#define GPIO_PORTC_AHB_DR2R_R   (*((volatile uint32 *)0x4005A500))
#define GPIO_PORTC_AHB_DR4R_R   (*((volatile uint32 *)0x4005A504))
#define GPIO_PORTC_AHB_DR8R_R   (*((volatile uint32 *)0x4005A508))
#define GPIO_PORTC_AHB_ODR_R    (*((volatile uint32 *)0x4005A50C))
#define GPIO_PORTC_AHB_PUR_R    (*((volatile uint32 *)0x4005A510))
#define GPIO_PORTC_AHB_PDR_R    (*((volatile uint32 *)0x4005A514))
#define GPIO_PORTC_AHB_SLR_R    (*((volatile uint32 *)0x4005A518))
#define GPIO_PORTC_AHB_DEN_R    (*((volatile uint32 *)0x4005A51C))
#define GPIO_PORTC_AHB_LOCK_R   (*((volatile uint32 *)0x4005A520))
#define GPIO_PORTC_AHB_CR_R     (*((volatile uint32 *)0x4005A524))
#define GPIO_PORTC_AHB_AMSEL_R  (*((volatile uint32 *)0x4005A528))
#define GPIO_PORTC_AHB_PCTL_R   (*((volatile uint32 *)0x4005A52C))
#define GPIO_PORTC_AHB_ADCCTL_R (*((volatile uint32 *)0x4005A530))
#define GPIO_PORTC_AHB_DMACTL_R (*((volatile uint32 *)0x4005A534))

//*****************************************************************************
//
// GPIO registers (PORTD AHB)
//
//*****************************************************************************
#define GPIO_PORTD_AHB_DATA_BITS_R                                            \
		((volatile uint32 *)0x4005B000)
#define GPIO_PORTD_AHB_DATA_R   (*((volatile uint32 *)0x4005B3FC))
#define GPIO_PORTD_AHB_DIR_R    (*((volatile uint32 *)0x4005B400))
#define GPIO_PORTD_AHB_IS_R     (*((volatile uint32 *)0x4005B404))
#define GPIO_PORTD_AHB_IBE_R    (*((volatile uint32 *)0x4005B408))
#define GPIO_PORTD_AHB_IEV_R    (*((volatile uint32 *)0x4005B40C))
#define GPIO_PORTD_AHB_IM_R     (*((volatile uint32 *)0x4005B410))
#define GPIO_PORTD_AHB_RIS_R    (*((volatile uint32 *)0x4005B414))
#define GPIO_PORTD_AHB_MIS_R    (*((volatile uint32 *)0x4005B418))
#define GPIO_PORTD_AHB_ICR_R    (*((volatile uint32 *)0x4005B41C))
#define GPIO_PORTD_AHB_AFSEL_R  (*((volatile uint32 *)0x4005B420))
#define GPIO_PORTD_AHB_DR2R_R   (*((volatile uint32 *)0x4005B500))
#define GPIO_PORTD_AHB_DR4R_R   (*((volatile uint32 *)0x4005B504))
#define GPIO_PORTD_AHB_DR8R_R   (*((volatile uint32 *)0x4005B508))
#define GPIO_PORTD_AHB_ODR_R    (*((volatile uint32 *)0x4005B50C))
#define GPIO_PORTD_AHB_PUR_R    (*((volatile uint32 *)0x4005B510))
#define GPIO_PORTD_AHB_PDR_R    (*((volatile uint32 *)0x4005B514))
#define GPIO_PORTD_AHB_SLR_R    (*((volatile uint32 *)0x4005B518))
#define GPIO_PORTD_AHB_DEN_R    (*((volatile uint32 *)0x4005B51C))
#define GPIO_PORTD_AHB_LOCK_R   (*((volatile uint32 *)0x4005B520))
#define GPIO_PORTD_AHB_CR_R     (*((volatile uint32 *)0x4005B524))
#define GPIO_PORTD_AHB_AMSEL_R  (*((volatile uint32 *)0x4005B528))
#define GPIO_PORTD_AHB_PCTL_R   (*((volatile uint32 *)0x4005B52C))
#define GPIO_PORTD_AHB_ADCCTL_R (*((volatile uint32 *)0x4005B530))
#define GPIO_PORTD_AHB_DMACTL_R (*((volatile uint32 *)0x4005B534))

//*****************************************************************************
//
// GPIO registers (PORTE AHB)
//
//*****************************************************************************
#define GPIO_PORTE_AHB_DATA_BITS_R                                            \
		((volatile uint32 *)0x4005C000)
#define GPIO_PORTE_AHB_DATA_R   (*((volatile uint32 *)0x4005C3FC))
#define GPIO_PORTE_AHB_DIR_R    (*((volatile uint32 *)0x4005C400))
#define GPIO_PORTE_AHB_IS_R     (*((volatile uint32 *)0x4005C404))
#define GPIO_PORTE_AHB_IBE_R    (*((volatile uint32 *)0x4005C408))
#define GPIO_PORTE_AHB_IEV_R    (*((volatile uint32 *)0x4005C40C))
#define GPIO_PORTE_AHB_IM_R     (*((volatile uint32 *)0x4005C410))
#define GPIO_PORTE_AHB_RIS_R    (*((volatile uint32 *)0x4005C414))
#define GPIO_PORTE_AHB_MIS_R    (*((volatile uint32 *)0x4005C418))
#define GPIO_PORTE_AHB_ICR_R    (*((volatile uint32 *)0x4005C41C))
#define GPIO_PORTE_AHB_AFSEL_R  (*((volatile uint32 *)0x4005C420))
#define GPIO_PORTE_AHB_DR2R_R   (*((volatile uint32 *)0x4005C500))
#define GPIO_PORTE_AHB_DR4R_R   (*((volatile uint32 *)0x4005C504))
#define GPIO_PORTE_AHB_DR8R_R   (*((volatile uint32 *)0x4005C508))
#define GPIO_PORTE_AHB_ODR_R    (*((volatile uint32 *)0x4005C50C))
#define GPIO_PORTE_AHB_PUR_R    (*((volatile uint32 *)0x4005C510))
#define GPIO_PORTE_AHB_PDR_R    (*((volatile uint32 *)0x4005C514))
#define GPIO_PORTE_AHB_SLR_R    (*((volatile uint32 *)0x4005C518))
#define GPIO_PORTE_AHB_DEN_R    (*((volatile uint32 *)0x4005C51C))
#define GPIO_PORTE_AHB_LOCK_R   (*((volatile uint32 *)0x4005C520))
#define GPIO_PORTE_AHB_CR_R     (*((volatile uint32 *)0x4005C524))
#define GPIO_PORTE_AHB_AMSEL_R  (*((volatile uint32 *)0x4005C528))
#define GPIO_PORTE_AHB_PCTL_R   (*((volatile uint32 *)0x4005C52C))
#define GPIO_PORTE_AHB_ADCCTL_R (*((volatile uint32 *)0x4005C530))
#define GPIO_PORTE_AHB_DMACTL_R (*((volatile uint32 *)0x4005C534))

//*****************************************************************************
//
// GPIO registers (PORTF AHB)
//
//*****************************************************************************
#define GPIO_PORTF_AHB_DATA_BITS_R                                            \
		((volatile uint32 *)0x4005D000)
#define GPIO_PORTF_AHB_DATA_R   (*((volatile uint32 *)0x4005D3FC))
#define GPIO_PORTF_AHB_DIR_R    (*((volatile uint32 *)0x4005D400))
#define GPIO_PORTF_AHB_IS_R     (*((volatile uint32 *)0x4005D404))
#define GPIO_PORTF_AHB_IBE_R    (*((volatile uint32 *)0x4005D408))
#define GPIO_PORTF_AHB_IEV_R    (*((volatile uint32 *)0x4005D40C))
#define GPIO_PORTF_AHB_IM_R     (*((volatile uint32 *)0x4005D410))
#define GPIO_PORTF_AHB_RIS_R    (*((volatile uint32 *)0x4005D414))
#define GPIO_PORTF_AHB_MIS_R    (*((volatile uint32 *)0x4005D418))
#define GPIO_PORTF_AHB_ICR_R    (*((volatile uint32 *)0x4005D41C))
#define GPIO_PORTF_AHB_AFSEL_R  (*((volatile uint32 *)0x4005D420))
#define GPIO_PORTF_AHB_DR2R_R   (*((volatile uint32 *)0x4005D500))
#define GPIO_PORTF_AHB_DR4R_R   (*((volatile uint32 *)0x4005D504))
#define GPIO_PORTF_AHB_DR8R_R   (*((volatile uint32 *)0x4005D508))
#define GPIO_PORTF_AHB_ODR_R    (*((volatile uint32 *)0x4005D50C))
#define GPIO_PORTF_AHB_PUR_R    (*((volatile uint32 *)0x4005D510))
#define GPIO_PORTF_AHB_PDR_R    (*((volatile uint32 *)0x4005D514))
#define GPIO_PORTF_AHB_SLR_R    (*((volatile uint32 *)0x4005D518))
#define GPIO_PORTF_AHB_DEN_R    (*((volatile uint32 *)0x4005D51C))
#define GPIO_PORTF_AHB_LOCK_R   (*((volatile uint32 *)0x4005D520))
#define GPIO_PORTF_AHB_CR_R     (*((volatile uint32 *)0x4005D524))
#define GPIO_PORTF_AHB_AMSEL_R  (*((volatile uint32 *)0x4005D528))
#define GPIO_PORTF_AHB_PCTL_R   (*((volatile uint32 *)0x4005D52C))
#define GPIO_PORTF_AHB_ADCCTL_R (*((volatile uint32 *)0x4005D530))
#define GPIO_PORTF_AHB_DMACTL_R (*((volatile uint32 *)0x4005D534))

//*****************************************************************************
//
// EEPROM registers (EEPROM)
//
//*****************************************************************************
#define EEPROM_EESIZE_R         (*((volatile uint32 *)0x400AF000))
#define EEPROM_EEBLOCK_R        (*((volatile uint32 *)0x400AF004))
#define EEPROM_EEOFFSET_R       (*((volatile uint32 *)0x400AF008))
#define EEPROM_EERDWR_R         (*((volatile uint32 *)0x400AF010))
#define EEPROM_EERDWRINC_R      (*((volatile uint32 *)0x400AF014))
#define EEPROM_EEDONE_R         (*((volatile uint32 *)0x400AF018))
#define EEPROM_EESUPP_R         (*((volatile uint32 *)0x400AF01C))
#define EEPROM_EEUNLOCK_R       (*((volatile uint32 *)0x400AF020))
#define EEPROM_EEPROT_R         (*((volatile uint32 *)0x400AF030))
#define EEPROM_EEPASS0_R        (*((volatile uint32 *)0x400AF034))
#define EEPROM_EEPASS1_R        (*((volatile uint32 *)0x400AF038))
#define EEPROM_EEPASS2_R        (*((volatile uint32 *)0x400AF03C))
#define EEPROM_EEINT_R          (*((volatile uint32 *)0x400AF040))
#define EEPROM_EEHIDE_R         (*((volatile uint32 *)0x400AF050))
#define EEPROM_EEDBGME_R        (*((volatile uint32 *)0x400AF080))
#define EEPROM_PP_R             (*((volatile uint32 *)0x400AFFC0))

//*****************************************************************************
//
// System Exception Module registers (SYSEXC)
//
//*****************************************************************************
#define SYSEXC_RIS_R            (*((volatile uint32 *)0x400F9000))
#define SYSEXC_IM_R             (*((volatile uint32 *)0x400F9004))
#define SYSEXC_MIS_R            (*((volatile uint32 *)0x400F9008))
#define SYSEXC_IC_R             (*((volatile uint32 *)0x400F900C))

//*****************************************************************************
//
// Hibernation module registers (HIB)
//
//*****************************************************************************
#define HIB_RTCC_R              (*((volatile uint32 *)0x400FC000))
#define HIB_RTCM0_R             (*((volatile uint32 *)0x400FC004))
#define HIB_RTCLD_R             (*((volatile uint32 *)0x400FC00C))
#define HIB_CTL_R               (*((volatile uint32 *)0x400FC010))
#define HIB_IM_R                (*((volatile uint32 *)0x400FC014))
#define HIB_RIS_R               (*((volatile uint32 *)0x400FC018))
#define HIB_MIS_R               (*((volatile uint32 *)0x400FC01C))
#define HIB_IC_R                (*((volatile uint32 *)0x400FC020))
#define HIB_RTCT_R              (*((volatile uint32 *)0x400FC024))
#define HIB_RTCSS_R             (*((volatile uint32 *)0x400FC028))
#define HIB_DATA_R              (*((volatile uint32 *)0x400FC030))

//*****************************************************************************
//
// FLASH registers (FLASH CTRL)
//
//*****************************************************************************
#define FLASH_FMA_R             (*((volatile uint32 *)0x400FD000))
#define FLASH_FMD_R             (*((volatile uint32 *)0x400FD004))
#define FLASH_FMC_R             (*((volatile uint32 *)0x400FD008))
#define FLASH_FCRIS_R           (*((volatile uint32 *)0x400FD00C))
#define FLASH_FCIM_R            (*((volatile uint32 *)0x400FD010))
#define FLASH_FCMISC_R          (*((volatile uint32 *)0x400FD014))
#define FLASH_FMC2_R            (*((volatile uint32 *)0x400FD020))
#define FLASH_FWBVAL_R          (*((volatile uint32 *)0x400FD030))
#define FLASH_FWBN_R            (*((volatile uint32 *)0x400FD100))
#define FLASH_FSIZE_R           (*((volatile uint32 *)0x400FDFC0))
#define FLASH_SSIZE_R           (*((volatile uint32 *)0x400FDFC4))
#define FLASH_ROMSWMAP_R        (*((volatile uint32 *)0x400FDFCC))
#define FLASH_RMCTL_R           (*((volatile uint32 *)0x400FE0F0))
#define FLASH_BOOTCFG_R         (*((volatile uint32 *)0x400FE1D0))
#define FLASH_USERREG0_R        (*((volatile uint32 *)0x400FE1E0))
#define FLASH_USERREG1_R        (*((volatile uint32 *)0x400FE1E4))
#define FLASH_USERREG2_R        (*((volatile uint32 *)0x400FE1E8))
#define FLASH_USERREG3_R        (*((volatile uint32 *)0x400FE1EC))
#define FLASH_FMPRE0_R          (*((volatile uint32 *)0x400FE200))
#define FLASH_FMPRE1_R          (*((volatile uint32 *)0x400FE204))
#define FLASH_FMPRE2_R          (*((volatile uint32 *)0x400FE208))
#define FLASH_FMPRE3_R          (*((volatile uint32 *)0x400FE20C))
#define FLASH_FMPPE0_R          (*((volatile uint32 *)0x400FE400))
#define FLASH_FMPPE1_R          (*((volatile uint32 *)0x400FE404))
#define FLASH_FMPPE2_R          (*((volatile uint32 *)0x400FE408))
#define FLASH_FMPPE3_R          (*((volatile uint32 *)0x400FE40C))

//*****************************************************************************
//
// System Control registers (SYSCTL)
//
//*****************************************************************************
#define SYSCTL_DID0_R           (*((volatile uint32 *)0x400FE000))
#define SYSCTL_DID1_R           (*((volatile uint32 *)0x400FE004))
#define SYSCTL_DC0_R            (*((volatile uint32 *)0x400FE008))
#define SYSCTL_DC1_R            (*((volatile uint32 *)0x400FE010))
#define SYSCTL_DC2_R            (*((volatile uint32 *)0x400FE014))
#define SYSCTL_DC3_R            (*((volatile uint32 *)0x400FE018))
#define SYSCTL_DC4_R            (*((volatile uint32 *)0x400FE01C))
#define SYSCTL_DC5_R            (*((volatile uint32 *)0x400FE020))
#define SYSCTL_DC6_R            (*((volatile uint32 *)0x400FE024))
#define SYSCTL_DC7_R            (*((volatile uint32 *)0x400FE028))
#define SYSCTL_DC8_R            (*((volatile uint32 *)0x400FE02C))
#define SYSCTL_PBORCTL_R        (*((volatile uint32 *)0x400FE030))
#define SYSCTL_SRCR0_R          (*((volatile uint32 *)0x400FE040))
#define SYSCTL_SRCR1_R          (*((volatile uint32 *)0x400FE044))
#define SYSCTL_SRCR2_R          (*((volatile uint32 *)0x400FE048))
#define SYSCTL_RIS_R            (*((volatile uint32 *)0x400FE050))
#define SYSCTL_IMC_R            (*((volatile uint32 *)0x400FE054))
#define SYSCTL_MISC_R           (*((volatile uint32 *)0x400FE058))
#define SYSCTL_RESC_R           (*((volatile uint32 *)0x400FE05C))
#define SYSCTL_RCC_R            (*((volatile uint32 *)0x400FE060))
#define SYSCTL_GPIOHBCTL_R      (*((volatile uint32 *)0x400FE06C))
#define SYSCTL_RCC2_R           (*((volatile uint32 *)0x400FE070))
#define SYSCTL_MOSCCTL_R        (*((volatile uint32 *)0x400FE07C))
#define SYSCTL_RCGC0_R          (*((volatile uint32 *)0x400FE100))
#define SYSCTL_RCGC1_R          (*((volatile uint32 *)0x400FE104))
#define SYSCTL_RCGC2_R          (*((volatile uint32 *)0x400FE108))
#define SYSCTL_SCGC0_R          (*((volatile uint32 *)0x400FE110))
#define SYSCTL_SCGC1_R          (*((volatile uint32 *)0x400FE114))
#define SYSCTL_SCGC2_R          (*((volatile uint32 *)0x400FE118))
#define SYSCTL_DCGC0_R          (*((volatile uint32 *)0x400FE120))
#define SYSCTL_DCGC1_R          (*((volatile uint32 *)0x400FE124))
#define SYSCTL_DCGC2_R          (*((volatile uint32 *)0x400FE128))
#define SYSCTL_DSLPCLKCFG_R     (*((volatile uint32 *)0x400FE144))
#define SYSCTL_SYSPROP_R        (*((volatile uint32 *)0x400FE14C))
#define SYSCTL_PIOSCCAL_R       (*((volatile uint32 *)0x400FE150))
#define SYSCTL_PIOSCSTAT_R      (*((volatile uint32 *)0x400FE154))
#define SYSCTL_PLLFREQ0_R       (*((volatile uint32 *)0x400FE160))
#define SYSCTL_PLLFREQ1_R       (*((volatile uint32 *)0x400FE164))
#define SYSCTL_PLLSTAT_R        (*((volatile uint32 *)0x400FE168))
#define SYSCTL_SLPPWRCFG_R      (*((volatile uint32 *)0x400FE188))
#define SYSCTL_DSLPPWRCFG_R     (*((volatile uint32 *)0x400FE18C))
#define SYSCTL_DC9_R            (*((volatile uint32 *)0x400FE190))
#define SYSCTL_NVMSTAT_R        (*((volatile uint32 *)0x400FE1A0))
#define SYSCTL_LDOSPCTL_R       (*((volatile uint32 *)0x400FE1B4))
#define SYSCTL_LDODPCTL_R       (*((volatile uint32 *)0x400FE1BC))
#define SYSCTL_PPWD_R           (*((volatile uint32 *)0x400FE300))
#define SYSCTL_PPTIMER_R        (*((volatile uint32 *)0x400FE304))
#define SYSCTL_PPGPIO_R         (*((volatile uint32 *)0x400FE308))
#define SYSCTL_PPDMA_R          (*((volatile uint32 *)0x400FE30C))
#define SYSCTL_PPHIB_R          (*((volatile uint32 *)0x400FE314))
#define SYSCTL_PPUART_R         (*((volatile uint32 *)0x400FE318))
#define SYSCTL_PPSSI_R          (*((volatile uint32 *)0x400FE31C))
#define SYSCTL_PPI2C_R          (*((volatile uint32 *)0x400FE320))
#define SYSCTL_PPUSB_R          (*((volatile uint32 *)0x400FE328))
#define SYSCTL_PPCAN_R          (*((volatile uint32 *)0x400FE334))
#define SYSCTL_PPADC_R          (*((volatile uint32 *)0x400FE338))
#define SYSCTL_PPACMP_R         (*((volatile uint32 *)0x400FE33C))
#define SYSCTL_PPPWM_R          (*((volatile uint32 *)0x400FE340))
#define SYSCTL_PPQEI_R          (*((volatile uint32 *)0x400FE344))
#define SYSCTL_PPEEPROM_R       (*((volatile uint32 *)0x400FE358))
#define SYSCTL_PPWTIMER_R       (*((volatile uint32 *)0x400FE35C))
#define SYSCTL_SRWD_R           (*((volatile uint32 *)0x400FE500))
#define SYSCTL_SRTIMER_R        (*((volatile uint32 *)0x400FE504))
#define SYSCTL_SRGPIO_R         (*((volatile uint32 *)0x400FE508))
#define SYSCTL_SRDMA_R          (*((volatile uint32 *)0x400FE50C))
#define SYSCTL_SRHIB_R          (*((volatile uint32 *)0x400FE514))
#define SYSCTL_SRUART_R         (*((volatile uint32 *)0x400FE518))
#define SYSCTL_SRSSI_R          (*((volatile uint32 *)0x400FE51C))
#define SYSCTL_SRI2C_R          (*((volatile uint32 *)0x400FE520))
#define SYSCTL_SRUSB_R          (*((volatile uint32 *)0x400FE528))
#define SYSCTL_SRCAN_R          (*((volatile uint32 *)0x400FE534))
#define SYSCTL_SRADC_R          (*((volatile uint32 *)0x400FE538))
#define SYSCTL_SRACMP_R         (*((volatile uint32 *)0x400FE53C))
#define SYSCTL_SRPWM_R          (*((volatile uint32 *)0x400FE540))
#define SYSCTL_SRQEI_R          (*((volatile uint32 *)0x400FE544))
#define SYSCTL_SREEPROM_R       (*((volatile uint32 *)0x400FE558))
#define SYSCTL_SRWTIMER_R       (*((volatile uint32 *)0x400FE55C))
#define SYSCTL_RCGCWD_R         (*((volatile uint32 *)0x400FE600))
#define SYSCTL_RCGCTIMER_R      (*((volatile uint32 *)0x400FE604))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32 *)0x400FE608))
#define SYSCTL_RCGCDMA_R        (*((volatile uint32 *)0x400FE60C))
#define SYSCTL_RCGCHIB_R        (*((volatile uint32 *)0x400FE614))
#define SYSCTL_RCGCUART_R       (*((volatile uint32 *)0x400FE618))
#define SYSCTL_RCGCSSI_R        (*((volatile uint32 *)0x400FE61C))
#define SYSCTL_RCGCI2C_R        (*((volatile uint32 *)0x400FE620))
#define SYSCTL_RCGCUSB_R        (*((volatile uint32 *)0x400FE628))
#define SYSCTL_RCGCCAN_R        (*((volatile uint32 *)0x400FE634))
#define SYSCTL_RCGCADC_R        (*((volatile uint32 *)0x400FE638))
#define SYSCTL_RCGCACMP_R       (*((volatile uint32 *)0x400FE63C))
#define SYSCTL_RCGCPWM_R        (*((volatile uint32 *)0x400FE640))
#define SYSCTL_RCGCQEI_R        (*((volatile uint32 *)0x400FE644))
#define SYSCTL_RCGCEEPROM_R     (*((volatile uint32 *)0x400FE658))
#define SYSCTL_RCGCWTIMER_R     (*((volatile uint32 *)0x400FE65C))
#define SYSCTL_SCGCWD_R         (*((volatile uint32 *)0x400FE700))
#define SYSCTL_SCGCTIMER_R      (*((volatile uint32 *)0x400FE704))
#define SYSCTL_SCGCGPIO_R       (*((volatile uint32 *)0x400FE708))
#define SYSCTL_SCGCDMA_R        (*((volatile uint32 *)0x400FE70C))
#define SYSCTL_SCGCHIB_R        (*((volatile uint32 *)0x400FE714))
#define SYSCTL_SCGCUART_R       (*((volatile uint32 *)0x400FE718))
#define SYSCTL_SCGCSSI_R        (*((volatile uint32 *)0x400FE71C))
#define SYSCTL_SCGCI2C_R        (*((volatile uint32 *)0x400FE720))
#define SYSCTL_SCGCUSB_R        (*((volatile uint32 *)0x400FE728))
#define SYSCTL_SCGCCAN_R        (*((volatile uint32 *)0x400FE734))
#define SYSCTL_SCGCADC_R        (*((volatile uint32 *)0x400FE738))
#define SYSCTL_SCGCACMP_R       (*((volatile uint32 *)0x400FE73C))
#define SYSCTL_SCGCPWM_R        (*((volatile uint32 *)0x400FE740))
#define SYSCTL_SCGCQEI_R        (*((volatile uint32 *)0x400FE744))
#define SYSCTL_SCGCEEPROM_R     (*((volatile uint32 *)0x400FE758))
#define SYSCTL_SCGCWTIMER_R     (*((volatile uint32 *)0x400FE75C))
#define SYSCTL_DCGCWD_R         (*((volatile uint32 *)0x400FE800))
#define SYSCTL_DCGCTIMER_R      (*((volatile uint32 *)0x400FE804))
#define SYSCTL_DCGCGPIO_R       (*((volatile uint32 *)0x400FE808))
#define SYSCTL_DCGCDMA_R        (*((volatile uint32 *)0x400FE80C))
#define SYSCTL_DCGCHIB_R        (*((volatile uint32 *)0x400FE814))
#define SYSCTL_DCGCUART_R       (*((volatile uint32 *)0x400FE818))
#define SYSCTL_DCGCSSI_R        (*((volatile uint32 *)0x400FE81C))
#define SYSCTL_DCGCI2C_R        (*((volatile uint32 *)0x400FE820))
#define SYSCTL_DCGCUSB_R        (*((volatile uint32 *)0x400FE828))
#define SYSCTL_DCGCCAN_R        (*((volatile uint32 *)0x400FE834))
#define SYSCTL_DCGCADC_R        (*((volatile uint32 *)0x400FE838))
#define SYSCTL_DCGCACMP_R       (*((volatile uint32 *)0x400FE83C))
#define SYSCTL_DCGCPWM_R        (*((volatile uint32 *)0x400FE840))
#define SYSCTL_DCGCQEI_R        (*((volatile uint32 *)0x400FE844))
#define SYSCTL_DCGCEEPROM_R     (*((volatile uint32 *)0x400FE858))
#define SYSCTL_DCGCWTIMER_R     (*((volatile uint32 *)0x400FE85C))
#define SYSCTL_PRWD_R           (*((volatile uint32 *)0x400FEA00))
#define SYSCTL_PRTIMER_R        (*((volatile uint32 *)0x400FEA04))
#define SYSCTL_PRGPIO_R         (*((volatile uint32 *)0x400FEA08))
#define SYSCTL_PRDMA_R          (*((volatile uint32 *)0x400FEA0C))
#define SYSCTL_PRHIB_R          (*((volatile uint32 *)0x400FEA14))
#define SYSCTL_PRUART_R         (*((volatile uint32 *)0x400FEA18))
#define SYSCTL_PRSSI_R          (*((volatile uint32 *)0x400FEA1C))
#define SYSCTL_PRI2C_R          (*((volatile uint32 *)0x400FEA20))
#define SYSCTL_PRUSB_R          (*((volatile uint32 *)0x400FEA28))
#define SYSCTL_PRCAN_R          (*((volatile uint32 *)0x400FEA34))
#define SYSCTL_PRADC_R          (*((volatile uint32 *)0x400FEA38))
#define SYSCTL_PRACMP_R         (*((volatile uint32 *)0x400FEA3C))
#define SYSCTL_PRPWM_R          (*((volatile uint32 *)0x400FEA40))
#define SYSCTL_PRQEI_R          (*((volatile uint32 *)0x400FEA44))
#define SYSCTL_PREEPROM_R       (*((volatile uint32 *)0x400FEA58))
#define SYSCTL_PRWTIMER_R       (*((volatile uint32 *)0x400FEA5C))

//*****************************************************************************
//
// Micro Direct Memory Access registers (UDMA)
//
//*****************************************************************************
#define UDMA_STAT_R             (*((volatile uint32 *)0x400FF000))
#define UDMA_CFG_R              (*((volatile uint32 *)0x400FF004))
#define UDMA_CTLBASE_R          (*((volatile uint32 *)0x400FF008))
#define UDMA_ALTBASE_R          (*((volatile uint32 *)0x400FF00C))
#define UDMA_WAITSTAT_R         (*((volatile uint32 *)0x400FF010))
#define UDMA_SWREQ_R            (*((volatile uint32 *)0x400FF014))
#define UDMA_USEBURSTSET_R      (*((volatile uint32 *)0x400FF018))
#define UDMA_USEBURSTCLR_R      (*((volatile uint32 *)0x400FF01C))
#define UDMA_REQMASKSET_R       (*((volatile uint32 *)0x400FF020))
#define UDMA_REQMASKCLR_R       (*((volatile uint32 *)0x400FF024))
#define UDMA_ENASET_R           (*((volatile uint32 *)0x400FF028))
#define UDMA_ENACLR_R           (*((volatile uint32 *)0x400FF02C))
#define UDMA_ALTSET_R           (*((volatile uint32 *)0x400FF030))
#define UDMA_ALTCLR_R           (*((volatile uint32 *)0x400FF034))
#define UDMA_PRIOSET_R          (*((volatile uint32 *)0x400FF038))
#define UDMA_PRIOCLR_R          (*((volatile uint32 *)0x400FF03C))
#define UDMA_ERRCLR_R           (*((volatile uint32 *)0x400FF04C))
#define UDMA_CHASGN_R           (*((volatile uint32 *)0x400FF500))
#define UDMA_CHIS_R             (*((volatile uint32 *)0x400FF504))
#define UDMA_CHMAP0_R           (*((volatile uint32 *)0x400FF510))
#define UDMA_CHMAP1_R           (*((volatile uint32 *)0x400FF514))
#define UDMA_CHMAP2_R           (*((volatile uint32 *)0x400FF518))
#define UDMA_CHMAP3_R           (*((volatile uint32 *)0x400FF51C))

//*****************************************************************************
//
// Micro Direct Memory Access (uDMA) offsets (UDMA)
//
//*****************************************************************************
#define UDMA_SRCENDP            0x00000000  // DMA Channel Source Address End
// Pointer
#define UDMA_DSTENDP            0x00000004  // DMA Channel Destination Address
// End Pointer
#define UDMA_CHCTL              0x00000008  // DMA Channel Control Word

//*****************************************************************************
//
// NVIC registers (NVIC)
//
//*****************************************************************************
#define NVIC_ACTLR_R            (*((volatile uint32 *)0xE000E008))
#define NVIC_ST_CTRL_R          (*((volatile uint32 *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32 *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32 *)0xE000E018))
#define NVIC_EN0_R              (*((volatile uint32 *)0xE000E100))
#define NVIC_EN1_R              (*((volatile uint32 *)0xE000E104))
#define NVIC_EN2_R              (*((volatile uint32 *)0xE000E108))
#define NVIC_EN3_R              (*((volatile uint32 *)0xE000E10C))
#define NVIC_EN4_R              (*((volatile uint32 *)0xE000E110))
#define NVIC_DIS0_R             (*((volatile uint32 *)0xE000E180))
#define NVIC_DIS1_R             (*((volatile uint32 *)0xE000E184))
#define NVIC_DIS2_R             (*((volatile uint32 *)0xE000E188))
#define NVIC_DIS3_R             (*((volatile uint32 *)0xE000E18C))
#define NVIC_DIS4_R             (*((volatile uint32 *)0xE000E190))
#define NVIC_PEND0_R            (*((volatile uint32 *)0xE000E200))
#define NVIC_PEND1_R            (*((volatile uint32 *)0xE000E204))
#define NVIC_PEND2_R            (*((volatile uint32 *)0xE000E208))
#define NVIC_PEND3_R            (*((volatile uint32 *)0xE000E20C))
#define NVIC_PEND4_R            (*((volatile uint32 *)0xE000E210))
#define NVIC_UNPEND0_R          (*((volatile uint32 *)0xE000E280))
#define NVIC_UNPEND1_R          (*((volatile uint32 *)0xE000E284))
#define NVIC_UNPEND2_R          (*((volatile uint32 *)0xE000E288))
#define NVIC_UNPEND3_R          (*((volatile uint32 *)0xE000E28C))
#define NVIC_UNPEND4_R          (*((volatile uint32 *)0xE000E290))
#define NVIC_ACTIVE0_R          (*((volatile uint32 *)0xE000E300))
#define NVIC_ACTIVE1_R          (*((volatile uint32 *)0xE000E304))
#define NVIC_ACTIVE2_R          (*((volatile uint32 *)0xE000E308))
#define NVIC_ACTIVE3_R          (*((volatile uint32 *)0xE000E30C))
#define NVIC_ACTIVE4_R          (*((volatile uint32 *)0xE000E310))
#define NVIC_PRI0_R             (*((volatile uint32 *)0xE000E400))
#define NVIC_PRI1_R             (*((volatile uint32 *)0xE000E404))
#define NVIC_PRI2_R             (*((volatile uint32 *)0xE000E408))
#define NVIC_PRI3_R             (*((volatile uint32 *)0xE000E40C))
#define NVIC_PRI4_R             (*((volatile uint32 *)0xE000E410))
#define NVIC_PRI5_R             (*((volatile uint32 *)0xE000E414))
#define NVIC_PRI6_R             (*((volatile uint32 *)0xE000E418))
#define NVIC_PRI7_R             (*((volatile uint32 *)0xE000E41C))
#define NVIC_PRI8_R             (*((volatile uint32 *)0xE000E420))
#define NVIC_PRI9_R             (*((volatile uint32 *)0xE000E424))
#define NVIC_PRI10_R            (*((volatile uint32 *)0xE000E428))
#define NVIC_PRI11_R            (*((volatile uint32 *)0xE000E42C))
#define NVIC_PRI12_R            (*((volatile uint32 *)0xE000E430))
#define NVIC_PRI13_R            (*((volatile uint32 *)0xE000E434))
#define NVIC_PRI14_R            (*((volatile uint32 *)0xE000E438))
#define NVIC_PRI15_R            (*((volatile uint32 *)0xE000E43C))
#define NVIC_PRI16_R            (*((volatile uint32 *)0xE000E440))
#define NVIC_PRI17_R            (*((volatile uint32 *)0xE000E444))
#define NVIC_PRI18_R            (*((volatile uint32 *)0xE000E448))
#define NVIC_PRI19_R            (*((volatile uint32 *)0xE000E44C))
#define NVIC_PRI20_R            (*((volatile uint32 *)0xE000E450))
#define NVIC_PRI21_R            (*((volatile uint32 *)0xE000E454))
#define NVIC_PRI22_R            (*((volatile uint32 *)0xE000E458))
#define NVIC_PRI23_R            (*((volatile uint32 *)0xE000E45C))
#define NVIC_PRI24_R            (*((volatile uint32 *)0xE000E460))
#define NVIC_PRI25_R            (*((volatile uint32 *)0xE000E464))
#define NVIC_PRI26_R            (*((volatile uint32 *)0xE000E468))
#define NVIC_PRI27_R            (*((volatile uint32 *)0xE000E46C))
#define NVIC_PRI28_R            (*((volatile uint32 *)0xE000E470))
#define NVIC_PRI29_R            (*((volatile uint32 *)0xE000E474))
#define NVIC_PRI30_R            (*((volatile uint32 *)0xE000E478))
#define NVIC_PRI31_R            (*((volatile uint32 *)0xE000E47C))
#define NVIC_PRI32_R            (*((volatile uint32 *)0xE000E480))
#define NVIC_PRI33_R            (*((volatile uint32 *)0xE000E484))
#define NVIC_PRI34_R            (*((volatile uint32 *)0xE000E488))
#define NVIC_CPUID_R            (*((volatile uint32 *)0xE000ED00))
#define NVIC_INT_CTRL_R         (*((volatile uint32 *)0xE000ED04))
#define NVIC_VTABLE_R           (*((volatile uint32 *)0xE000ED08))
#define NVIC_APINT_R            (*((volatile uint32 *)0xE000ED0C))
#define NVIC_SYS_CTRL_R         (*((volatile uint32 *)0xE000ED10))
#define NVIC_CFG_CTRL_R         (*((volatile uint32 *)0xE000ED14))
#define NVIC_SYS_PRI1_R         (*((volatile uint32 *)0xE000ED18))
#define NVIC_SYS_PRI2_R         (*((volatile uint32 *)0xE000ED1C))
#define NVIC_SYS_PRI3_R         (*((volatile uint32 *)0xE000ED20))
#define NVIC_SYS_HND_CTRL_R     (*((volatile uint32 *)0xE000ED24))
#define NVIC_FAULT_STAT_R       (*((volatile uint32 *)0xE000ED28))
#define NVIC_HFAULT_STAT_R      (*((volatile uint32 *)0xE000ED2C))
#define NVIC_DEBUG_STAT_R       (*((volatile uint32 *)0xE000ED30))
#define NVIC_MM_ADDR_R          (*((volatile uint32 *)0xE000ED34))
#define NVIC_FAULT_ADDR_R       (*((volatile uint32 *)0xE000ED38))
#define NVIC_CPAC_R             (*((volatile uint32 *)0xE000ED88))
#define NVIC_MPU_TYPE_R         (*((volatile uint32 *)0xE000ED90))
#define NVIC_MPU_CTRL_R         (*((volatile uint32 *)0xE000ED94))
#define NVIC_MPU_NUMBER_R       (*((volatile uint32 *)0xE000ED98))
#define NVIC_MPU_BASE_R         (*((volatile uint32 *)0xE000ED9C))
#define NVIC_MPU_ATTR_R         (*((volatile uint32 *)0xE000EDA0))
#define NVIC_MPU_BASE1_R        (*((volatile uint32 *)0xE000EDA4))
#define NVIC_MPU_ATTR1_R        (*((volatile uint32 *)0xE000EDA8))
#define NVIC_MPU_BASE2_R        (*((volatile uint32 *)0xE000EDAC))
#define NVIC_MPU_ATTR2_R        (*((volatile uint32 *)0xE000EDB0))
#define NVIC_MPU_BASE3_R        (*((volatile uint32 *)0xE000EDB4))
#define NVIC_MPU_ATTR3_R        (*((volatile uint32 *)0xE000EDB8))
#define NVIC_DBG_CTRL_R         (*((volatile uint32 *)0xE000EDF0))
#define NVIC_DBG_XFER_R         (*((volatile uint32 *)0xE000EDF4))
#define NVIC_DBG_DATA_R         (*((volatile uint32 *)0xE000EDF8))
#define NVIC_DBG_INT_R          (*((volatile uint32 *)0xE000EDFC))
#define NVIC_SW_TRIG_R          (*((volatile uint32 *)0xE000EF00))
#define NVIC_FPCC_R             (*((volatile uint32 *)0xE000EF34))
#define NVIC_FPCA_R             (*((volatile uint32 *)0xE000EF38))
#define NVIC_FPDSC_R            (*((volatile uint32 *)0xE000EF3C))

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOAD register.
//
//*****************************************************************************
#define WDT_LOAD_M              0xFFFFFFFF  // Watchdog Load Value
#define WDT_LOAD_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_VALUE register.
//
//*****************************************************************************
#define WDT_VALUE_M             0xFFFFFFFF  // Watchdog Value
#define WDT_VALUE_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_CTL register.
//
//*****************************************************************************
#define WDT_CTL_WRC             0x80000000  // Write Complete
#define WDT_CTL_INTTYPE         0x00000004  // Watchdog Interrupt Type
#define WDT_CTL_RESEN           0x00000002  // Watchdog Reset Enable
#define WDT_CTL_INTEN           0x00000001  // Watchdog Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_ICR register.
//
//*****************************************************************************
#define WDT_ICR_M               0xFFFFFFFF  // Watchdog Interrupt Clear
#define WDT_ICR_S               0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_RIS register.
//
//*****************************************************************************
#define WDT_RIS_WDTRIS          0x00000001  // Watchdog Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_MIS register.
//
//*****************************************************************************
#define WDT_MIS_WDTMIS          0x00000001  // Watchdog Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_TEST register.
//
//*****************************************************************************
#define WDT_TEST_STALL          0x00000100  // Watchdog Stall Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOCK register.
//
//*****************************************************************************
#define WDT_LOCK_M              0xFFFFFFFF  // Watchdog Lock
#define WDT_LOCK_UNLOCKED       0x00000000  // Unlocked
#define WDT_LOCK_LOCKED         0x00000001  // Locked
#define WDT_LOCK_UNLOCK         0x1ACCE551  // Unlocks the watchdog timer

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_IM register.
//
//*****************************************************************************
#define GPIO_IM_GPIO_M          0x000000FF  // GPIO Interrupt Mask Enable
#define GPIO_IM_GPIO_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_RIS register.
//
//*****************************************************************************
#define GPIO_RIS_GPIO_M         0x000000FF  // GPIO Interrupt Raw Status
#define GPIO_RIS_GPIO_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_MIS register.
//
//*****************************************************************************
#define GPIO_MIS_GPIO_M         0x000000FF  // GPIO Masked Interrupt Status
#define GPIO_MIS_GPIO_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_ICR register.
//
//*****************************************************************************
#define GPIO_ICR_GPIO_M         0x000000FF  // GPIO Interrupt Clear
#define GPIO_ICR_GPIO_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_LOCK register.
//
//*****************************************************************************
#define GPIO_LOCK_M             0xFFFFFFFF  // GPIO Lock
#define GPIO_LOCK_UNLOCKED      0x00000000  // The GPIOCR register is unlocked
// and may be modified
#define GPIO_LOCK_LOCKED        0x00000001  // The GPIOCR register is locked
// and may not be modified
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register

//*****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port A.
//
//*****************************************************************************

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
//*****************************************************************************
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_FRF_TI          0x00000010  // Synchronous Serial Frame Format
#define SSI_CR0_FRF_NMW         0x00000020  // MICROWIRE Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select
#define SSI_CR0_DSS_4           0x00000003  // 4-bit data
#define SSI_CR0_DSS_5           0x00000004  // 5-bit data
#define SSI_CR0_DSS_6           0x00000005  // 6-bit data
#define SSI_CR0_DSS_7           0x00000006  // 7-bit data
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR0_DSS_9           0x00000008  // 9-bit data
#define SSI_CR0_DSS_10          0x00000009  // 10-bit data
#define SSI_CR0_DSS_11          0x0000000A  // 11-bit data
#define SSI_CR0_DSS_12          0x0000000B  // 12-bit data
#define SSI_CR0_DSS_13          0x0000000C  // 13-bit data
#define SSI_CR0_DSS_14          0x0000000D  // 14-bit data
#define SSI_CR0_DSS_15          0x0000000E  // 15-bit data
#define SSI_CR0_DSS_16          0x0000000F  // 16-bit data
#define SSI_CR0_SCR_S           8

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
//*****************************************************************************
#define SSI_CR1_EOT             0x00000010  // End of Transmission
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
// Enable
#define SSI_CR1_LBM             0x00000001  // SSI Loopback Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
//*****************************************************************************
#define SSI_DR_DATA_M           0x0000FFFF  // SSI Receive/Transmit Data
#define SSI_DR_DATA_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
//*****************************************************************************
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit
#define SSI_SR_RFF              0x00000008  // SSI Receive FIFO Full
#define SSI_SR_RNE              0x00000004  // SSI Receive FIFO Not Empty
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full
#define SSI_SR_TFE              0x00000001  // SSI Transmit FIFO Empty

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
//*****************************************************************************
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor
#define SSI_CPSR_CPSDVSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
//*****************************************************************************
#define SSI_IM_TXIM             0x00000008  // SSI Transmit FIFO Interrupt Mask
#define SSI_IM_RXIM             0x00000004  // SSI Receive FIFO Interrupt Mask
#define SSI_IM_RTIM             0x00000002  // SSI Receive Time-Out Interrupt
// Mask
#define SSI_IM_RORIM            0x00000001  // SSI Receive Overrun Interrupt
// Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
#define SSI_RIS_TXRIS           0x00000008  // SSI Transmit FIFO Raw Interrupt
// Status
#define SSI_RIS_RXRIS           0x00000004  // SSI Receive FIFO Raw Interrupt
// Status
#define SSI_RIS_RTRIS           0x00000002  // SSI Receive Time-Out Raw
// Interrupt Status
#define SSI_RIS_RORRIS          0x00000001  // SSI Receive Overrun Raw
// Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
#define SSI_MIS_TXMIS           0x00000008  // SSI Transmit FIFO Masked
// Interrupt Status
#define SSI_MIS_RXMIS           0x00000004  // SSI Receive FIFO Masked
// Interrupt Status
#define SSI_MIS_RTMIS           0x00000002  // SSI Receive Time-Out Masked
// Interrupt Status
#define SSI_MIS_RORMIS          0x00000001  // SSI Receive Overrun Masked
// Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
#define SSI_ICR_RTIC            0x00000002  // SSI Receive Time-Out Interrupt
// Clear
#define SSI_ICR_RORIC           0x00000001  // SSI Receive Overrun Interrupt
// Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DMACTL register.
//
//*****************************************************************************
#define SSI_DMACTL_TXDMAE       0x00000002  // Transmit DMA Enable
#define SSI_DMACTL_RXDMAE       0x00000001  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CC register.
//
//*****************************************************************************
#define SSI_CC_CS_M             0x0000000F  // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL        0x00000000  // System clock (based on clock
// source and divisor factor)
#define SSI_CC_CS_PIOSC         0x00000005  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DR register.
//
//*****************************************************************************
#define UART_DR_OE              0x00000800  // UART Overrun Error
#define UART_DR_BE              0x00000400  // UART Break Error
#define UART_DR_PE              0x00000200  // UART Parity Error
#define UART_DR_FE              0x00000100  // UART Framing Error
#define UART_DR_DATA_M          0x000000FF  // Data Transmitted or Received
#define UART_DR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RSR register.
//
//*****************************************************************************
#define UART_RSR_OE             0x00000008  // UART Overrun Error
#define UART_RSR_BE             0x00000004  // UART Break Error
#define UART_RSR_PE             0x00000002  // UART Parity Error
#define UART_RSR_FE             0x00000001  // UART Framing Error

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
//*****************************************************************************
#define UART_ECR_DATA_M         0x000000FF  // Error Clear
#define UART_ECR_DATA_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FR register.
//
//*****************************************************************************
#define UART_FR_TXFE            0x00000080  // UART Transmit FIFO Empty
#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_FR_BUSY            0x00000008  // UART Busy
#define UART_FR_CTS             0x00000001  // Clear To Send

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
//*****************************************************************************
#define UART_ILPR_ILPDVSR_M     0x000000FF  // IrDA Low-Power Divisor
#define UART_ILPR_ILPDVSR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IBRD register.
//
//*****************************************************************************
#define UART_IBRD_DIVINT_M      0x0000FFFF  // Integer Baud-Rate Divisor
#define UART_IBRD_DIVINT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FBRD register.
//
//*****************************************************************************
#define UART_FBRD_DIVFRAC_M     0x0000003F  // Fractional Baud-Rate Divisor
#define UART_FBRD_DIVFRAC_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
//*****************************************************************************
#define UART_LCRH_SPS           0x00000080  // UART Stick Parity Select
#define UART_LCRH_WLEN_M        0x00000060  // UART Word Length
#define UART_LCRH_WLEN_5        0x00000000  // 5 bits (default)
#define UART_LCRH_WLEN_6        0x00000020  // 6 bits
#define UART_LCRH_WLEN_7        0x00000040  // 7 bits
#define UART_LCRH_WLEN_8        0x00000060  // 8 bits
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_LCRH_STP2          0x00000008  // UART Two Stop Bits Select
#define UART_LCRH_EPS           0x00000004  // UART Even Parity Select
#define UART_LCRH_PEN           0x00000002  // UART Parity Enable
#define UART_LCRH_BRK           0x00000001  // UART Send Break

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CTL register.
//
//*****************************************************************************
#define UART_CTL_CTSEN          0x00008000  // Enable Clear To Send
#define UART_CTL_RTSEN          0x00004000  // Enable Request to Send
#define UART_CTL_RTS            0x00000800  // Request to Send
#define UART_CTL_RXE            0x00000200  // UART Receive Enable
#define UART_CTL_TXE            0x00000100  // UART Transmit Enable
#define UART_CTL_LBE            0x00000080  // UART Loop Back Enable
#define UART_CTL_HSE            0x00000020  // High-Speed Enable
#define UART_CTL_EOT            0x00000010  // End of Transmission
#define UART_CTL_SMART          0x00000008  // ISO 7816 Smart Card Support
#define UART_CTL_SIRLP          0x00000004  // UART SIR Low-Power Mode
#define UART_CTL_SIREN          0x00000002  // UART SIR Enable
#define UART_CTL_UARTEN         0x00000001  // UART Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IFLS register.
//
//*****************************************************************************
#define UART_IFLS_RX_M          0x00000038  // UART Receive Interrupt FIFO
// Level Select
#define UART_IFLS_RX1_8         0x00000000  // RX FIFO >= 1/8 full
#define UART_IFLS_RX2_8         0x00000008  // RX FIFO >= 1/4 full
#define UART_IFLS_RX4_8         0x00000010  // RX FIFO >= 1/2 full (default)
#define UART_IFLS_RX6_8         0x00000018  // RX FIFO >= 3/4 full
#define UART_IFLS_RX7_8         0x00000020  // RX FIFO >= 7/8 full
#define UART_IFLS_TX_M          0x00000007  // UART Transmit Interrupt FIFO
// Level Select
#define UART_IFLS_TX1_8         0x00000000  // TX FIFO <= 1/8 full
#define UART_IFLS_TX2_8         0x00000001  // TX FIFO <= 1/4 full
#define UART_IFLS_TX4_8         0x00000002  // TX FIFO <= 1/2 full (default)
#define UART_IFLS_TX6_8         0x00000003  // TX FIFO <= 3/4 full
#define UART_IFLS_TX7_8         0x00000004  // TX FIFO <= 7/8 full

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IM register.
//
//*****************************************************************************
#define UART_IM_9BITIM          0x00001000  // 9-Bit Mode Interrupt Mask
#define UART_IM_OEIM            0x00000400  // UART Overrun Error Interrupt
// Mask
#define UART_IM_BEIM            0x00000200  // UART Break Error Interrupt Mask
#define UART_IM_PEIM            0x00000100  // UART Parity Error Interrupt Mask
#define UART_IM_FEIM            0x00000080  // UART Framing Error Interrupt
// Mask
#define UART_IM_RTIM            0x00000040  // UART Receive Time-Out Interrupt
// Mask
#define UART_IM_TXIM            0x00000020  // UART Transmit Interrupt Mask
#define UART_IM_RXIM            0x00000010  // UART Receive Interrupt Mask
#define UART_IM_CTSMIM          0x00000002  // UART Clear to Send Modem
// Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
//*****************************************************************************
#define UART_RIS_9BITRIS        0x00001000  // 9-Bit Mode Raw Interrupt Status
#define UART_RIS_OERIS          0x00000400  // UART Overrun Error Raw Interrupt
// Status
#define UART_RIS_BERIS          0x00000200  // UART Break Error Raw Interrupt
// Status
#define UART_RIS_PERIS          0x00000100  // UART Parity Error Raw Interrupt
// Status
#define UART_RIS_FERIS          0x00000080  // UART Framing Error Raw Interrupt
// Status
#define UART_RIS_RTRIS          0x00000040  // UART Receive Time-Out Raw
// Interrupt Status
#define UART_RIS_TXRIS          0x00000020  // UART Transmit Raw Interrupt
// Status
#define UART_RIS_RXRIS          0x00000010  // UART Receive Raw Interrupt
// Status
#define UART_RIS_CTSRIS         0x00000002  // UART Clear to Send Modem Raw
// Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
//*****************************************************************************
#define UART_MIS_9BITMIS        0x00001000  // 9-Bit Mode Masked Interrupt
// Status
#define UART_MIS_OEMIS          0x00000400  // UART Overrun Error Masked
// Interrupt Status
#define UART_MIS_BEMIS          0x00000200  // UART Break Error Masked
// Interrupt Status
#define UART_MIS_PEMIS          0x00000100  // UART Parity Error Masked
// Interrupt Status
#define UART_MIS_FEMIS          0x00000080  // UART Framing Error Masked
// Interrupt Status
#define UART_MIS_RTMIS          0x00000040  // UART Receive Time-Out Masked
// Interrupt Status
#define UART_MIS_TXMIS          0x00000020  // UART Transmit Masked Interrupt
// Status
#define UART_MIS_RXMIS          0x00000010  // UART Receive Masked Interrupt
// Status
#define UART_MIS_CTSMIS         0x00000002  // UART Clear to Send Modem Masked
// Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
//*****************************************************************************
#define UART_ICR_9BITIC         0x00001000  // 9-Bit Mode Interrupt Clear
#define UART_ICR_OEIC           0x00000400  // Overrun Error Interrupt Clear
#define UART_ICR_BEIC           0x00000200  // Break Error Interrupt Clear
#define UART_ICR_PEIC           0x00000100  // Parity Error Interrupt Clear
#define UART_ICR_FEIC           0x00000080  // Framing Error Interrupt Clear
#define UART_ICR_RTIC           0x00000040  // Receive Time-Out Interrupt Clear
#define UART_ICR_TXIC           0x00000020  // Transmit Interrupt Clear
#define UART_ICR_RXIC           0x00000010  // Receive Interrupt Clear
#define UART_ICR_CTSMIC         0x00000002  // UART Clear to Send Modem
// Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
//*****************************************************************************
#define UART_DMACTL_DMAERR      0x00000004  // DMA on Error
#define UART_DMACTL_TXDMAE      0x00000002  // Transmit DMA Enable
#define UART_DMACTL_RXDMAE      0x00000001  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITADDR
// register.
//
//*****************************************************************************
#define UART_9BITADDR_9BITEN    0x00008000  // Enable 9-Bit Mode
#define UART_9BITADDR_ADDR_M    0x000000FF  // Self Address for 9-Bit Mode
#define UART_9BITADDR_ADDR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITAMASK
// register.
//
//*****************************************************************************
#define UART_9BITAMASK_MASK_M   0x000000FF  // Self Address Mask for 9-Bit Mode
#define UART_9BITAMASK_MASK_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_PP register.
//
//*****************************************************************************
#define UART_PP_NB              0x00000002  // 9-Bit Support
#define UART_PP_SC              0x00000001  // Smart Card Support

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CC register.
//
//*****************************************************************************
#define UART_CC_CS_M            0x0000000F  // UART Baud Clock Source
#define UART_CC_CS_SYSCLK       0x00000000  // System clock (based on clock
// source and divisor factor)
#define UART_CC_CS_PIOSC        0x00000005  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MSA register.
//
//*****************************************************************************
#define I2C_MSA_SA_M            0x000000FE  // I2C Slave Address
#define I2C_MSA_RS              0x00000001  // Receive not send
#define I2C_MSA_SA_S            1

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCS register.
//
//*****************************************************************************
#define I2C_MCS_CLKTO           0x00000080  // Clock Timeout Error
#define I2C_MCS_BUSBSY          0x00000040  // Bus Busy
#define I2C_MCS_IDLE            0x00000020  // I2C Idle
#define I2C_MCS_ARBLST          0x00000010  // Arbitration Lost
#define I2C_MCS_HS              0x00000010  // High-Speed Enable
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MDR register.
//
//*****************************************************************************
#define I2C_MDR_DATA_M          0x000000FF  // This byte contains the data
// transferred during a transaction
#define I2C_MDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MTPR register.
//
//*****************************************************************************
#define I2C_MTPR_HS             0x00000080  // High-Speed Enable
#define I2C_MTPR_TPR_M          0x0000007F  // Timer Period
#define I2C_MTPR_TPR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
//*****************************************************************************
#define I2C_MIMR_CLKIM          0x00000002  // Clock Timeout Interrupt Mask
#define I2C_MIMR_IM             0x00000001  // Master Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
//*****************************************************************************
#define I2C_MRIS_CLKRIS         0x00000002  // Clock Timeout Raw Interrupt
// Status
#define I2C_MRIS_RIS            0x00000001  // Master Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
//*****************************************************************************
#define I2C_MMIS_CLKMIS         0x00000002  // Clock Timeout Masked Interrupt
// Status
#define I2C_MMIS_MIS            0x00000001  // Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
//*****************************************************************************
#define I2C_MICR_CLKIC          0x00000002  // Clock Timeout Interrupt Clear
#define I2C_MICR_IC             0x00000001  // Master Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
//*****************************************************************************
#define I2C_MCR_GFE             0x00000040  // I2C Glitch Filter Enable
#define I2C_MCR_SFE             0x00000020  // I2C Slave Function Enable
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define I2C_MCR_LPBK            0x00000001  // I2C Loopback

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCLKOCNT register.
//
//*****************************************************************************
#define I2C_MCLKOCNT_CNTL_M     0x000000FF  // I2C Master Count
#define I2C_MCLKOCNT_CNTL_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBMON register.
//
//*****************************************************************************
#define I2C_MBMON_SDA           0x00000002  // I2C SDA Status
#define I2C_MBMON_SCL           0x00000001  // I2C SCL Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR2 register.
//
//*****************************************************************************
#define I2C_MCR2_GFPW_M         0x00000070  // I2C Glitch Filter Pulse Width
#define I2C_MCR2_GFPW_BYPASS    0x00000000  // Bypass
#define I2C_MCR2_GFPW_1         0x00000010  // 1 clock
#define I2C_MCR2_GFPW_2         0x00000020  // 2 clocks
#define I2C_MCR2_GFPW_3         0x00000030  // 3 clocks
#define I2C_MCR2_GFPW_4         0x00000040  // 4 clocks
#define I2C_MCR2_GFPW_8         0x00000050  // 8 clocks
#define I2C_MCR2_GFPW_16        0x00000060  // 16 clocks
#define I2C_MCR2_GFPW_31        0x00000070  // 31 clocks

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR register.
//
//*****************************************************************************
#define I2C_SOAR_OAR_M          0x0000007F  // I2C Slave Own Address
#define I2C_SOAR_OAR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SCSR register.
//
//*****************************************************************************
#define I2C_SCSR_OAR2SEL        0x00000008  // OAR2 Address Matched
#define I2C_SCSR_FBR            0x00000004  // First Byte Received
#define I2C_SCSR_TREQ           0x00000002  // Transmit Request
#define I2C_SCSR_DA             0x00000001  // Device Active
#define I2C_SCSR_RREQ           0x00000001  // Receive Request

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SDR register.
//
//*****************************************************************************
#define I2C_SDR_DATA_M          0x000000FF  // Data for Transfer
#define I2C_SDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SIMR register.
//
//*****************************************************************************
#define I2C_SIMR_STOPIM         0x00000004  // Stop Condition Interrupt Mask
#define I2C_SIMR_STARTIM        0x00000002  // Start Condition Interrupt Mask
#define I2C_SIMR_DATAIM         0x00000001  // Data Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
//*****************************************************************************
#define I2C_SRIS_STOPRIS        0x00000004  // Stop Condition Raw Interrupt
// Status
#define I2C_SRIS_STARTRIS       0x00000002  // Start Condition Raw Interrupt
// Status
#define I2C_SRIS_DATARIS        0x00000001  // Data Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SMIS register.
//
//*****************************************************************************
#define I2C_SMIS_STOPMIS        0x00000004  // Stop Condition Masked Interrupt
// Status
#define I2C_SMIS_STARTMIS       0x00000002  // Start Condition Masked Interrupt
// Status
#define I2C_SMIS_DATAMIS        0x00000001  // Data Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SICR register.
//
//*****************************************************************************
#define I2C_SICR_STOPIC         0x00000004  // Stop Condition Interrupt Clear
#define I2C_SICR_STARTIC        0x00000002  // Start Condition Interrupt Clear
#define I2C_SICR_DATAIC         0x00000001  // Data Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR2 register.
//
//*****************************************************************************
#define I2C_SOAR2_OAR2EN        0x00000080  // I2C Slave Own Address 2 Enable
#define I2C_SOAR2_OAR2_M        0x0000007F  // I2C Slave Own Address 2
#define I2C_SOAR2_OAR2_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SACKCTL register.
//
//*****************************************************************************
#define I2C_SACKCTL_ACKOVAL     0x00000002  // I2C Slave ACK Override Value
#define I2C_SACKCTL_ACKOEN      0x00000001  // I2C Slave ACK Override Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PP register.
//
//*****************************************************************************
#define I2C_PP_HS               0x00000001  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PC register.
//
//*****************************************************************************
#define I2C_PC_HS               0x00000001  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CTL register.
//
//*****************************************************************************
#define PWM_CTL_GLOBALSYNC3     0x00000008  // Update PWM Generator 3
#define PWM_CTL_GLOBALSYNC2     0x00000004  // Update PWM Generator 2
#define PWM_CTL_GLOBALSYNC1     0x00000002  // Update PWM Generator 1
#define PWM_CTL_GLOBALSYNC0     0x00000001  // Update PWM Generator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_SYNC register.
//
//*****************************************************************************
#define PWM_SYNC_SYNC3          0x00000008  // Reset Generator 3 Counter
#define PWM_SYNC_SYNC2          0x00000004  // Reset Generator 2 Counter
#define PWM_SYNC_SYNC1          0x00000002  // Reset Generator 1 Counter
#define PWM_SYNC_SYNC0          0x00000001  // Reset Generator 0 Counter

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENABLE register.
//
//*****************************************************************************
#define PWM_ENABLE_PWM7EN       0x00000080  // MnPWM7 Output Enable
#define PWM_ENABLE_PWM6EN       0x00000040  // MnPWM6 Output Enable
#define PWM_ENABLE_PWM5EN       0x00000020  // MnPWM5 Output Enable
#define PWM_ENABLE_PWM4EN       0x00000010  // MnPWM4 Output Enable
#define PWM_ENABLE_PWM3EN       0x00000008  // MnPWM3 Output Enable
#define PWM_ENABLE_PWM2EN       0x00000004  // MnPWM2 Output Enable
#define PWM_ENABLE_PWM1EN       0x00000002  // MnPWM1 Output Enable
#define PWM_ENABLE_PWM0EN       0x00000001  // MnPWM0 Output Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INVERT register.
//
//*****************************************************************************
#define PWM_INVERT_PWM7INV      0x00000080  // Invert MnPWM7 Signal
#define PWM_INVERT_PWM6INV      0x00000040  // Invert MnPWM6 Signal
#define PWM_INVERT_PWM5INV      0x00000020  // Invert MnPWM5 Signal
#define PWM_INVERT_PWM4INV      0x00000010  // Invert MnPWM4 Signal
#define PWM_INVERT_PWM3INV      0x00000008  // Invert MnPWM3 Signal
#define PWM_INVERT_PWM2INV      0x00000004  // Invert MnPWM2 Signal
#define PWM_INVERT_PWM1INV      0x00000002  // Invert MnPWM1 Signal
#define PWM_INVERT_PWM0INV      0x00000001  // Invert MnPWM0 Signal

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULT register.
//
//*****************************************************************************
#define PWM_FAULT_FAULT7        0x00000080  // MnPWM7 Fault
#define PWM_FAULT_FAULT6        0x00000040  // MnPWM6 Fault
#define PWM_FAULT_FAULT5        0x00000020  // MnPWM5 Fault
#define PWM_FAULT_FAULT4        0x00000010  // MnPWM4 Fault
#define PWM_FAULT_FAULT3        0x00000008  // MnPWM3 Fault
#define PWM_FAULT_FAULT2        0x00000004  // MnPWM2 Fault
#define PWM_FAULT_FAULT1        0x00000002  // MnPWM1 Fault
#define PWM_FAULT_FAULT0        0x00000001  // MnPWM0 Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INTEN register.
//
//*****************************************************************************
#define PWM_INTEN_INTFAULT1     0x00020000  // Interrupt Fault 1
#define PWM_INTEN_INTFAULT0     0x00010000  // Interrupt Fault 0
#define PWM_INTEN_INTPWM3       0x00000008  // PWM3 Interrupt Enable
#define PWM_INTEN_INTPWM2       0x00000004  // PWM2 Interrupt Enable
#define PWM_INTEN_INTPWM1       0x00000002  // PWM1 Interrupt Enable
#define PWM_INTEN_INTPWM0       0x00000001  // PWM0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_RIS register.
//
//*****************************************************************************
#define PWM_RIS_INTFAULT1       0x00020000  // Interrupt Fault PWM 1
#define PWM_RIS_INTFAULT0       0x00010000  // Interrupt Fault PWM 0
#define PWM_RIS_INTPWM3         0x00000008  // PWM3 Interrupt Asserted
#define PWM_RIS_INTPWM2         0x00000004  // PWM2 Interrupt Asserted
#define PWM_RIS_INTPWM1         0x00000002  // PWM1 Interrupt Asserted
#define PWM_RIS_INTPWM0         0x00000001  // PWM0 Interrupt Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ISC register.
//
//*****************************************************************************
#define PWM_ISC_INTFAULT1       0x00020000  // FAULT1 Interrupt Asserted
#define PWM_ISC_INTFAULT0       0x00010000  // FAULT0 Interrupt Asserted
#define PWM_ISC_INTPWM3         0x00000008  // PWM3 Interrupt Status
#define PWM_ISC_INTPWM2         0x00000004  // PWM2 Interrupt Status
#define PWM_ISC_INTPWM1         0x00000002  // PWM1 Interrupt Status
#define PWM_ISC_INTPWM0         0x00000001  // PWM0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_STATUS register.
//
//*****************************************************************************
#define PWM_STATUS_FAULT1       0x00000002  // Generator 1 Fault Status
#define PWM_STATUS_FAULT0       0x00000001  // Generator 0 Fault Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULTVAL register.
//
//*****************************************************************************
#define PWM_FAULTVAL_PWM7       0x00000080  // MnPWM7 Fault Value
#define PWM_FAULTVAL_PWM6       0x00000040  // MnPWM6 Fault Value
#define PWM_FAULTVAL_PWM5       0x00000020  // MnPWM5 Fault Value
#define PWM_FAULTVAL_PWM4       0x00000010  // MnPWM4 Fault Value
#define PWM_FAULTVAL_PWM3       0x00000008  // MnPWM3 Fault Value
#define PWM_FAULTVAL_PWM2       0x00000004  // MnPWM2 Fault Value
#define PWM_FAULTVAL_PWM1       0x00000002  // MnPWM1 Fault Value
#define PWM_FAULTVAL_PWM0       0x00000001  // MnPWM0 Fault Value

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENUPD register.
//
//*****************************************************************************
#define PWM_ENUPD_ENUPD7_M      0x0000C000  // MnPWM7 Enable Update Mode
#define PWM_ENUPD_ENUPD7_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD7_LSYNC  0x00008000  // Locally Synchronized
#define PWM_ENUPD_ENUPD7_GSYNC  0x0000C000  // Globally Synchronized
#define PWM_ENUPD_ENUPD6_M      0x00003000  // MnPWM6 Enable Update Mode
#define PWM_ENUPD_ENUPD6_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD6_LSYNC  0x00002000  // Locally Synchronized
#define PWM_ENUPD_ENUPD6_GSYNC  0x00003000  // Globally Synchronized
#define PWM_ENUPD_ENUPD5_M      0x00000C00  // MnPWM5 Enable Update Mode
#define PWM_ENUPD_ENUPD5_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD5_LSYNC  0x00000800  // Locally Synchronized
#define PWM_ENUPD_ENUPD5_GSYNC  0x00000C00  // Globally Synchronized
#define PWM_ENUPD_ENUPD4_M      0x00000300  // MnPWM4 Enable Update Mode
#define PWM_ENUPD_ENUPD4_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD4_LSYNC  0x00000200  // Locally Synchronized
#define PWM_ENUPD_ENUPD4_GSYNC  0x00000300  // Globally Synchronized
#define PWM_ENUPD_ENUPD3_M      0x000000C0  // MnPWM3 Enable Update Mode
#define PWM_ENUPD_ENUPD3_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD3_LSYNC  0x00000080  // Locally Synchronized
#define PWM_ENUPD_ENUPD3_GSYNC  0x000000C0  // Globally Synchronized
#define PWM_ENUPD_ENUPD2_M      0x00000030  // MnPWM2 Enable Update Mode
#define PWM_ENUPD_ENUPD2_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD2_LSYNC  0x00000020  // Locally Synchronized
#define PWM_ENUPD_ENUPD2_GSYNC  0x00000030  // Globally Synchronized
#define PWM_ENUPD_ENUPD1_M      0x0000000C  // MnPWM1 Enable Update Mode
#define PWM_ENUPD_ENUPD1_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD1_LSYNC  0x00000008  // Locally Synchronized
#define PWM_ENUPD_ENUPD1_GSYNC  0x0000000C  // Globally Synchronized
#define PWM_ENUPD_ENUPD0_M      0x00000003  // MnPWM0 Enable Update Mode
#define PWM_ENUPD_ENUPD0_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD0_LSYNC  0x00000002  // Locally Synchronized
#define PWM_ENUPD_ENUPD0_GSYNC  0x00000003  // Globally Synchronized

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CTL register.
//
//*****************************************************************************
#define PWM_0_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_0_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_0_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_0_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_0_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_0_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_0_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_0_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_0_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_0_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_0_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_0_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_0_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_0_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_0_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_0_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_0_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_0_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_0_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_0_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_0_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_0_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_0_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_0_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_0_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_0_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_0_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_0_CTL_MODE          0x00000002  // Counter Mode
#define PWM_0_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_INTEN register.
//
//*****************************************************************************
#define PWM_0_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
// Down
#define PWM_0_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_0_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
// Down
#define PWM_0_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_0_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_0_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_0_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
// Down
#define PWM_0_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
// Up
#define PWM_0_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
// Down
#define PWM_0_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
// Up
#define PWM_0_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_0_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_RIS register.
//
//*****************************************************************************
#define PWM_0_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
// Status
#define PWM_0_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_0_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
// Status
#define PWM_0_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_0_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_0_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_ISC register.
//
//*****************************************************************************
#define PWM_0_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_0_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_0_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_0_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_0_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_0_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_LOAD register.
//
//*****************************************************************************
#define PWM_0_LOAD_M            0x0000FFFF  // Counter Load Value
#define PWM_0_LOAD_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_COUNT register.
//
//*****************************************************************************
#define PWM_0_COUNT_M           0x0000FFFF  // Counter Value
#define PWM_0_COUNT_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPA register.
//
//*****************************************************************************
#define PWM_0_CMPA_M            0x0000FFFF  // Comparator A Value
#define PWM_0_CMPA_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPB register.
//
//*****************************************************************************
#define PWM_0_CMPB_M            0x0000FFFF  // Comparator B Value
#define PWM_0_CMPB_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENA register.
//
//*****************************************************************************
#define PWM_0_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_0_GENA_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_0_GENA_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_0_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_0_GENA_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_0_GENA_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_0_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_0_GENA_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_0_GENA_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_0_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_0_GENA_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_0_GENA_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_0_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_0_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_0_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_0_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_0_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_0_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_0_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_0_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_0_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENB register.
//
//*****************************************************************************
#define PWM_0_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_0_GENB_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_0_GENB_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_0_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_0_GENB_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_0_GENB_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_0_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_0_GENB_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_0_GENB_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_0_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_0_GENB_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_0_GENB_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_0_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_0_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_0_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_0_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_0_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_0_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_0_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_0_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_0_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBCTL register.
//
//*****************************************************************************
#define PWM_0_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBRISE register.
//
//*****************************************************************************
#define PWM_0_DBRISE_DELAY_M    0x00000FFF  // Dead-Band Rise Delay
#define PWM_0_DBRISE_DELAY_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBFALL register.
//
//*****************************************************************************
#define PWM_0_DBFALL_DELAY_M    0x00000FFF  // Dead-Band Fall Delay
#define PWM_0_DBFALL_DELAY_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_0_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_0_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_0_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_0_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_0_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_0_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_0_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_0_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_0_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_0_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_0_MINFLTPER_M       0x0000FFFF  // Minimum Fault Period
#define PWM_0_MINFLTPER_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CTL register.
//
//*****************************************************************************
#define PWM_1_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_1_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_1_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_1_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_1_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_1_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_1_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_1_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_1_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_1_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_1_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_1_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_1_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_1_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_1_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_1_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_1_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_1_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_1_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_1_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_1_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_1_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_1_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_1_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_1_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_1_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_1_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_1_CTL_MODE          0x00000002  // Counter Mode
#define PWM_1_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_INTEN register.
//
//*****************************************************************************
#define PWM_1_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
// Down
#define PWM_1_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_1_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
// Down
#define PWM_1_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_1_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_1_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_1_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
// Down
#define PWM_1_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
// Up
#define PWM_1_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
// Down
#define PWM_1_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
// Up
#define PWM_1_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_1_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_RIS register.
//
//*****************************************************************************
#define PWM_1_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
// Status
#define PWM_1_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_1_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
// Status
#define PWM_1_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_1_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_1_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_ISC register.
//
//*****************************************************************************
#define PWM_1_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_1_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_1_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_1_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_1_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_1_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_LOAD register.
//
//*****************************************************************************
#define PWM_1_LOAD_LOAD_M       0x0000FFFF  // Counter Load Value
#define PWM_1_LOAD_LOAD_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_COUNT register.
//
//*****************************************************************************
#define PWM_1_COUNT_COUNT_M     0x0000FFFF  // Counter Value
#define PWM_1_COUNT_COUNT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPA register.
//
//*****************************************************************************
#define PWM_1_CMPA_COMPA_M      0x0000FFFF  // Comparator A Value
#define PWM_1_CMPA_COMPA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPB register.
//
//*****************************************************************************
#define PWM_1_CMPB_COMPB_M      0x0000FFFF  // Comparator B Value
#define PWM_1_CMPB_COMPB_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENA register.
//
//*****************************************************************************
#define PWM_1_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_1_GENA_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_1_GENA_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_1_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_1_GENA_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_1_GENA_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_1_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_1_GENA_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_1_GENA_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_1_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_1_GENA_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_1_GENA_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_1_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_1_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_1_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_1_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_1_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_1_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_1_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_1_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_1_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_1_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENB register.
//
//*****************************************************************************
#define PWM_1_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_1_GENB_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_1_GENB_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_1_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_1_GENB_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_1_GENB_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_1_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_1_GENB_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_1_GENB_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_1_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_1_GENB_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_1_GENB_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_1_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_1_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_1_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_1_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_1_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_1_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_1_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_1_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_1_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_1_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBCTL register.
//
//*****************************************************************************
#define PWM_1_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBRISE register.
//
//*****************************************************************************
#define PWM_1_DBRISE_RISEDELAY_M                                              \
		0x00000FFF  // Dead-Band Rise Delay
#define PWM_1_DBRISE_RISEDELAY_S                                              \
		0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBFALL register.
//
//*****************************************************************************
#define PWM_1_DBFALL_FALLDELAY_M                                              \
		0x00000FFF  // Dead-Band Fall Delay
#define PWM_1_DBFALL_FALLDELAY_S                                              \
		0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_1_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_1_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_1_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_1_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_1_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_1_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_1_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_1_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_1_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_1_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_1_MINFLTPER_MFP_M   0x0000FFFF  // Minimum Fault Period
#define PWM_1_MINFLTPER_MFP_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CTL register.
//
//*****************************************************************************
#define PWM_2_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_2_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_2_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_2_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_2_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_2_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_2_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_2_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_2_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_2_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_2_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_2_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_2_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_2_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_2_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_2_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_2_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_2_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_2_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_2_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_2_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_2_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_2_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_2_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_2_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_2_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_2_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_2_CTL_MODE          0x00000002  // Counter Mode
#define PWM_2_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_INTEN register.
//
//*****************************************************************************
#define PWM_2_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
// Down
#define PWM_2_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_2_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
// Down
#define PWM_2_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_2_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_2_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_2_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
// Down
#define PWM_2_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
// Up
#define PWM_2_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
// Down
#define PWM_2_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
// Up
#define PWM_2_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_2_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_RIS register.
//
//*****************************************************************************
#define PWM_2_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
// Status
#define PWM_2_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_2_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
// Status
#define PWM_2_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_2_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_2_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_ISC register.
//
//*****************************************************************************
#define PWM_2_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_2_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_2_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_2_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_2_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_2_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_LOAD register.
//
//*****************************************************************************
#define PWM_2_LOAD_LOAD_M       0x0000FFFF  // Counter Load Value
#define PWM_2_LOAD_LOAD_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_COUNT register.
//
//*****************************************************************************
#define PWM_2_COUNT_COUNT_M     0x0000FFFF  // Counter Value
#define PWM_2_COUNT_COUNT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPA register.
//
//*****************************************************************************
#define PWM_2_CMPA_COMPA_M      0x0000FFFF  // Comparator A Value
#define PWM_2_CMPA_COMPA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPB register.
//
//*****************************************************************************
#define PWM_2_CMPB_COMPB_M      0x0000FFFF  // Comparator B Value
#define PWM_2_CMPB_COMPB_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENA register.
//
//*****************************************************************************
#define PWM_2_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_2_GENA_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_2_GENA_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_2_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_2_GENA_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_2_GENA_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_2_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_2_GENA_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_2_GENA_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_2_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_2_GENA_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_2_GENA_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_2_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_2_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_2_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_2_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_2_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_2_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_2_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_2_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_2_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_2_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENB register.
//
//*****************************************************************************
#define PWM_2_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_2_GENB_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_2_GENB_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_2_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_2_GENB_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_2_GENB_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_2_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_2_GENB_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_2_GENB_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_2_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_2_GENB_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_2_GENB_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_2_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_2_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_2_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_2_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_2_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_2_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_2_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_2_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_2_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_2_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBCTL register.
//
//*****************************************************************************
#define PWM_2_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBRISE register.
//
//*****************************************************************************
#define PWM_2_DBRISE_RISEDELAY_M                                              \
		0x00000FFF  // Dead-Band Rise Delay
#define PWM_2_DBRISE_RISEDELAY_S                                              \
		0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBFALL register.
//
//*****************************************************************************
#define PWM_2_DBFALL_FALLDELAY_M                                              \
		0x00000FFF  // Dead-Band Fall Delay
#define PWM_2_DBFALL_FALLDELAY_S                                              \
		0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_2_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_2_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_2_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_2_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_2_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_2_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_2_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_2_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_2_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_2_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_2_MINFLTPER_MFP_M   0x0000FFFF  // Minimum Fault Period
#define PWM_2_MINFLTPER_MFP_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CTL register.
//
//*****************************************************************************
#define PWM_3_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_3_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_3_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_3_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_3_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_3_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_3_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_3_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_3_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_3_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_3_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_3_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_3_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_3_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_3_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_3_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_3_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_3_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_3_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_3_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_3_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_3_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_3_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_3_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_3_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_3_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_3_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_3_CTL_MODE          0x00000002  // Counter Mode
#define PWM_3_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_INTEN register.
//
//*****************************************************************************
#define PWM_3_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
// Down
#define PWM_3_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_3_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
// Down
#define PWM_3_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_3_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_3_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_3_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
// Down
#define PWM_3_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
// Up
#define PWM_3_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
// Down
#define PWM_3_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
// Up
#define PWM_3_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_3_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_RIS register.
//
//*****************************************************************************
#define PWM_3_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
// Status
#define PWM_3_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_3_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
// Status
#define PWM_3_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_3_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_3_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_ISC register.
//
//*****************************************************************************
#define PWM_3_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_3_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_3_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_3_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_3_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_3_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_LOAD register.
//
//*****************************************************************************
#define PWM_3_LOAD_LOAD_M       0x0000FFFF  // Counter Load Value
#define PWM_3_LOAD_LOAD_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_COUNT register.
//
//*****************************************************************************
#define PWM_3_COUNT_COUNT_M     0x0000FFFF  // Counter Value
#define PWM_3_COUNT_COUNT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPA register.
//
//*****************************************************************************
#define PWM_3_CMPA_COMPA_M      0x0000FFFF  // Comparator A Value
#define PWM_3_CMPA_COMPA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPB register.
//
//*****************************************************************************
#define PWM_3_CMPB_COMPB_M      0x0000FFFF  // Comparator B Value
#define PWM_3_CMPB_COMPB_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENA register.
//
//*****************************************************************************
#define PWM_3_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_3_GENA_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_3_GENA_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_3_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_3_GENA_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_3_GENA_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_3_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_3_GENA_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_3_GENA_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_3_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_3_GENA_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_3_GENA_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_3_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_3_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_3_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_3_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_3_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_3_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_3_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_3_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_3_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_3_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENB register.
//
//*****************************************************************************
#define PWM_3_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_3_GENB_ACTCMPBD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_3_GENB_ACTCMPBD_ZERO                                              \
		0x00000800  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_3_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_3_GENB_ACTCMPBU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_3_GENB_ACTCMPBU_ZERO                                              \
		0x00000200  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_3_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_3_GENB_ACTCMPAD_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_3_GENB_ACTCMPAD_ZERO                                              \
		0x00000080  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_3_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_3_GENB_ACTCMPAU_NONE                                              \
		0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_3_GENB_ACTCMPAU_ZERO                                              \
		0x00000020  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_3_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_3_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_3_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_3_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_3_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_3_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_3_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_3_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_3_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_3_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBCTL register.
//
//*****************************************************************************
#define PWM_3_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBRISE register.
//
//*****************************************************************************
#define PWM_3_DBRISE_RISEDELAY_M                                              \
		0x00000FFF  // Dead-Band Rise Delay
#define PWM_3_DBRISE_RISEDELAY_S                                              \
		0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBFALL register.
//
//*****************************************************************************
#define PWM_3_DBFALL_FALLDELAY_M                                              \
		0x00000FFF  // Dead-Band Fall Delay
#define PWM_3_DBFALL_FALLDELAY_S                                              \
		0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_3_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_3_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_3_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_3_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_3_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_3_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_3_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_3_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_3_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_3_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_3_MINFLTPER_MFP_M   0x0000FFFF  // Minimum Fault Period
#define PWM_3_MINFLTPER_MFP_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSEN register.
//
//*****************************************************************************
#define PWM_0_FLTSEN_FAULT1     0x00000002  // Fault1 Sense
#define PWM_0_FLTSEN_FAULT0     0x00000001  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_0_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_0_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_0_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_0_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_0_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_0_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_0_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_0_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_0_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_0_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSEN register.
//
//*****************************************************************************
#define PWM_1_FLTSEN_FAULT1     0x00000002  // Fault1 Sense
#define PWM_1_FLTSEN_FAULT0     0x00000001  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_1_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_1_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_1_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_1_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_1_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_1_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_1_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_1_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_1_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_1_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_2_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_2_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_2_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_2_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_2_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_2_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_2_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_2_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_2_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_2_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_3_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_3_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_3_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_3_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_3_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_3_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_3_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_3_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_3_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_3_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_PP register.
//
//*****************************************************************************
#define PWM_PP_ONE              0x00000400  // One-Shot Mode
#define PWM_PP_EFAULT           0x00000200  // Extended Fault
#define PWM_PP_ESYNC            0x00000100  // Extended Synchronization
#define PWM_PP_FCNT_M           0x000000F0  // Fault Inputs (per PWM unit)
#define PWM_PP_GCNT_M           0x0000000F  // Generators
#define PWM_PP_FCNT_S           4
#define PWM_PP_GCNT_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_CTL register.
//
//*****************************************************************************
#define QEI_CTL_FILTCNT_M       0x000F0000  // Input Filter Prescale Count
#define QEI_CTL_FILTEN          0x00002000  // Enable Input Filter
#define QEI_CTL_STALLEN         0x00001000  // Stall QEI
#define QEI_CTL_INVI            0x00000800  // Invert Index Pulse
#define QEI_CTL_INVB            0x00000400  // Invert PhB
#define QEI_CTL_INVA            0x00000200  // Invert PhA
#define QEI_CTL_VELDIV_M        0x000001C0  // Predivide Velocity
#define QEI_CTL_VELDIV_1        0x00000000  // QEI clock /1
#define QEI_CTL_VELDIV_2        0x00000040  // QEI clock /2
#define QEI_CTL_VELDIV_4        0x00000080  // QEI clock /4
#define QEI_CTL_VELDIV_8        0x000000C0  // QEI clock /8
#define QEI_CTL_VELDIV_16       0x00000100  // QEI clock /16
#define QEI_CTL_VELDIV_32       0x00000140  // QEI clock /32
#define QEI_CTL_VELDIV_64       0x00000180  // QEI clock /64
#define QEI_CTL_VELDIV_128      0x000001C0  // QEI clock /128
#define QEI_CTL_VELEN           0x00000020  // Capture Velocity
#define QEI_CTL_RESMODE         0x00000010  // Reset Mode
#define QEI_CTL_CAPMODE         0x00000008  // Capture Mode
#define QEI_CTL_SIGMODE         0x00000004  // Signal Mode
#define QEI_CTL_SWAP            0x00000002  // Swap Signals
#define QEI_CTL_ENABLE          0x00000001  // Enable QEI
#define QEI_CTL_FILTCNT_S       16

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_STAT register.
//
//*****************************************************************************
#define QEI_STAT_DIRECTION      0x00000002  // Direction of Rotation
#define QEI_STAT_ERROR          0x00000001  // Error Detected

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_POS register.
//
//*****************************************************************************
#define QEI_POS_M               0xFFFFFFFF  // Current Position Integrator
// Value
#define QEI_POS_S               0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_MAXPOS register.
//
//*****************************************************************************
#define QEI_MAXPOS_M            0xFFFFFFFF  // Maximum Position Integrator
// Value
#define QEI_MAXPOS_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_LOAD register.
//
//*****************************************************************************
#define QEI_LOAD_M              0xFFFFFFFF  // Velocity Timer Load Value
#define QEI_LOAD_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_TIME register.
//
//*****************************************************************************
#define QEI_TIME_M              0xFFFFFFFF  // Velocity Timer Current Value
#define QEI_TIME_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_COUNT register.
//
//*****************************************************************************
#define QEI_COUNT_M             0xFFFFFFFF  // Velocity Pulse Count
#define QEI_COUNT_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_SPEED register.
//
//*****************************************************************************
#define QEI_SPEED_M             0xFFFFFFFF  // Velocity
#define QEI_SPEED_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_INTEN register.
//
//*****************************************************************************
#define QEI_INTEN_ERROR         0x00000008  // Phase Error Interrupt Enable
#define QEI_INTEN_DIR           0x00000004  // Direction Change Interrupt
// Enable
#define QEI_INTEN_TIMER         0x00000002  // Timer Expires Interrupt Enable
#define QEI_INTEN_INDEX         0x00000001  // Index Pulse Detected Interrupt
// Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_RIS register.
//
//*****************************************************************************
#define QEI_RIS_ERROR           0x00000008  // Phase Error Detected
#define QEI_RIS_DIR             0x00000004  // Direction Change Detected
#define QEI_RIS_TIMER           0x00000002  // Velocity Timer Expired
#define QEI_RIS_INDEX           0x00000001  // Index Pulse Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_ISC register.
//
//*****************************************************************************
#define QEI_ISC_ERROR           0x00000008  // Phase Error Interrupt
#define QEI_ISC_DIR             0x00000004  // Direction Change Interrupt
#define QEI_ISC_TIMER           0x00000002  // Velocity Timer Expired Interrupt
#define QEI_ISC_INDEX           0x00000001  // Index Pulse Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CFG register.
//
//*****************************************************************************
#define TIMER_CFG_M             0x00000007  // GPTM Configuration
#define TIMER_CFG_32_BIT_TIMER  0x00000000  // For a 16/32-bit timer, this
// value selects the 32-bit timer
// configuration
#define TIMER_CFG_32_BIT_RTC    0x00000001  // For a 16/32-bit timer, this
// value selects the 32-bit
// real-time clock (RTC) counter
// configuration
#define TIMER_CFG_16_BIT        0x00000004  // For a 16/32-bit timer, this
// value selects the 16-bit timer
// configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMR register.
//
//*****************************************************************************
#define TIMER_TAMR_TAPLO        0x00000800  // GPTM Timer A PWM Legacy
// Operation
#define TIMER_TAMR_TAMRSU       0x00000400  // GPTM Timer A Match Register
// Update
#define TIMER_TAMR_TAPWMIE      0x00000200  // GPTM Timer A PWM Interrupt
// Enable
#define TIMER_TAMR_TAILD        0x00000100  // GPTM Timer A Interval Load Write
#define TIMER_TAMR_TASNAPS      0x00000080  // GPTM Timer A Snap-Shot Mode
#define TIMER_TAMR_TAWOT        0x00000040  // GPTM Timer A Wait-on-Trigger
#define TIMER_TAMR_TAMIE        0x00000020  // GPTM Timer A Match Interrupt
// Enable
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAAMS        0x00000008  // GPTM Timer A Alternate Mode
// Select
#define TIMER_TAMR_TACMR        0x00000004  // GPTM Timer A Capture Mode
#define TIMER_TAMR_TAMR_M       0x00000003  // GPTM Timer A Mode
#define TIMER_TAMR_TAMR_1_SHOT  0x00000001  // One-Shot Timer mode
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_TAMR_TAMR_CAP     0x00000003  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMR register.
//
//*****************************************************************************
#define TIMER_TBMR_TBPLO        0x00000800  // GPTM Timer B PWM Legacy
// Operation
#define TIMER_TBMR_TBMRSU       0x00000400  // GPTM Timer B Match Register
// Update
#define TIMER_TBMR_TBPWMIE      0x00000200  // GPTM Timer B PWM Interrupt
// Enable
#define TIMER_TBMR_TBILD        0x00000100  // GPTM Timer B Interval Load Write
#define TIMER_TBMR_TBSNAPS      0x00000080  // GPTM Timer B Snap-Shot Mode
#define TIMER_TBMR_TBWOT        0x00000040  // GPTM Timer B Wait-on-Trigger
#define TIMER_TBMR_TBMIE        0x00000020  // GPTM Timer B Match Interrupt
// Enable
#define TIMER_TBMR_TBCDIR       0x00000010  // GPTM Timer B Count Direction
#define TIMER_TBMR_TBAMS        0x00000008  // GPTM Timer B Alternate Mode
// Select
#define TIMER_TBMR_TBCMR        0x00000004  // GPTM Timer B Capture Mode
#define TIMER_TBMR_TBMR_M       0x00000003  // GPTM Timer B Mode
#define TIMER_TBMR_TBMR_1_SHOT  0x00000001  // One-Shot Timer mode
#define TIMER_TBMR_TBMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_TBMR_TBMR_CAP     0x00000003  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CTL register.
//
//*****************************************************************************
#define TIMER_CTL_TBPWML        0x00004000  // GPTM Timer B PWM Output Level
#define TIMER_CTL_TBOTE         0x00002000  // GPTM Timer B Output Trigger
// Enable
#define TIMER_CTL_TBEVENT_M     0x00000C00  // GPTM Timer B Event Mode
#define TIMER_CTL_TBEVENT_POS   0x00000000  // Positive edge
#define TIMER_CTL_TBEVENT_NEG   0x00000400  // Negative edge
#define TIMER_CTL_TBEVENT_BOTH  0x00000C00  // Both edges
#define TIMER_CTL_TBSTALL       0x00000200  // GPTM Timer B Stall Enable
#define TIMER_CTL_TBEN          0x00000100  // GPTM Timer B Enable
#define TIMER_CTL_TAPWML        0x00000040  // GPTM Timer A PWM Output Level
#define TIMER_CTL_TAOTE         0x00000020  // GPTM Timer A Output Trigger
// Enable
#define TIMER_CTL_RTCEN         0x00000010  // GPTM RTC Stall Enable
#define TIMER_CTL_TAEVENT_M     0x0000000C  // GPTM Timer A Event Mode
#define TIMER_CTL_TAEVENT_POS   0x00000000  // Positive edge
#define TIMER_CTL_TAEVENT_NEG   0x00000004  // Negative edge
#define TIMER_CTL_TAEVENT_BOTH  0x0000000C  // Both edges
#define TIMER_CTL_TASTALL       0x00000002  // GPTM Timer A Stall Enable
#define TIMER_CTL_TAEN          0x00000001  // GPTM Timer A Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_SYNC register.
//
//*****************************************************************************
#define TIMER_SYNC_SYNCWT5_M    0x00C00000  // Synchronize GPTM 32/64-Bit Timer
// 5
#define TIMER_SYNC_SYNCWT5_NONE 0x00000000  // GPTM 32/64-Bit Timer 5 is not
// affected
#define TIMER_SYNC_SYNCWT5_TA   0x00400000  // A timeout event for Timer A of
// GPTM 32/64-Bit Timer 5 is
// triggered
#define TIMER_SYNC_SYNCWT5_TB   0x00800000  // A timeout event for Timer B of
// 