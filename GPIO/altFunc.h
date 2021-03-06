/******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: altFunc.h
 *
 * Description: Header file for pins' alternate functions
 *
 * Author: Mai Esmail Gamal
 *
 *******************************************************************************/

#ifndef ALTFUNC_H_
#define ALTFUNC_H_

#define GPIO_PORT_MODE       0
/*******************************************************************************
 *                      		PORTA Pins                                     *
 *******************************************************************************/
#define PA7_M         0xF0000000  // PA7 Mask
#define PA7_I2C1SDA   0x30000000  // I2C1SDA on PA7
#define PA7_M1PWM3    0x50000000  // M1PWM3 on PA7
#define PA6_M         0x0F000000  // PA6 Mask
#define PA6_I2C1SCL   0x03000000  // I2C1SCL on PA6
#define PA6_M1PWM2    0x05000000  // M1PWM2 on PA6
#define PA5_M         0x00F00000  // PA5 Mask
#define PA5_SSI0TX    0x00200000  // SSI0TX on PA5
#define PA4_M         0x000F0000  // PA4 Mask
#define PA4_SSI0RX    0x00020000  // SSI0RX on PA4
#define PA3_M         0x0000F000  // PA3 Mask
#define PA3_SSI0FSS   0x00002000  // SSI0FSS on PA3
#define PA2_M         0x00000F00  // PA2 Mask
#define PA2_SSI0CLK   0x00000200  // SSI0CLK on PA2
#define PA1_M         0x000000F0  // PA1 Mask
#define PA1_U0TX      0x00000010  // U0TX on PA1
#define PA1_CAN1TX    0x00000080  // CAN1TX on PA1
#define PA0_M         0x0000000F  // PA0 Mask
#define PA0_U0RX      0x00000001  // U0RX on PA0
#define PA0_CAN1RX    0x00000008  // CAN1RX on PA0

/*******************************************************************************
 *                      		PORTB Pins                                     *
 *******************************************************************************/
#define PB7_M         0xF0000000  // PB7 Mask
#define PB7_SSI2TX    0x20000000  // SSI2TX on PB7
#define PB7_M0PWM1    0x40000000  // M0PWM1 on PB7
#define PB7_T0CCP1    0x70000000  // T0CCP1 on PB7
#define PB6_M         0x0F000000  // PB6 Mask
#define PB6_SSI2RX    0x02000000  // SSI2RX on PB6
#define PB6_M0PWM0    0x04000000  // M0PWM0 on PB6
#define PB6_T0CCP0    0x07000000  // T0CCP0 on PB6
#define PB5_M         0x00F00000  // PB5 Mask
#define PB5_SSI2FSS   0x00200000  // SSI2FSS on PB5
#define PB5_M0PWM3    0x00400000  // M0PWM3 on PB5
#define PB5_T1CCP1    0x00700000  // T1CCP1 on PB5
#define PB5_CAN0TX    0x00800000  // CAN0TX on PB5
#define PB4_M         0x000F0000  // PB4 Mask
#define PB4_SSI2CLK   0x00020000  // SSI2CLK on PB4
#define PB4_M0PWM2    0x00040000  // M0PWM2 on PB4
#define PB4_T1CCP0    0x00070000  // T1CCP0 on PB4
#define PB4_CAN0RX    0x00080000  // CAN0RX on PB4
#define PB3_M         0x0000F000  // PB3 Mask
#define PB3_I2C0SDA   0x00003000  // I2C0SDA on PB3
#define PB3_T3CCP1    0x00007000  // T3CCP1 on PB3
#define PB2_M         0x00000F00  // PB2 Mask
#define PB2_I2C0SCL   0x00000300  // I2C0SCL on PB2
#define PB2_T3CCP0    0x00000700  // T3CCP0 on PB2
#define PB1_M         0x000000F0  // PB1 Mask
#define PB1_USB0VBUS  0x00000000  // USB0VBUS on PB1
#define PB1_U1TX      0x00000010  // U1TX on PB1
#define PB1_T2CCP1    0x00000070  // T2CCP1 on PB1
#define PB0_M         0x0000000F  // PB0 Mask
#define PB0_USB0ID    0x00000000  // USB0ID on PB0
#define PB0_U1RX      0x00000001  // U1RX on PB0
#define PB0_T2CCP0    0x00000007  // T2CCP0 on PB0

/*******************************************************************************
 *                      		PORTC Pins                                     *
 *******************************************************************************/
#define PC7_M         0xF0000000  // PC7 Mask
#define PC7_U3TX      0x10000000  // U3TX on PC7
#define PC7_WT1CCP1   0x70000000  // WT1CCP1 on PC7
#define PC7_USB0PFLT  0x80000000  // USB0PFLT on PC7
#define PC6_M         0x0F000000  // PC6 Mask
#define PC6_U3RX      0x01000000  // U3RX on PC6
#define PC6_PHB1      0x06000000  // PHB1 on PC6
#define PC6_WT1CCP0   0x07000000  // WT1CCP0 on PC6
#define PC6_USB0EPEN  0x08000000  // USB0EPEN on PC6
#define PC5_M         0x00F00000  // PC5 Mask
#define PC5_U4TX      0x00100000  // U4TX on PC5
#define PC5_U1TX      0x00200000  // U1TX on PC5
#define PC5_M0PWM7    0x00400000  // M0PWM7 on PC5
#define PC5_PHA1      0x00600000  // PHA1 on PC5
#define PC5_WT0CCP1   0x00700000  // WT0CCP1 on PC5
#define PC5_U1CTS     0x00800000  // U1CTS on PC5
#define PC4_M         0x000F0000  // PC4 Mask
#define PC4_U4RX      0x00010000  // U4RX on PC4
#define PC4_U1RX      0x00020000  // U1RX on PC4
#define PC4_M0PWM6    0x00040000  // M0PWM6 on PC4
#define PC4_IDX1      0x00060000  // IDX1 on PC4
#define PC4_WT0CCP0   0x00070000  // WT0CCP0 on PC4
#define PC4_U1RTS     0x00080000  // U1RTS on PC4
#define PC3_M         0x0000F000  // PC3 Mask
#define PC3_TDO       0x00001000  // TDO on PC3
#define PC3_T5CCP1    0x00007000  // T5CCP1 on PC3
#define PC2_M         0x00000F00  // PC2 Mask
#define PC2_TDI       0x00000100  // TDI on PC2
#define PC2_T5CCP0    0x00000700  // T5CCP0 on PC2
#define PC1_M         0x000000F0  // PC1 Mask
#define PC1_TMS       0x00000010  // TMS on PC1
#define PC1_T4CCP1    0x00000070  // T4CCP1 on PC1
#define PC0_M         0x0000000F  // PC0 Mask
#define PC0_TCK       0x00000001  // TCK on PC0
#define PC0_T4CCP0    0x00000007  // T4CCP0 on PC0

/*******************************************************************************
 *                      		PORTD Pins                                     *
 *******************************************************************************/
#define PD7_M         0xF0000000  // PD7 Mask
#define PD7_U2TX      0x10000000  // U2TX on PD7
#define PD7_PHB0      0x60000000  // PHB0 on PD7
#define PD7_WT5CCP1   0x70000000  // WT5CCP1 on PD7
#define PD7_NMI       0x80000000  // NMI on PD7
#define PD6_M         0x0F000000  // PD6 Mask
#define PD6_U2RX      0x01000000  // U2RX on PD6
#define PD6_M0FAULT0  0x04000000  // M0FAULT0 on PD6
#define PD6_PHA0      0x06000000  // PHA0 on PD6
#define PD6_WT5CCP0   0x07000000  // WT5CCP0 on PD6
#define PD5_M         0x00F00000  // PD5 Mask
#define PD5_USB0DP    0x00000000  // USB0DP on PD5
#define PD5_U6TX      0x00100000  // U6TX on PD5
#define PD5_WT4CCP1   0x00700000  // WT4CCP1 on PD5
#define PD4_M         0x000F0000  // PD4 Mask
#define PD4_USB0DM    0x00000000  // USB0DM on PD4
#define PD4_U6RX      0x00010000  // U6RX on PD4
#define PD4_WT4CCP0   0x00070000  // WT4CCP0 on PD4
#define PD3_M         0x0000F000  // PD3 Mask
#define PD3_AIN4      0x00000000  // AIN4 on PD3
#define PD3_SSI3TX    0x00001000  // SSI3TX on PD3
#define PD3_SSI1TX    0x00002000  // SSI1TX on PD3
#define PD3_IDX0      0x00006000  // IDX0 on PD3
#define PD3_WT3CCP1   0x00007000  // WT3CCP1 on PD3
#define PD3_USB0PFLT  0x00008000  // USB0PFLT on PD3
#define PD2_M         0x00000F00  // PD2 Mask
#define PD2_AIN5      0x00000000  // AIN5 on PD2
#define PD2_SSI3RX    0x00000100  // SSI3RX on PD2
#define PD2_SSI1RX    0x00000200  // SSI1RX on PD2
#define PD2_M0FAULT0  0x00000400  // M0FAULT0 on PD2
#define PD2_WT3CCP0   0x00000700  // WT3CCP0 on PD2
#define PD2_USB0EPEN  0x00000800  // USB0EPEN on PD2
#define PD1_M         0x000000F0  // PD1 Mask
#define PD1_AIN6      0x00000000  // AIN6 on PD1
#define PD1_SSI3FSS   0x00000010  // SSI3FSS on PD1
#define PD1_SSI1FSS   0x00000020  // SSI1FSS on PD1
#define PD1_I2C3SDA   0x00000030  // I2C3SDA on PD1
#define PD1_M0PWM7    0x00000040  // M0PWM7 on PD1
#define PD1_M1PWM1    0x00000050  // M1PWM1 on PD1
#define PD1_WT2CCP1   0x00000070  // WT2CCP1 on PD1
#define PD0_M         0x0000000F  // PD0 Mask
#define PD0_AIN7      0x00000000  // AIN7 on PD0
#define PD0_SSI3CLK   0x00000001  // SSI3CLK on PD0
#define PD0_SSI1CLK   0x00000002  // SSI1CLK on PD0
#define PD0_I2C3SCL   0x00000003  // I2C3SCL on PD0
#define PD0_M0PWM6    0x00000004  // M0PWM6 on PD0
#define PD0_M1PWM0    0x00000005  // M1PWM0 on PD0
#define PD0_WT2CCP0   0x00000007  // WT2CCP0 on PD0

/*******************************************************************************
 *                      		PORTE Pins                                     *
 *******************************************************************************/
#define PE5_M         0x00F00000  // PE5 Mask
#define PE5_AIN8      0x00000000  // AIN8 on PE5
#define PE5_U5TX      0x00100000  // U5TX on PE5
#define PE5_I2C2SDA   0x00300000  // I2C2SDA on PE5
#define PE5_M0PWM5    0x00400000  // M0PWM5 on PE5
#define PE5_M1PWM3    0x00500000  // M1PWM3 on PE5
#define PE5_CAN0TX    0x00800000  // CAN0TX on PE5
#define PE4_M         0x000F0000  // PE4 Mask
#define PE4_AIN9      0x00000000  // AIN9 on PE4
#define PE4_U5RX      0x00010000  // U5RX on PE4
#define PE4_I2C2SCL   0x00030000  // I2C2SCL on PE4
#define PE4_M0PWM4    0x00040000  // M0PWM4 on PE4
#define PE4_M1PWM2    0x00050000  // M1PWM2 on PE4
#define PE4_CAN0RX    0x00080000  // CAN0RX on PE4
#define PE3_M         0x0000F000  // PE3 Mask
#define PE3_AIN0      0x00000000  // AIN0 on PE3
#define PE2_M         0x00000F00  // PE2 Mask
#define PE2_AIN1      0x00000000  // AIN1 on PE2
#define PE1_M         0x000000F0  // PE1 Mask
#define PE1_AIN2      0x00000000  // AIN2 on PE1
#define PE1_U7TX      0x00000010  // U7TX on PE1
#define PE0_M         0x0000000F  // PE0 Mask
#define PE0_AIN3      0x00000000  // AIN3 on PE0
#define PE0_U7RX      0x00000001  // U7RX on PE0


/*******************************************************************************
 *                      		PORTF Pins                                     *
 *******************************************************************************/

#define PF4_M         0x000F0000  // PF4 Mask
#define PF4_M1FAULT0  0x00050000  // M1FAULT0 on PF4
#define PF4_IDX0      0x00060000  // IDX0 on PF4
#define PF4_T2CCP0    0x00070000  // T2CCP0 on PF4
#define PF4_USB0EPEN  0x00080000  // USB0EPEN on PF4
#define PF3_M         0x0000F000  // PF3 Mask
#define PF3_SSI1FSS   0x00002000  // SSI1FSS on PF3
#define PF3_CAN0TX    0x00003000  // CAN0TX on PF3
#define PF3_M1PWM7    0x00005000  // M1PWM7 on PF3
#define PF3_T1CCP1    0x00007000  // T1CCP1 on PF3
#define PF3_TRCLK     0x0000E000  // TRCLK on PF3
#define PF2_M         0x00000F00  // PF2 Mask
#define PF2_SSI1CLK   0x00000200  // SSI1CLK on PF2
#define PF2_M0FAULT0  0x00000400  // M0FAULT0 on PF2
#define PF2_M1PWM6    0x00000500  // M1PWM6 on PF2
#define PF2_T1CCP0    0x00000700  // T1CCP0 on PF2
#define PF2_TRD0      0x00000E00  // TRD0 on PF2
#define PF1_M         0x000000F0  // PF1 Mask
#define PF1_U1CTS     0x00000010  // U1CTS on PF1
#define PF1_SSI1TX    0x00000020  // SSI1TX on PF1
#define PF1_M1PWM5    0x00000050  // M1PWM5 on PF1
#define PF1_PHB0      0x00000060  // PHB0 on PF1
#define PF1_T0CCP1    0x00000070  // T0CCP1 on PF1
#define PF1_C1O       0x00000090  // C1O on PF1
#define PF1_TRD1      0x000000E0  // TRD1 on PF1
#define PF0_M         0x0000000F  // PF0 Mask
#define PF0_U1RTS     0x00000001  // U1RTS on PF0
#define PF0_SSI1RX    0x00000002  // SSI1RX on PF0
#define PF0_CAN0RX    0x00000003  // CAN0RX on PF0
#define PF0_M1PWM4    0x00000005  // M1PWM4 on PF0
#define PF0_PHA0      0x00000006  // PHA0 on PF0
#define PF0_T0CCP0    0x00000007  // T0CCP0 on PF0
#define PF0_NMI       0x00000008  // NMI on PF0
#define PF0_C0O       0x00000009  // C0O on PF0

#endif /* ALTFUNC_H_ */