 /******************************************************************************
 *
 * Module: Common - Platform Types Abstraction
 *
 * File Name: std_types.h
 *
 * Description: Standard Types for our MicroWave Project
 * 
 * Authors: Mostafa Mahmoud Ali & Mai Esmail Gamal
 *
 *******************************************************************************/

#ifndef STD_TYPES_H_
#define STD_TYPES_H_

#include <stdint.h>  

/* Boolean Data Type */
typedef unsigned char boolean;

/* Boolean Values */
#ifndef FALSE
#define FALSE       (0u)
#endif
#ifndef TRUE
#define TRUE        (1u)
#endif

#define LOGIC_HIGH        (1u)
#define LOGIC_LOW         (0u)

#define NULL_PTR    ((void*)0)

/*
 * Standard types already defined in <stdint.h> :
 
typedef unsigned char         uint8_t;          //           0 .. 255              
typedef unsigned short        uint16_t;         //           0 .. 65535            
typedef unsigned int         uint32_t;         //           0 .. 4294967295       
typedef unsigned long long    uint64_t;         //       0 .. 18446744073709551615

*/


typedef signed char           sint8_t;          /*        -128 .. +127             */
typedef signed short          sint16_t;         /*      -32768 .. +32767           */
typedef signed int           sint32_t;         /* -2147483648 .. +2147483647      */
typedef signed long long      sint64_t;         /* -9223372036854775808 .. 9223372036854775807 */
typedef float                 float32;
typedef double                float64;


#endif /* STD_TYPE_H_ */
