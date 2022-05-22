 /******************************************************************************
 *
 * Module: Common - Platform Types Abstraction
 *
 * File Name: std_types.h
 *
 * Description: Standard Types for our MicroWave Project
 * 
 * Authors: Mostafa Mahmoud Ali 
 *
 *******************************************************************************/

#ifndef STD_TYPES_H_
#define STD_TYPES_H_


/* Boolean Data Type */
typedef unsigned char boolean;

/* Boolean Values */
#ifndef FALSE
#define FALSE       0
#endif
#ifndef TRUE
#define TRUE        1
#endif

#define LOGIC_HIGH        1
#define LOGIC_LOW         0

#define NULL_PTR    ((void*)0)

typedef signed char           sint8_t;          /*        -128 .. +127             */
typedef signed short          sint16_t;         /*      -32768 .. +32767           */
typedef signed long           sint32_t;         /* -2147483648 .. +2147483647      */
typedef signed long long      sint64_t;         /* -9223372036854775808 .. 9223372036854775807 */
typedef float                 float32;
typedef double                float64;

#endif /* STD_TYPE_H_ */
