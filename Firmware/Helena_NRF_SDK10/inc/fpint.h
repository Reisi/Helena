/**
  ******************************************************************************
  * @file    fpint.h
  * @author  Thomas Reisnecker
  * @brief   This header provides fix point data types
  *
  * @note    types have the format q[i_]f_t, where
  *          i is the number of integral digits (optional if i = 0),
  *          f is the number of fractional digits,
  *          if i+f is odd, it is a signed data type
  *          if i+f is even, it is a unsigned data type
  *          if i+f = 7/8 the data type is 8 bits wide
  *          if i+f = 15/16 the data type is 16 bits wide
  *          ...
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FPINT_H_INCLUDED
#define FPINT_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef int8_t q7_t;
typedef int8_t q1_6_t;
typedef int8_t q2_5_t;
typedef int8_t q3_4_t;
typedef int8_t q4_3_t;
typedef int8_t q5_2_t;
typedef int8_t q6_1_t;

typedef uint8_t q8_t;
typedef uint8_t q1_7_t;
typedef uint8_t q2_6_t;
typedef uint8_t q3_5_t;
typedef uint8_t q4_4_t;
typedef uint8_t q5_3_t;
typedef uint8_t q6_2_t;
typedef uint8_t q7_1_t;

typedef int16_t q15_t;
typedef int16_t q1_14_t;
typedef int16_t q2_13_t;
typedef int16_t q3_12_t;
typedef int16_t q4_11_t;
typedef int16_t q5_10_t;
typedef int16_t q6_9_t;
typedef int16_t q7_8_t;
typedef int16_t q8_7_t;
typedef int16_t q9_6_t;
typedef int16_t q10_5_t;
typedef int16_t q11_4_t;
typedef int16_t q12_3_t;
typedef int16_t q13_2_t;
typedef int16_t q14_1_t;

typedef uint16_t q16_t;
typedef uint16_t q1_15_t;
typedef uint16_t q2_14_t;
typedef uint16_t q3_13_t;
typedef uint16_t q4_12_t;
typedef uint16_t q5_11_t;
typedef uint16_t q6_10_t;
typedef uint16_t q7_9_t;
typedef uint16_t q8_8_t;
typedef uint16_t q9_7_t;
typedef uint16_t q10_6_t;
typedef uint16_t q11_5_t;
typedef uint16_t q12_4_t;
typedef uint16_t q13_3_t;
typedef uint16_t q14_2_t;
typedef uint16_t q15_1_t;

typedef int32_t q31_t;
typedef int32_t q1_30_t;
typedef int32_t q2_29_t;
typedef int32_t q3_28_t;
typedef int32_t q4_27_t;
typedef int32_t q5_26_t;
typedef int32_t q6_25_t;
typedef int32_t q7_24_t;
typedef int32_t q8_23_t;
typedef int32_t q9_22_t;
typedef int32_t q10_21_t;
typedef int32_t q11_20_t;
typedef int32_t q12_19_t;
typedef int32_t q13_18_t;
typedef int32_t q14_17_t;
typedef int32_t q15_16_t;
typedef int32_t q16_15_t;
typedef int32_t q17_14_t;
typedef int32_t q18_13_t;
typedef int32_t q19_12_t;
typedef int32_t q20_11_t;
typedef int32_t q21_10_t;
typedef int32_t q22_9_t;
typedef int32_t q23_8_t;
typedef int32_t q24_7_t;
typedef int32_t q25_6_t;
typedef int32_t q26_5_t;
typedef int32_t q27_4_t;
typedef int32_t q28_3_t;
typedef int32_t q29_2_t;
typedef int32_t q30_1_t;

typedef uint32_t q32_t;
typedef uint32_t q1_31_t;
typedef uint32_t q2_30_t;
typedef uint32_t q3_29_t;
typedef uint32_t q4_28_t;
typedef uint32_t q5_27_t;
typedef uint32_t q6_26_t;
typedef uint32_t q7_25_t;
typedef uint32_t q8_24_t;
typedef uint32_t q9_23_t;
typedef uint32_t q10_22_t;
typedef uint32_t q11_21_t;
typedef uint32_t q12_20_t;
typedef uint32_t q13_19_t;
typedef uint32_t q14_18_t;
typedef uint32_t q15_17_t;
typedef uint32_t q16_16_t;
typedef uint32_t q17_15_t;
typedef uint32_t q18_14_t;
typedef uint32_t q19_13_t;
typedef uint32_t q20_12_t;
typedef uint32_t q21_11_t;
typedef uint32_t q22_10_t;
typedef uint32_t q23_9_t;
typedef uint32_t q24_8_t;
typedef uint32_t q25_7_t;
typedef uint32_t q26_6_t;
typedef uint32_t q27_5_t;
typedef uint32_t q28_4_t;
typedef uint32_t q29_3_t;
typedef uint32_t q30_2_t;
typedef uint32_t q31_1_t;

#endif // FPINT_H_INCLUDED

/**END OF FILE*****************************************************************/
