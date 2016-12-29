/**
  ******************************************************************************
  * @file    ?.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    ?
  * @brief   ?
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fastmath.h"
#include "arm_common_tables.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
q15_t arm_asin_q15(q15_t y)
{
    int_fast8_t sign, i_search, i_step;

    sign = y < 0 ? -1 : 1;
    y *= sign;

    i_search = FAST_MATH_TABLE_SIZE / 8;
    i_step   = i_search >> 1;

    while(i_step > 1)
    {
        i_search += y >= sinTable_q15[i_search] ? i_step : -i_step;
        i_step >>= 1;
    }
    if (y < sinTable_q15[i_search])
        i_search--;

    q15_t angle;
    angle = i_search << FAST_MATH_Q15_SHIFT;
    angle += ((y - sinTable_q15[i_search]) << FAST_MATH_Q15_SHIFT) / (sinTable_q15[i_search+1] - sinTable_q15[i_search]);
    if (sign == -1)
        angle = ((int32_t)1<<15) - angle;
    return angle;
}

/********************************************//**
 * @brief Fast approximation to the trigonometric arcustangens2 function for Q15 data.
 *
 * @param[in] y y-axis input value
 * @param[in] x x-axis input value
 * @return angle
 *
 * The Q15 output value is in the range [0 +0.9999] and is mapped to a radian
 * value in the range [0 2*pi).
 *
 ***********************************************/
q15_t arm_atan2_q15(int16_t y, int16_t x)
{
    int_fast16_t index_min, index_max;
    int32_t diff_min, diff_max;

    // check special case when x = 0
    if (x == 0)
    {
        if (y > 0)
            return 0x2000;
        else if (y < 0)
            return 0x6000;
        else
            return 0;
    }
    // otherwise check the quadrant the result has to be
    else if (x > 0)
    {
        if (y >= 0) // first quadrant
        {
            index_min = 0 >> FAST_MATH_Q15_SHIFT;
            index_max = 0x2000 >> FAST_MATH_Q15_SHIFT;
        }
        else        //fourth quadrant
        {
            index_min = 0x6000 >> FAST_MATH_Q15_SHIFT;
            index_max = 0x8000l >> FAST_MATH_Q15_SHIFT;
        }
    }
    else
    {
        if (y >= 0) // second quadrant
        {
            index_min = 0x2000 >> FAST_MATH_Q15_SHIFT;
            index_max = 0x4000 >> FAST_MATH_Q15_SHIFT;
        }
        else        // third quadrant
        {
            index_min = 0x4000 >> FAST_MATH_Q15_SHIFT;
            index_max = 0x6000 >> FAST_MATH_Q15_SHIFT;
        }
    }

    // now search the best table entry fitting the input parameters.
    // therefore the product of y * cos(alpha) is subtracted of x * sin(alpha),
    // this difference is negative if the angle is too low and positive if too high

    // calculate differences for minimum and maximum values
    diff_min = (int32_t)x * sinTable_q15[index_min] - (int32_t)y * sinTable_q15[(index_min + (0x2000 >> FAST_MATH_Q15_SHIFT)) % FAST_MATH_TABLE_SIZE];
    diff_max = (int32_t)x * sinTable_q15[index_max] - (int32_t)y * sinTable_q15[(index_max + (0x2000 >> FAST_MATH_Q15_SHIFT)) % FAST_MATH_TABLE_SIZE];

    // searching for best fit by starting in the middle, if this angle is to low/high
    // replace max/min and start over, this way the best fitting angle pair is found
    // after 7 steps.
    while(1)
    {
        int_fast16_t index_check = (index_min + index_max) >> 1;
        int32_t diff = (int32_t)x * sinTable_q15[index_check] - (int32_t)y * sinTable_q15[(index_check + (0x2000 >> FAST_MATH_Q15_SHIFT)) % FAST_MATH_TABLE_SIZE];
        if (diff > 0)
        {
            index_max = index_check;
            diff_max = diff;
        }
        else
        {
            index_min = index_check;
            diff_min = diff;
        }
        // check if best fitting angle pair is found and calculate fractional part with linear approximation
        if (index_min + 1 == index_max)
        {
            index_check <<= FAST_MATH_Q15_SHIFT;
            index_check += (-diff_min << FAST_MATH_Q15_SHIFT) / (diff_max - diff_min);
            return index_check;
        }
    }

}

/**END OF FILE*****************************************************************/



