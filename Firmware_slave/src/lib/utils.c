/*
 * Copyright (C) 2017 EPFL-LSRO (Laboratoire de Systemes Robotiques).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "utils.h"

/**
 * @brief Endless loop function to stop the execution of the program here.
 * @note This function does nothing if CPU_TRAPS_ENABLED is set to zero.
 */
void utils_TrapCpu(void)
{
#if CPU_TRAPS_ENABLED
    while(1)
        ;
#endif
}

/**
  * @brief  "Busy wait" delay function 
  * @param  duration: Delay time in [us] (approximative value based on a 168MHz core clock)
  */
void utils_DelayUs(uint32_t duration)
{
	uint32_t i 	= 0;
	uint32_t j 	= 0;
    
	for(i=0; i<=duration; i++)
    {
		for(j=0; j<=52; j++)
        {
#ifdef __GNUC__
            asm("nop");
#else
            __asm{NOP};
#endif
		}
	}
}


/**
  * @brief  "Busy wait"  delay function 
  * @param  duration: Delay time in [ms] (approximative value based on a 168MHz core clock).
  * @note This delay is approximative, and may last longer if there are many interrupts.
  */
void utils_DelayMs(uint32_t duration)
{
	uint32_t i 	= 0;
	uint32_t j 	= 0;
	for(i=0; i<=duration; i++)
    {
		for(j=0; j<=33600; j++)
        {
#ifdef __GNUC__
            asm("nop");
#else
            __asm{NOP};
#endif
		}
	}
}


/**
  * @brief  Saturate a float number between two bounds.
  * @param  val: value to constrain between two limits.
  * @param	min: minimum
  * @param	max: maximum
  * @retval None.
  */
void utils_SaturateF(float32_t *val, float32_t min, float32_t max)
{
	if(*val < min)
		*val = min;
	else if(*val > max)
		*val = max;
}

/**
  * @brief  Saturate an integer number between two bounds.
  * @param  val: value to constrain between the two limits.
  * @param	min: lower limit.
  * @param	max: upper limit.
  * @retval None.
  */
void utils_SaturateU(uint32_t *val, uint32_t min, uint32_t max)
{
	if(*val < min)
		*val = min;
	else if(*val > max)
		*val = max;
}

/**
  * @brief  Compute the mean of the array values.
  * @param  array: array of float number to get the mean from.
  * @param	size: size of the array.
  * @retval the mean of the array values.
  */
float32_t utils_Mean(float32_t *array, int size)
{
    int i;
    float32_t sum = 0.0f;
    
    for(i=0; i<size; i++)
        sum += array[i];
    
    return sum / (float32_t) size;
}
