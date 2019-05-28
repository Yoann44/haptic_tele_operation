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

#ifndef __CALLBACK_TIMERS_H
#define __CALLBACK_TIMERS_H

#include "../main.h"

typedef void (*cbt_PeriodicTaskFunc)(void);

// TIMX_PERIOD is the clock divider at the input of the timers.
// So, the timer will increment its counter, every TIMX_PERIOD ticks of the system clock (168 MHz).
#define TIM10_PRESCALER ((uint16_t)(SystemCoreClock/APB2_PRESCALER*TIM_MULTIPLIER/1000000-1)) // CLK_CNT = 1[us] (current loop)
#define TIM6_PRESCALER ((uint16_t)(SystemCoreClock/APB1_PRESCALER*TIM_MULTIPLIER/1000000-1)) // CLK_CNT = 1[us] (control loop)
#define TIM7_PRESCALER ((uint16_t)(SystemCoreClock/APB1_PRESCALER*TIM_MULTIPLIER/1000000-1)) // CLK_CNT = 1[us] (data loop)

/** @defgroup CallbackTimers Driver / Callback timers
  * @brief Driver to call functions at a fixed rate.
  *
  * This driver setups three timers of the STM32, in order to call at a precise
  * rate the control functions: the current regulation loop, the position
  * regulation loop and the communication loop (data streaming part only).
  *
  * In the initialization code, first call cbt_Init(). Then call each
  * cbt_Set*LoopTimer() function, giving the pointer to the function to call as
  * an argument.
  *
  * @addtogroup CallbackTimers
  * @{
  */

void cbt_Init(void);
void cbt_SetCurrentLoopTimer(cbt_PeriodicTaskFunc f, uint32_t period);
void cbt_SetHapticControllerTimer(cbt_PeriodicTaskFunc f, uint32_t period);
void cbt_SetCommLoopTimer(cbt_PeriodicTaskFunc f, uint32_t period);
void cbt_SetHapticControllerPeriod(uint32_t period);
void cbt_SetCommLoopPeriod(uint32_t period);
uint32_t cbt_GetCurrentLoopPeriod(void);
uint32_t cbt_GetHapticControllerPeriod(void);
uint32_t cbt_GetCommLoopPeriod(void);

/**
  * @}
  */

#endif
