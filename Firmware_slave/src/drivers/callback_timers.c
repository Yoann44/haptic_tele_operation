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

#include "callback_timers.h"

#include "../lib/utils.h"
#include "../torque_regulator.h"
#include "../haptic_controller.h"

#define TE_LOOP_MIN_VALUE 50    // Minimum period for any loop [us].
#define TE_LOOP_MAX_VALUE 65534 // Maximum period for any loop [us].

#define TE_CURRENT_LOOP_DEFAULT_VAL 50   // Current control loop period [us] (max 2^16-1) (default value at reset).
#define TE_CONTROL_LOOP_DEFAULT_VAL 350  // Main control loop period [us] (max 2^16-1) (default value at reset).
#define TE_DATA_LOOP_DEFAULT_VAL    1000 // Data loop period [us] (max 2^16-1) (default value at reset).

cbt_PeriodicTaskFunc cbt_tim10Task, cbt_tim6Task, cbt_tim7Task;
volatile float32_t cbt_ucLoad; // Processor load (%).

void tim10InitFunc(void);
void tim67InitFunc(void);

/**
  * @brief  Initialize the timers to call an interrupt routine periodically.
  */
void cbt_Init(void)
{    
    // Initialize the timers.
    tim10InitFunc();
    tim67InitFunc();
    
    // Initialize the function pointers to NULL, in order to be able to test
    // if they are already affected or not.
    cbt_tim10Task = NULL;
    cbt_tim6Task = NULL;
    cbt_tim7Task = NULL;
}

/**
  * @brief  Set the function to call periodically by the timer 1.
  * @param  f: the function to call periodically.
  * @param  period: the period between each call of f [us].
  */
void cbt_SetCurrentLoopTimer(cbt_PeriodicTaskFunc f, uint32_t period)
{
    cbt_tim10Task = f;
    
    utils_SaturateU(&period, TE_LOOP_MIN_VALUE, TE_LOOP_MAX_VALUE);
    TIM10->ARR = (uint16_t) period;
}

/**
  * @brief  Set the function to call periodically by the timer 6.
  * @param  f: the function to call periodically.
  * @param  period: the period between each call of f [us].
  */
void cbt_SetHapticControllerTimer(cbt_PeriodicTaskFunc f, uint32_t period)
{
    cbt_tim6Task = f;
    
    utils_SaturateU(&period, TE_LOOP_MIN_VALUE, TE_LOOP_MAX_VALUE);
    TIM6->ARR = (uint16_t) period;
}

/**
  * @brief  Set the function to call periodically by the timer 7.
  * @param  f: the function to call periodically.
  * @param  period: the period between each call of f [us].
  */
void cbt_SetCommLoopTimer(cbt_PeriodicTaskFunc f, uint32_t period)
{
    cbt_tim7Task = f;
    
    utils_SaturateU(&period, TE_LOOP_MIN_VALUE, TE_LOOP_MAX_VALUE);
    TIM7->ARR = (uint16_t) period;
}

/**
  * @brief  Initialize TIM10 used for timing the current loop (resolution 1us).
  */
void tim10InitFunc(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    //TIM_OCInitTypeDef   TIM_OCInitStruct;

    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,  ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel                   = TIM1_UP_TIM10_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = CURRENT_LOOP_IRQ_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_TimeBaseStruct.TIM_Period        = (uint16_t)(TE_CURRENT_LOOP_DEFAULT_VAL-1);    
    TIM_TimeBaseStruct.TIM_Prescaler     = TIM10_PRESCALER;
    TIM_TimeBaseStruct.TIM_ClockDivision = 0; // TIM_CKD_DIV2;
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStruct);

    TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);      
    TIM_Cmd(TIM10, ENABLE);
}

/**
  * @brief  Initialize TIM6, for timing the main control loop (resolution 1us)
  *         Initialize TIM7, for timing the data transmission loop (resolution 1us)
  */
void tim67InitFunc(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    NVIC_InitTypeDef        NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6|RCC_APB1Periph_TIM7, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel                   = TIM6_DAC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = CONTROL_LOOP_IRQ_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    NVIC_InitStruct.NVIC_IRQChannel                   = TIM7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = DATA_LOOP_IRQ_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStruct);    

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Prescaler     = TIM6_PRESCALER;
    TIM_TimeBaseStruct.TIM_Period        = (uint16_t)(TE_CONTROL_LOOP_DEFAULT_VAL-1);
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Prescaler     = TIM7_PRESCALER;
    TIM_TimeBaseStruct.TIM_Period        = (uint16_t)(TE_DATA_LOOP_DEFAULT_VAL-1);
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStruct);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

/**
  * @brief  Interrupt from current control loop timer (TIM10)
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET)
    {
        if(cbt_tim10Task != NULL)
            cbt_tim10Task();
		
		TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
	}
}

/**
  * @brief  Interrupt from main control loop timer (TIM6)
  */
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        if(cbt_tim6Task != NULL)
            cbt_tim6Task();
        
        // Percentage of time consumed by the control task  0..100%.
		cbt_ucLoad = (((float32_t)(TIM6->CNT))*100)/((float32_t)(TIM6->ARR));

		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}


/**
  * @brief  Interrupt from data transmission loop timer (TIM7) (data loop)
  */
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
		if(cbt_tim7Task != NULL)
            cbt_tim7Task();
        
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

/**
  * @brief  Set the period of the position loop.
  * @param  period: the new period of the position loop [us].
  */
void cbt_SetHapticControllerPeriod(uint32_t period)
{
    utils_SaturateU(&period, TE_LOOP_MIN_VALUE, TE_LOOP_MAX_VALUE);
    TIM6->ARR = period;
}

/**
  * @brief  Set the period of the communication loop.
  * @param  period: the new period of the communication loop [us].
  */
void cbt_SetCommLoopPeriod(uint32_t period)
{
    utils_SaturateU(&period, TE_LOOP_MIN_VALUE, TE_LOOP_MAX_VALUE);
    TIM7->ARR = period;
}

/**
  * @brief  Get the period of the current loop.
  * @return the period of the current loop [us].
  */
uint32_t cbt_GetCurrentLoopPeriod(void)
{
    return TIM10->ARR;
}

/**
  * @brief  Get the period of the position loop.
  * @return the period of the position loop [us].
  */
uint32_t cbt_GetHapticControllerPeriod(void)
{
    return TIM6->ARR;
}

/**
  * @brief  Get the period of the communication loop.
  * @return the period of the communication loop [us].
  */
uint32_t cbt_GetCommLoopPeriod(void)
{
    return TIM7->ARR;
}
