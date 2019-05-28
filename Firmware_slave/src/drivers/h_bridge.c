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

#include "h_bridge.h"

void hb_Tim8Init(void);

/**
  * @brief  Initialize the pins and the PWM timer to control the H-bridge.
  */
void hb_Init(void)
{
    // Init the GPIO pins.
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin  = nFAULT_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(nFAULT_Port, &GPIO_InitStruct);   

    GPIO_InitStruct.GPIO_Pin   = nSLEEP_Pin;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(nSLEEP_Port, &GPIO_InitStruct);   
    GPIO_WriteBit(nSLEEP_Port, nSLEEP_Pin, Bit_SET);

    GPIO_InitStruct.GPIO_Pin = (uint16_t)(1<<ENB1_Pin);  
    GPIO_Init(ENB1_Port, &GPIO_InitStruct); 
    GPIO_WriteBit(ENB1_Port, (uint16_t)(1<<ENB1_Pin), Bit_RESET);

    GPIO_InitStruct.GPIO_Pin = (uint16_t)(1<<ENB2_Pin);
    GPIO_Init(ENB2_Port, &GPIO_InitStruct);
    GPIO_WriteBit(ENB2_Port, (uint16_t)(1<<ENB2_Pin), Bit_RESET);
    
    // Init the timer 8, that generates the PWM signal.
    hb_Tim8Init();
}

/**
  * @brief  Initialize TIM8 as a PWM generator.
  */
void hb_Tim8Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef       TIM_OCInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,  ENABLE);
    
    GPIO_InitStruct.GPIO_Pin   = (uint16_t)(1<<PWM1_Pin);
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(PWM1_Port, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = (uint16_t)(1<<PWM2_Pin);
    GPIO_Init(PWM2_Port, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(PWM1_Port, (uint8_t)PWM1_Pin,  GPIO_AF_TIM8);
    GPIO_PinAFConfig(PWM2_Port, (uint8_t)PWM2_Pin,  GPIO_AF_TIM8);
    
    TIM_TimeBaseStruct.TIM_Period            = PWM_TIM_PERIODE;  
    TIM_TimeBaseStruct.TIM_Prescaler         = PWM_TIM_PRESCALER;
    TIM_TimeBaseStruct.TIM_ClockDivision     = 0;
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStruct);

    
    TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse       = ((int16_t)(PWM_TIM_PERIODE>>1)); //50% duty cycle
    TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM8, &TIM_OCInitStruct);
    
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC2Init(TIM8, &TIM_OCInitStruct);
    
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

    // TIM8 TRGO selection.
    TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);

    TIM_Cmd(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
}


/**
  * @brief  Enable the motor driver.
  */
void hb_Enable()
{
    GPIO_WriteBit(ENB1_Port, (uint16_t)(1<<ENB1_Pin), Bit_SET);
    GPIO_WriteBit(ENB2_Port, (uint16_t)(1<<ENB2_Pin), Bit_SET);
}

/**
  * @brief  Disable the motor driver.
  */
void hb_Disable()
{
    GPIO_WriteBit(ENB1_Port, (uint16_t)(1<<ENB1_Pin), Bit_RESET);
    GPIO_WriteBit(ENB2_Port, (uint16_t)(1<<ENB2_Pin), Bit_RESET);
}

/**
  * @brief  Set the PWM duty.
  * @param  ratio: PWM duty ("on" time ratio) [-1.0 to 1.0]. 0 means zero
  * voltage at the motor leads, -1 and 1 are max voltages.
  */
void hb_SetPWM(float32_t ratio)
{
    uint16_t pwmDutyCycle;

    pwmDutyCycle = ((uint16_t)((ratio+1)*((float32_t)((TIM8->ARR)>>1))));
        
    TIM8->CCR1 = pwmDutyCycle; // The same value is affected to both timers, because they have
    TIM8->CCR2 = pwmDutyCycle; // an opposite polarity.
}

/**
  * @brief Gets the fault state.
  * Gets the fault state of the H-Bridge from its nFAULT line. A fault may be
  * because of an undervoltage, overcurrent or overtemperature.
  * @return true if there is a fault, false otherwise.
  */
bool hb_HasFault(void)
{
    return !GPIO_ReadInputDataBit(nFAULT_Port, nFAULT_Pin);
}
