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

#include "button.h"
#include "../communication.h"

#define BOARD_BUTTON_PORT GPIOE
#define BOARD_BUTTON_PIN GPIO_Pin_2
#define USER_BUTTON_IRQ_CHANNEL EXTI2_IRQn
#define USER_BUTTON_IRQ_LINE EXTI_Line2
#define USER_BUTTON_IRQ_PINSOURCE EXTI_PinSource2
#define USER_BUTTON_IRQ_SOURCE EXTI_PortSourceGPIOE

void (*userButtonStateChangedCallback)(bool); ///< Function called when the button state changes. The bool parameter is true if the button was released.

/**
 * @brief Initializes the button module.
 * @param stateChangedCallback function pointer to the callback function that
 * should be called when the button state changes. If this feature is not used,
 * NULL can be passed instead.
 */
void but_Init(void (*stateChangedCallback)(bool))
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // Initialize the pin as input.
    GPIO_InitStruct.GPIO_Pin   = BOARD_BUTTON_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(BOARD_BUTTON_PORT, &GPIO_InitStruct);
    
    // Optionally setup the callback that triggers when the button is pressed or
    // released.
    userButtonStateChangedCallback = stateChangedCallback;
    
    if(stateChangedCallback != NULL)
    {
        SYSCFG_EXTILineConfig(USER_BUTTON_IRQ_SOURCE,
                              USER_BUTTON_IRQ_PINSOURCE);
        
        EXTI_InitStruct.EXTI_Line = USER_BUTTON_IRQ_LINE;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&EXTI_InitStruct);
     
        NVIC_InitStruct.NVIC_IRQChannel = USER_BUTTON_IRQ_CHANNEL;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = USER_BUTTON_IRQ_PRIORITY;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
    }
}

/**
 * @brief Gets the current state of the user button.
 * @return the state of the button (1=released, 0=pressed).
 */
bool but_GetState(void)
{  
    return GPIO_ReadInputDataBit(BOARD_BUTTON_PORT, BOARD_BUTTON_PIN) != 0;
}

/**
 * @brief Calls the user button callback function.
 * @remark: this function is called automatically by the pin change interrupt of
 * the user button.
 */
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(USER_BUTTON_IRQ_LINE) != RESET)
    {
        if(userButtonStateChangedCallback != NULL)
            userButtonStateChangedCallback(but_GetState());
        
        // Clear interrupt flag.
        EXTI_ClearITPendingBit(USER_BUTTON_IRQ_LINE);
    }
}
