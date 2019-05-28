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

#include "incr_encoder.h"

void tim5InitFunc(void);
void extitInitFunc(void);

float32_t enc_offset; // [deg].

/**
  * @brief  Initialize the incremental encoder driver.
  *
  * This function initialize the timer 5 as a quadrature decoder, and set the
  * current paddle position to zero.
  */
void enc_Init(void)
{
    tim5InitFunc(); // Setup the timer 5 to be incremented with the quadrature signal of the encoder.
    //extitInitFunc(); // Setup interrupt for the index line of the encoder.
    
    enc_offset = 0.0f;
}

/**
  * @brief Gets the current motor shaft angle, measured by the encoder counter.
  * @retval The current angle of the motor shaft [deg].
  */
float32_t enc_GetPosition(void)
{
    // Convert the counter value (coder increments) to the paddle angle in degrees.
    int32_t rawCounter = (int32_t)TIM_GetCounter(TIM5); // Coder position (max +/- 2^31).
    float32_t outputAngle = ((float32_t)rawCounter) / ((float32_t)CODER_RESOLUTION) * 360.0f;
    
    outputAngle += enc_offset;
    
    return outputAngle;
}

/**
  * @brief Set the position offset.
  *
  * The new position offset is computed such that the given position will now be
  * returned by enc_GetPosition() if the paddle remains at this physical position.
  * @param newPosition the new paddle position.
  */
void enc_SetPosition(float32_t newPosition)
{
    enc_offset = 0.0f;
    
    enc_offset = newPosition - enc_GetPosition();
}

/**
  * @brief  Initialize TIM5 as a counter for quadrature signals.
  */
void tim5InitFunc(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    GPIO_InitTypeDef        GPIO_InitStruct;

    // Setup the two pins wired to the encoder, to be used as the "clock" of the timer.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_InitStruct.GPIO_Pin = CODER_A_Pin;
    GPIO_Init(CODER_A_Port, &GPIO_InitStruct);
    GPIO_PinAFConfig(CODER_A_Port, CODER_A_PinSource, GPIO_AF_TIM5);

    GPIO_InitStruct.GPIO_Pin = CODER_B_Pin;
    GPIO_Init(CODER_B_Port, &GPIO_InitStruct);
    GPIO_PinAFConfig(CODER_B_Port, CODER_B_PinSource, GPIO_AF_TIM5);

    TIM_TimeBaseStruct.TIM_Period        = 0xFFFFFFFF;   
    TIM_TimeBaseStruct.TIM_Prescaler     = (uint16_t)0;
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);    
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM5, 0);

    TIM_Cmd(TIM5, ENABLE);
}

/**
  * @brief  Initialize interrupt from Index line of the coder 
  */
void extitInitFunc(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = CODER_I_Pin;
    GPIO_Init(CODER_I_Port, &GPIO_InitStruct);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);
    
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Line    = EXTI_Line8;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = EXTI9_5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = CODER_INDEX_IRQ_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}



/**
  * @brief  Interrupt from Index line of the coder (check step loss)
  * @note This function currently does nothing.
  */
void EXTI9_5_IRQHandler(void){
//      static int32_t motorPosTurn_old         = 0;
//      static int32_t motorPosTurn_old_tmp = 0;
//      int16_t localVar;
    
        if(EXTI_GetITStatus(EXTI_Line8) != RESET)
        {
//              localVar = (int16_t)TIM_GetCounter(TIM4);
//              debugVar02 = localVar;
//              if(localVar>1000){                  
//                      motorPosTurn_old = motorPosTurn;
//                      motorPosTurn++;
//              }else if(localVar<(-1000)){
//                      motorPosTurn_old = motorPosTurn;
//                      motorPosTurn--;
//              }else{
//                      motorPosTurn_old_tmp    = motorPosTurn;
//                      motorPosTurn                    = motorPosTurn_old;
//                      motorPosTurn_old            = motorPosTurn_old_tmp;
//              }
//              
//              TIM_SetCounter(TIM4,(uint32_t)CODER_INDEX_POS);
//              if(~statusReg & REF_POS_INIT){      // First time index is detected.
//                      statusReg |= REF_POS_INIT;
//                      motorPosTurn = 0;
//                      //TIM_SetCounter(TIM4,(uint32_t)6000);
//              }
                EXTI_ClearITPendingBit(EXTI_Line8);
        }
}
