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

#ifndef __H_BRIDGE_H
#define __H_BRIDGE_H

#include "../main.h"

#define PWM1_Pin  6
#define PWM1_Port GPIOC
#define PWM2_Pin  7
#define PWM2_Port GPIOC
#define ENB1_Pin  8
#define ENB1_Port GPIOC
#define ENB2_Pin  9
#define ENB2_Port GPIOC
#define nFAULT_Pin  GPIO_Pin_15
#define nFAULT_Port GPIOD
#define nSLEEP_Pin  GPIO_Pin_14
#define nSLEEP_Port GPIOD

#define PWM_FREQUENCY 30000 // Motor PWM freq. [Hz] 

#define PWM_RESOL_SHIFT_DWN 6 // PWM_SCALE = 0xFFFF/2^PWM_RESOL_SHIFT_DWN 
#define CURRENT_SCALE_RESOL ((uint32_t)(16-PWM_RESOL_SHIFT_DWN))
#define PWM_TIM_PERIODE     ((int16_t)(0xFFFF>>PWM_RESOL_SHIFT_DWN))
#define PWM_TIM_PRESCALER   ((int16_t)(SystemCoreClock/APB2_PRESCALER*TIM_MULTIPLIER/PWM_FREQUENCY/PWM_TIM_PERIODE-1))

/** @defgroup H_Bridge Driver / H-bridge
  * @brief Driver for the H-bridge of the motor.
  *
  * The DC motor is supplied by a H-bridge on the HRI board. These power
  * electronics are controlled by the microcontroller using a PWM timer.
  *
  * Call hb_Init() first in the initialization code. Then, call hb_Enable()
  * to enable the H-bridge chip (power on), and hb_SetPWM() to set the motor
  * voltage.
  *
  * @addtogroup H_Bridge
  * @{
  */

void hb_Init(void);
void hb_Enable(void);
void hb_Disable(void);
void hb_SetPWM(float32_t ratio);
bool hb_HasFault(void);

/**
  * @}
  */

#endif
