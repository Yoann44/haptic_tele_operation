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

#ifndef __INCR_ENCODERS_H
#define __INCR_ENCODERS_H

#include "../main.h"

#define CODER_A_Pin       GPIO_Pin_0
#define CODER_A_Port      GPIOA
#define CODER_A_PinSource GPIO_PinSource0

#define CODER_B_Pin       GPIO_Pin_1
#define CODER_B_Port      GPIOA
#define CODER_B_PinSource GPIO_PinSource1

#define CODER_I_Pin       GPIO_Pin_3
#define CODER_I_Port      GPIOE

#define CODER_RESOLUTION ((uint32_t)2000) // Number of increments per turn.

/** @defgroup Encoder Driver / Incremental encoder
  * @brief Driver for an incremental encoder.
  *
  * This driver uses a timer of the STM32, configured as a quadrature decoder.
  *
  * Call enc_Init() first in the initialization code. Then, call enc_GetPosition() to
  * get the decoded value.
  * The output value is the paddle position in degrees, taking the reduction
  * ratio into account.
  *
  * @addtogroup Encoder
  * @{
  */

void enc_Init(void);
float32_t enc_GetPosition(void);
void enc_SetPosition(float32_t newPosition);

/**
  * @}
  */

#endif
