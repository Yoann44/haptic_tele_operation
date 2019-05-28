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

#ifndef __DAC_H
#define __DAC_H

#include "../main.h"

#define DAC1_Pin GPIO_Pin_4
#define DAC2_Pin GPIO_Pin_5
#define DAC_Port GPIOA

#define DAC_FINAL_RANGE 9.0f // -9 to 9V after the op-amp.
#define DAC_MAX 4095 // 12-bits.

/** @defgroup DAC Driver / DAC
  * @brief Driver for the digital-to-analog converter.
  *
  * The STM32 features a two channels DAC, which means it is able to drive two
  * pins with an analog voltage. On the HRI board, this voltage is mulltiplied
  * by an amplification stage, so the final output range is +-9V. The
  * corresponding output pins are on the J1 connector: ANOUT1 (2) and ANOUT2 (4).
  *
  * In the initialization code, call dac_Init() once. Then call
  * dac_GetVoltageX() everytime the output voltage needs to be updated.
  *
  * @addtogroup DAC
  * @{
  */

void dac_Init(void);
void dac_SetVoltage1(float32_t voltage);
void dac_SetVoltage2(float32_t voltage);
float32_t dac_GetVoltage1(void);
float32_t dac_GetVoltage2(void);

/**
  * @}
  */

#endif
