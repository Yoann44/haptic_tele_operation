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

#ifndef __ADC_H
#define __ADC_H

#include "../main.h"

#define ADC_HALL_PIN GPIO_Pin_1
#define ADC_HALL_PORT GPIOB
#define ADC_HALL_CHANNEL ADC_Channel_9

#define ADC_ANIN1_PIN GPIO_Pin_6
#define ADC_ANIN1_PORT GPIOA
#define ADC_ANIN1_CHANNEL ADC_Channel_6

#define ADC_ANIN2_PIN GPIO_Pin_7
#define ADC_ANIN2_PORT GPIOA
#define ADC_ANIN2_CHANNEL ADC_Channel_7

#define ADC_ANIN3_PIN GPIO_Pin_4
#define ADC_ANIN3_PORT GPIOC
#define ADC_ANIN3_CHANNEL ADC_Channel_14

#define ADC_ANIN4_PIN GPIO_Pin_5
#define ADC_ANIN4_PORT GPIOC
#define ADC_ANIN4_CHANNEL ADC_Channel_15

#define ADC_CURRENT_SENSE_PIN GPIO_Pin_3
#define ADC_CURRENT_SENSE_PORT GPIOA
#define ADC_CURRENT_SENSE_CHANNEL ADC_Channel_3

#define ADC_MAX 4095.0f // Maximum value of the ADC register (2^12 - 1).
#define ADC_MAX_CONVERSION_TIME 100 // To avoid locking if the conversion was not started properly.
#define ADC_BUFFER_SIZE  33 // Fadc =~300kHz -> TE_ADC = 3.33us -> Average over 32 sample => usable bandwidth <10kHz
#define ADC_CURRENT_SCALE (ADC_REF_VOLTAGE / (CURRENT_SHUNT_RESISTANCE * CURRENT_SHUNT_AMPLIFIER_GAIN * ADC_MAX)) // Scale between ADC increment and current [A/incr].
#define ADC_CALIB_N_SAMPLES 1000

/** @defgroup ADC Driver / ADC
  * @brief Driver for the analog-to-digital peripheral of the STM32.
  *
  * An analog-to-digital converter (ADC) is used to measure the voltage of one
  * or several microcontroller analog pins.
  *
  * In addition, there is an additional channel to measure the current going
  * through the motor (useful for current regulation).
  *
  * Call adc_Init() first, in the initialization code. Then, call
  * adc_GetChannelVoltage() every time you need the voltage.
  *
  * To measure accurately the motor current, a calibration has to be performed
  * first. Call dc_CalibrateCurrentSens() in the main(), when the current
  * regulation is disabled (see H-bridge documentation). Then, call
  * adc_GetCurrent() every time it is needed. This function returns the last
  * value transfered by the DMA, so there is no conversion delay when calling
  * this function.
  *
  * @addtogroup ADC
  * @{
  */

/**
  * @brief Enum that corresponds to the two ADC input channels of the board.
  */
typedef enum
{
    ADC_CHANNEL_9 = 9, ///< Pin HALL_INPUT (5) of the connector J9 (Hall position sensor).
    ADC_CHANNEL_6 = 6, ///< Pin AN_IN1 (29 & 30) of the connector J12 (analog extension).
    ADC_CHANNEL_7 = 7, ///< Pin AN_IN2 (33 & 34) of the connector J12 (analog extension).
    ADC_CHANNEL_14 = 14, ///< Pin AN_IN3 (37 & 38) of the connector J12 (analog extension).
    ADC_CHANNEL_15 = 15, ///< Pin AN_IN4 (41 & 42) of the connector J12 (analog extension).
} AdcChannel;

void adc_Init(void);
void adc_CalibrateCurrentSens(void);
float32_t adc_GetCurrent(void); // [mA].
float32_t adc_GetChannelVoltage(AdcChannel channel);

/**
  * @}
  */

#endif
