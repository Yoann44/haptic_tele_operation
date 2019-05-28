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

#ifndef __HALL_H
#define __HALL_H

#include "../main.h"
#include "adc.h"

/** @defgroup Hall Driver / Hall sensor
  * @brief Driver for a Hall-effect angular sensor.
  *
  * This driver uses an ADC input, ANIN1 or ANIN2 (see ADC).
  * To use this, connect wire the sensor as this:
  * * hall red -> +5V (pin 1 or 5 of J2).
  * * hall yellow -> GND (pin 2 or 6 of J2).
  * * hall blue -> ANINX+ (pin 3 or 7 of J2).
  * * GND (pin 2 or 6 of J2) -> ANINX- (pin 4 or 8 of J2).
  *
  * Set the sensitivity switches to +-10V.
  *
  * Call hall_Init() first in the initialization code. Then, call hall_Get() to
  * get the angle of the paddle.
  *
  * @addtogroup Hall
  * @{
  */

void hall_Init(AdcChannel channel);
float32_t hall_GetVoltage(void);

/**
  * @}
  */

#endif
