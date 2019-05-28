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

#ifndef __LED_H
#define __LED_H

#include "../main.h"

/** @defgroup LED Driver / LED
  * @brief Driver to control a LEDs row.
  *
  * Call led_Init() first in the initialization code. Then, call led_Set() to
  * set the LEDs ON or OFF.
  *
  * @addtogroup LED
  * @{
  */

void led_Init(void);
float32_t led_Get(int ledIndex);
void led_Set(int ledIndex, float32_t brightness);

/**
  * @}
  */

#endif
