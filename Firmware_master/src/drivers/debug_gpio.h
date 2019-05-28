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

#ifndef __DEBUG_GPIO_H
#define __DEBUG_GPIO_H

#include "../main.h"

/** @defgroup DGPIO Driver / Debug GPIOs
  * @brief Driver to control three GPIOs, to debug easily with an oscilloscope.
  *
  * Call dio_Init() first in the initialization code. Then, call dio_Set() to 
  * set the GPIO states.
  *
  * @addtogroup DGPIO
  * @{
  */

void dio_Init(void);
bool dio_Get(int pinIndex);
void dio_Set(int pinIndex, bool high);
void dio_Toggle(int pinIndex);

/**
  * @}
  */

#endif
