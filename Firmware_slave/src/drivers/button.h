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

#ifndef __BUTTON_H
#define __BUTTON_H

#include "../main.h"

/** @defgroup Button Driver / Button
  * @brief Driver to access a button
  *
  * Call but_Init() first in the initialization code. Then, call but_GetState()
  * to read the current state of the button.
  *
  * It is also possible to pass a function pointer to the but_Init() function.
  * Then, the given function will be called automatically when the button state
  * changes (pressed or released).
  *
  * @addtogroup Button
  * @{
  */

void but_Init(void (*stateChangedCallback)(bool));
bool but_GetState(void);

/**
  * @}
  */

#endif
