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

#ifndef __EXTERNAL_MOTORBOARD_H
#define __EXTERNAL_MOTORBOARD_H

#include "../main.h"

/** @defgroup ExtMotorboard Driver / External motorboard
  * @brief Driver for an external motorboard.
  *
  * This driver controls an external motorboard, controlled through a serial
  * link. This allows driving larger motors.
  *
  * Call emot_Init() first in the initialization code. Then, call
  * emot_SetTorque() to set the motor torque, and emot_GetPosition() to get the
  * motor shaft position.
  *
  * @addtogroup ExtMotorboard
  * @{
  */

void emot_Init(void);
void emot_SetTorque(float32_t torque);
float32_t emot_GetPosition(void);

/**
  * @}
  */

#endif
