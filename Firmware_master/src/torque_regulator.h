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

#ifndef __TORQUE_REGULATOR_H
#define __TORQUE_REGULATOR_H

#include "main.h"

/** @defgroup TorqueRegulator Main / Torque Regulator
  * @brief Sets the desired motor, by controlling the current.
  *
  * Call torq_Init() to setup this module. Its interrupt function will be called
  * automatically periodically. Then, call torq_StartCurrentLoop() when the
  * current sensor is calibrated. torq_SetTorque() can now be called at any time
  * to set the target torque.
  *
  * @addtogroup TorqueRegulator
  * @{
  */

void torq_Init(void);
void torq_StartCurrentLoop(void);
void torq_SetTorque(float32_t torque);

/**
  * @}
  */

#endif
