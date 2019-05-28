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

#ifndef __HAPTIC_CONTROLLER_H
#define __HAPTIC_CONTROLLER_H

#include "main.h"

/** @defgroup HapticController Main / Haptic controller
  * @brief Main haptic paddle controller.
  *
  * This module is the high-level controller of the board. Basically, its role
  * is to process the sensors data and then compute a motor torque, in order to
  * achieve a spicific haptic effect.
  *
  * The content of hapt_Update() is typically what the user of the board will
  * modify, depending on the selected sensors and control algorithms.
  *
  * Call hapt_Init() to setup this module. Its interrupt function will be called
  * automatically periodically.
  *
  * @addtogroup HapticController
  * @{
  */

void hapt_Init(void);

/**
  * @}
  */

#endif
