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

#ifndef __UTILS_H
#define __UTILS_H

#include "../main.h"

/** @defgroup Utils Lib / Utils
  * @brief Utility functions and constants.
  *
  * This module provides functions and constants that may be used at many
  * places in the code.
  *
  * @addtogroup Utils
  * @{
  */

#define CPU_TRAPS_ENABLED 1 ///< utils_TrapCpu() will be block forever if 1, or return immediately if 0.

#define SECOND_TO_MICROSECOND 1000000.0f
#define MICROSECOND_TO_SECOND (1.0f/SECOND_TO_MICROSECOND)

void utils_TrapCpu(void);

void utils_DelayUs(uint32_t duration);
void utils_DelayMs(uint32_t duration);

void utils_DelayUs(uint32_t duration);
void utils_DelayMs(uint32_t duration);
void utils_SaturateF(float32_t *val, float32_t min, float32_t max);
void utils_SaturateU(uint32_t *val, uint32_t min, uint32_t max);
float32_t utils_Mean(float32_t *array, int size);

/**
  * @}
  */

#endif
