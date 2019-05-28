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

#ifndef __BASIC_FILTER_H
#define __BASIC_FILTER_H

#include "../main.h"

/** @defgroup BasicFilter Lib / Basic filter
  * @brief Basic iterative filter for smoothing.
  *
  * First, instance a bfilt_BasicFilter structure (e.g. "bfilt_BasicFilter
  * b;"), then initialize it once with bfilt_Init(). Then, every time a
  * new value of the signal is received, call bfilt_Step().
  *
  * @addtogroup BasicFilter
  * @{
  */

/**
  *@brief Basic filter structure.
  */
typedef struct
{
    float32_t tau, ///< The strength of the filter (0.0-1.0). 0 filters the most, 1 does not filter.
              filteredValue; ///< Last computed filtered value.
} bfilt_BasicFilter;

void bfilt_Init(bfilt_BasicFilter* filter, float32_t tau, float32_t initialValue);
float32_t bfilt_Step(bfilt_BasicFilter* filter, float32_t newValue);

/**
  * @}
  */

#endif
