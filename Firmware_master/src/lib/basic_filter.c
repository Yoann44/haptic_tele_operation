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

#include "basic_filter.h"

/**
  * @brief  Initialize a BasicFilter structure.
  * @param  filter: pointer on the structure to initialize.
  * @param	tau: the contribution of every new value. 0.0 (max filtering) to 1.0 (no filtering).
  * @param	initialValue: initial value for the filter.
  */
void bfilt_Init(bfilt_BasicFilter* filter, float32_t tau, float32_t initialValue)
{
    filter->tau = tau;
    filter->filteredValue = initialValue;
}

/**
  * @brief  Step a BasicFilter structure.
  * @param  filter: pointer on the structure to initialize.
  * @param	newValue: the new value of the signal to filter.
  * @retval the filtered value.
  * @note   this implementation is very basic, and does not take the timestep (dt) into account.
  */
float32_t bfilt_Step(bfilt_BasicFilter* filter, float32_t newValue)
{
    filter->filteredValue = filter->filteredValue*(1.0f-filter->tau) + newValue*filter->tau;
    return filter->filteredValue;
}
