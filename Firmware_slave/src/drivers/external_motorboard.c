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

#include "external_motorboard.h"

/**
  * @brief Initializes the motorboard driver.
  */
void emot_Init(void)
{
	// Not implemented yet.
}

/**
  * @brief Set the motor torque.
  * @param torque the torque the motor should apply [N.m].
  */
void emot_SetTorque(float32_t torque)
{
	
}

/**
  * @brief Gets the current motor shaft position.
  * @return the motor position given by the encoder [deg].
  */
float32_t emot_GetPosition(void)
{
	return 0.0f;
}
