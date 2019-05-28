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

#ifndef __PID_H
#define __PID_H

#include "../main.h"

/** @defgroup PID Lib / PID regulator
  * @brief Proportionnal-integral-derivative regulator with integrator
  * saturation capability.
  *
  * First, instantiate a pid_Pid structure (e.g. "pid_Pid pid;"), then
  * initialize it once with pid_Init(). Then, every time a new command needs to
  * be computed (typically when a new measurement arrives), call pid_Step().
  *
  * @addtogroup PID
  * @{
  */

/**
  *@brief PID regulator structure
  */
typedef struct
{
    float32_t kp, ///< Proportional coefficient.
              ki, ///< Integral coefficient. Disabled if negative or zero.
              kd, ///< Derivative coefficient.
              arw, ///< Max value of the integrator ("anti-reset windup"). Disabled if negative or zero.
              previousErr, ///< Error (current-target) at the previous timestep, for derivative computation.
              integrator, ///< Integrator value for the integral part of the PID.
              current, ///< Current state of the system to control.
              target, ///< Desired state of the system to control.
              command, ///< Output command computed by the PID regulator.
              feedforward; ///< Feedforward coefficient.
} pid_Pid;

void pid_Init(pid_Pid *pid, float32_t kp, float32_t ki, float32_t kd,
              float32_t arw, float32_t feedforward);
float32_t pid_Step(pid_Pid *pid, float32_t current, float32_t target, float32_t dt);

/**
  * @}
  */

#endif
