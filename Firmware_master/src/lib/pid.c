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

#include "pid.h"
#include "utils.h"

/**
  * @brief Initialize the PID structure.
  * @param pid: pointer to the PID structure.
  * @param kp: proportionnal coefficient of the PID. No effect if zero.
  * @param ki: integral coefficient of the PID. No effect if zero.
  * @param kd: derivative coefficient of the PID. No effect if zero.
  * @param arw: maximum value of the integrator (anti-reset windup). Disabled if negative.
  * @param feedforward: component proportional to the target (not the error). No effect if zero.
  */
void pid_Init(pid_Pid *pid, float32_t kp, float32_t ki, float32_t kd,
              float32_t arw, float32_t feedforward)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->arw = arw;
    pid->previousErr = 0.0f;
    pid->integrator = 0.0f;
    pid->feedforward = feedforward;
}

/**
  * @brief  Step the PID structure.
  * @param  pid: pointer to the PID structure.
  * @param  current: current state of the system to control.
  * @param  target: target state of the system to control.
  * @param  dt: timestep (time since the last call of this function) [s].
  * @retval command to apply to the system.
  */
float32_t pid_Step(pid_Pid *pid, float32_t current, float32_t target, float32_t dt)
{
    float32_t err;

    pid->current = current;
    pid->target = target;

    // Error computation.
    err = target - current;
    
    // Feedforward part.
    pid->command = pid->feedforward * target;
    
    // Proportionnal part.
    pid->command += pid->kp * err;
    
    // Integral part.
    if(pid->ki > 0.0f)
    {
        pid->integrator += pid->ki * (err * dt);
        
        if(pid->arw > 0.0f)
            utils_SaturateF(&pid->integrator, -pid->arw, pid->arw);
        
        pid->command += pid->integrator;
    }
    
    // Derivative part.
    pid->command += pid->kd * ((err - pid->previousErr) / dt);
    pid->previousErr = err;
    
    return pid->command;
}
