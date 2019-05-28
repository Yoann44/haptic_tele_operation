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

#include "communication.h"
#include "drivers/adc.h"
#include "drivers/dac.h"
#include "drivers/debug_gpio.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/h_bridge.h"
#include "drivers/callback_timers.h"
#include "lib/basic_filter.h"
#include "lib/pid.h"
#include "lib/utils.h"
#include "torque_regulator.h"

#define KP_CURRENT_DEFAULT_VAL 100.0f // [V/A].
#define KI_CURRENT_DEFAULT_VAL 500000.0f // [V/(A.s)].
#define KD_CURRENT_DEFAULT_VAL 0.0f // [V/(A/s)].
#define FF_CURRENT_DEFAULT_VAL MOTOR_RESISTANCE // [V/A].
#define CURRENT_INTEGRATOR_SAT_DEFAULT_VAL 10.0f // [V].
#define CURRENT_LOOP_PWM_MAX_DUTY_CYCLE 0.98f // Max PWM value (normalized) duty cycle (less than 1 to ensure bootstrap capacitor charging).

#define CURRENT_LOOP_PERIOD 50 // Current control loop period [us].

#define SOFTER_PID_DURATION 0.005f // Short time when softer PID settings are used, in case a H-bridge fault is detected [s].

volatile float32_t torq_targetCurrent; // Target current [A].
volatile pid_Pid torq_currentPid;

bool torq_regulateCurrent;
float32_t torq_pidSoftModeTime; // Remaining time for the "softer PID settings" mode [s].

void torq_RegulateCurrent(void);

/**
  * @brief Initialize the position and current controllers.
  */
void torq_Init(void)
{
    torq_targetCurrent = 0.0f;
    torq_pidSoftModeTime = 0.0f;
    
    // By default the current regulator is off, to allow the calibration of the
    // current sensor.
    torq_regulateCurrent = false;
    
    // Setup the PID.
    pid_Init((pid_Pid*)&torq_currentPid, KP_CURRENT_DEFAULT_VAL,
             KI_CURRENT_DEFAULT_VAL, KD_CURRENT_DEFAULT_VAL,
             CURRENT_INTEGRATOR_SAT_DEFAULT_VAL, FF_CURRENT_DEFAULT_VAL);

    // Make the timers call the regulation function periodically.
    cbt_SetCurrentLoopTimer(torq_RegulateCurrent, CURRENT_LOOP_PERIOD);

    // Share some variables with the computer.
    comm_monitorFloat("actual_current [A]", (float32_t*)&torq_currentPid.current, READONLY);
    comm_monitorFloat("target_current [A]", (float32_t*)&torq_currentPid.target, READONLY);
}

/**
  * @brief Start the current regulation.
  */
void torq_StartCurrentLoop(void)
{
    torq_regulateCurrent = true;
}

/**
  * @brief  Current regulation "loop" function.
  */
void torq_RegulateCurrent()
{
    float32_t motorVoltage, // Motor command voltage [V].
              pwmNormalizedDutyCycle; // Motor normalized PWM duty (-1 ot 1).
    float32_t motorCurrentCurrent; // [A].
    float32_t dt; // [s].

    // Compute the dt.
    dt = (float32_t)cbt_GetCurrentLoopPeriod()*MICROSECOND_TO_SECOND;
             
    // Get the actual current.
    motorCurrentCurrent = adc_GetCurrent();

    // If the H-Bridge chip is in fault, it will block the current. So, reset
    // the PID integrator and soften the PID settings, to avoid a new current
    // surge when it will resume the current. This avoid the continuous
    // "beeping" that can appear in some conditions.
    if(hb_HasFault())
    {
        torq_currentPid.kp = KP_CURRENT_DEFAULT_VAL / 4.0f;
        torq_currentPid.ki = KI_CURRENT_DEFAULT_VAL / 4.0f;
        torq_currentPid.integrator = 0.0f;
        torq_pidSoftModeTime = SOFTER_PID_DURATION;
    }

    // Manage the "softer PID" mode.
    if(torq_pidSoftModeTime > 0.0f)
    {
        torq_pidSoftModeTime -= dt;
        torq_currentPid.integrator = 0.0f; // Keep the integrator empty.

        if(torq_pidSoftModeTime <= 0.0f)
        {
            // Go back to the normal PID settings.
            torq_currentPid.kp = KP_CURRENT_DEFAULT_VAL;
            torq_currentPid.ki = KI_CURRENT_DEFAULT_VAL;
        }
    }

    // Regulate.
    motorVoltage = -pid_Step((pid_Pid*)&torq_currentPid,
                             motorCurrentCurrent,
                             torq_targetCurrent,
                             (float32_t)cbt_GetCurrentLoopPeriod()*MICROSECOND_TO_SECOND);
    
    // Normalize to get a signed PWM duty (between -1 and 1).
    pwmNormalizedDutyCycle = motorVoltage / H_BRIDGE_SUPPLY_VOLTAGE;
    utils_SaturateF(&pwmNormalizedDutyCycle, -CURRENT_LOOP_PWM_MAX_DUTY_CYCLE,
                    CURRENT_LOOP_PWM_MAX_DUTY_CYCLE);

    // Apply the computed PWM duty, if the current regulation is enabled.
    if(torq_regulateCurrent)
        hb_SetPWM(pwmNormalizedDutyCycle);
    else
        hb_SetPWM(0.0f);
}

/**
 * @brief Sets the target motor torque.
 * @param torque target motor torque [N.m].
 * @remark The given value will be saturated to the motor torque, if larger.
 */
void torq_SetTorque(float32_t torque)
{
    utils_SaturateF(&torque, -MOTOR_NOMINAL_TORQUE, MOTOR_NOMINAL_TORQUE);

    torq_targetCurrent = -torque / MOTOR_TORQUE_CONST; // Invert the current sign, so that a positive target torque makes the motor spin in the defined positive direction.
}
