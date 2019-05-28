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

#include "hall.h"

#define HALL_AMPLI_GAIN 1.1f
#define HALL_VOLT_DIVIDER 0.5f

AdcChannel hall_channel;

/**
  * @brief  Initialize the hall sensor driver.
  * @param  channel: the ADC channel the hall sensor is wired to (0 or 1).
  */
void hall_Init(AdcChannel channel)
{
    hall_channel = channel;
}

/**
  * @brief Return the Hall sensor output voltage.
  * @return The Hall sensor output voltage [V].
  */
float32_t hall_GetVoltage(void)
{
    // Because of the analog stages between the Hall sensor output pin and the
    // ADC pin, a conversion is necessary to obtain the Hall output voltage.
    float32_t voltage = adc_GetChannelVoltage(hall_channel); // Voltage at the ADC pin.
    voltage = voltage / HALL_VOLT_DIVIDER; // Voltage between the Bessel filter and the voltage divider.
    voltage = (voltage - ADC_REF_VOLTAGE) / HALL_AMPLI_GAIN + ADC_REF_VOLTAGE; // Voltage at the Hall output pin.

    return voltage;
}
