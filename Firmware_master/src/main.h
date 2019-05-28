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

/** \mainpage HRI board firmware documentation
 * \section intro_sec Introduction
 * This is the documentation of the HRI board firmware. You will find a
 * description of the provided library and drivers. As all the hardware and
 * software is very new, please do not hesitate to suggest improvements to the
 * course assistants (Romain Baud, Philipp Hörler and Laurent Jenni).
 *
 * \section code_struct_sec Code structure
 * The code is divided into 3 parts:
 *  - main: this is where the controllers and the communication protocol are
 * implemented. The call of all the initialization functions is also made here.
 *  - library: all the algorithms, data structures that could be used at many
 * places, are put into the library.
 *  - drivers: there is one driver per peripheral. All the hardware-specific
 * code is wrapped into these files, so that the user does not have to worry
 * about the microcontroller operation (and spend a lot of time to read the
 * documentation).
 *
 * One module (controller, communication, library or driver) is always splitted
 * in two files, .h and .c, like in C++.
 * @image html soft_architecture.png
 *
 * \section lib_how_to How to use the provided library
 * For most of the driver/library modules, call x_Init() only once, when the
 * program starts (typically in the main()). Then, you can call
 * x_Step()/x_Set()/x_Get() everytime as needed, typically in the control
 * functions called periodically (ctrl_RegulatePosition()...).
 * 
 * See the "Modules" section of this documentation to see how they work.
 *
 * \section stm32_resources_usage STM32's resources usage
 * \subsection stm32_resources_usage_timers Timers
 *  - TIM1: PWM of the 4 user LEDs.
 *  - TIM2: -
 *  - TIM3: -
 *  - TIM4: -
 *  - TIM5: encoder quadrature decoder.
 *  - TIM6: position loop.
 *  - TIM7: variables streaming loop.
 *  - TIM8: H-bridge PWM.
 *  - TIM9: -
 *  - TIM10: current loop.
 *  - TIM11: -
 *  - TIM12: -
 *  - TIM13: -
 *  - TIM14: -
 * \subsection stm32_resources_usage_adcs ADCs
 *  - ADC1: general purpose ADC.
 *  - ADC2: -
 *  - ADC3: motor current sensor ADC.
 * \subsection stm32_resources_usage_dac DAC
 *  - DAC: DAC extension.
 * \subsection stm32_resources_usage_buses Communication buses
 *  - USART1: UART extension.
 *  - USART2: USB communication UART.
 *  - I2C1: I2C extension.
 *  - SPI3: SPI extension.
 */

#ifndef __MAIN_H
#define __MAIN_H

#include <math.h>
#include <stdbool.h>
#include "arm_math.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_wwdg.h"

// Clocks configuration.
#define STM_SYSCLOCK_FREQ 168000000 // [Hz].
#define APB1_PRESCALER 4
#define APB2_PRESCALER 2
#define TIM_MULTIPLIER 2

// Interupt priority.
#define CURRENT_LOOP_IRQ_PRIORITY 1 // High freq loop, should interrupt all the others.
#define CONTROL_LOOP_IRQ_PRIORITY 2
#define CODER_INDEX_IRQ_PRIORITY  2 // Useless, remove?
#define UART_RX_IRQ_PRIORIY       3
#define DATA_LOOP_IRQ_PRIORITY    4 // Streaming packets.
#define USER_BUTTON_IRQ_PRIORITY  4

// Electrical parameters.
#define STM_SUPPLY_VOLTAGE 3.3f // Power supply voltage of the microcontroller [V].
#define ADC_REF_VOLTAGE 2.5f // Voltage reference of the ADC (VREF) [V].
#define H_BRIDGE_SUPPLY_VOLTAGE 24.0f // [V].
#define CURRENT_SHUNT_RESISTANCE 0.025f // [ohm].
#define CURRENT_SHUNT_AMPLIFIER_GAIN 30.0f // Gain of 60 (AD817) / 2 (voltage divider) [].
#define MOTOR_RESISTANCE (10.6f + 5.0f) // 10.6 ohm according to the datasheet, actually more, depends on the motor [ohm].

// Mechanical parameters.
#define REDUCTION_RATIO 15.0f
#define MOTOR_TORQUE_CONST 0.0538f // [N.m/A].
#define MOTOR_SPEED_CONST 177.0f // [RPM/V].
#define MOTOR_NOMINAL_TORQUE 0.0323f // [N.m].

#endif
