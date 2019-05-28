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

#include "led.h"
#include "../communication.h"

#define N_LEDS 4

#define LED_TIMER TIM1
#define LED_PORT GPIOA
#define LED_MAX_DUTY 255
#define LED_PWM_FREQ 32000 // [Hz].

typedef struct
{
    uint16_t const pin, pinSource;
    volatile uint32_t *const duty;
} led_Led;

led_Led led_leds[N_LEDS] =
{
    { GPIO_Pin_8, GPIO_PinSource8, &LED_TIMER->CCR1 },
    { GPIO_Pin_9, GPIO_PinSource9, &LED_TIMER->CCR2 },
    { GPIO_Pin_10, GPIO_PinSource10, &LED_TIMER->CCR3 },
    { GPIO_Pin_11, GPIO_PinSource11, &LED_TIMER->CCR4 },
};

void setLed0(float32_t brightness) { led_Set(0, brightness); };
void setLed1(float32_t brightness) { led_Set(1, brightness); };
void setLed2(float32_t brightness) { led_Set(2, brightness); };
void setLed3(float32_t brightness) { led_Set(3, brightness); };

float32_t getLed0(void) { return led_Get(0); };
float32_t getLed1(void) { return led_Get(1); };
float32_t getLed2(void) { return led_Get(2); };
float32_t getLed3(void) { return led_Get(3); };

/**
 * @brief Initializes the LEDs module.
 */
void led_Init(void)
{
    int i;
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    // Initialize each pin.
    for(i=0; i<N_LEDS; i++)
    {
        GPIO_InitStruct.GPIO_Pin   = led_leds[i].pin;
        GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_Init(LED_PORT, &GPIO_InitStruct);
        
        GPIO_PinAFConfig(LED_PORT, led_leds[i].pinSource, GPIO_AF_TIM1);
    }
    
    // Initialize the timer peripheral, as PWM generator.
    TIM_TimeBaseStruct.TIM_Period = LED_MAX_DUTY;  
    TIM_TimeBaseStruct.TIM_Prescaler = (STM_SYSCLOCK_FREQ / APB2_PRESCALER * 2)
                                       / LED_PWM_FREQ / LED_MAX_DUTY;
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(LED_TIMER, &TIM_TimeBaseStruct);
    
    TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse       = 0;
    TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    
    TIM_OC1Init(LED_TIMER, &TIM_OCInitStruct);
    TIM_OC2Init(LED_TIMER, &TIM_OCInitStruct);
    TIM_OC3Init(LED_TIMER, &TIM_OCInitStruct);
    TIM_OC4Init(LED_TIMER, &TIM_OCInitStruct);
    
    TIM_OC1PreloadConfig(LED_TIMER, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(LED_TIMER, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(LED_TIMER, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(LED_TIMER, TIM_OCPreload_Enable);

    TIM_SelectOutputTrigger(LED_TIMER, TIM_TRGOSource_Update);

    TIM_Cmd(LED_TIMER, ENABLE);
    TIM_CtrlPWMOutputs(LED_TIMER, ENABLE);
    
    // Create the SyncVars.
    comm_monitorFloatFunc("led_0 [0.0-1.0]", getLed0, setLed0);
    comm_monitorFloatFunc("led_1 [0.0-1.0]", getLed1, setLed1);
    comm_monitorFloatFunc("led_2 [0.0-1.0]", getLed2, setLed2);
    comm_monitorFloatFunc("led_3 [0.0-1.0]", getLed3, setLed3);
}

/**
 * @brief Gets the intensity of a single LED.
 * @param ledIndex: LED index (0, 1, 2 or 3).
 * @return the current LED intensity [0.0-1.0].
 */
float32_t led_Get(int ledIndex)
{
    if(ledIndex >= 0 && ledIndex < N_LEDS)
        return ((float32_t)(*led_leds[ledIndex].duty)) / (float32_t)LED_MAX_DUTY;
    else
        return 0.0f;
}

/**
 * @brief Sets the intensity of a single LED.
 * @param ledIndex: LED index (0, 1, 2 or 3).
 * @param brightness: the LED intensity [0.0-1.0].
 */
void led_Set(int ledIndex, float32_t brightness)
{
    if(ledIndex >= 0 && ledIndex < N_LEDS &&
       brightness >= 0.0f && brightness <= 1.0f)
    {
        *led_leds[ledIndex].duty = (uint32_t)(brightness * (float32_t)LED_MAX_DUTY);
    }
}
