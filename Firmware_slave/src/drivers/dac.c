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

#include "dac.h"
#include "../lib/utils.h"

/**
  * @brief  Setup a DAC with 2 channels.
  */
void dac_Init(void)
{
	DAC_InitTypeDef  DAC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
    // Periph clock enable.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    // GPIO configuration.
	GPIO_InitStructure.GPIO_Pin  = DAC1_Pin|DAC2_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(DAC_Port, &GPIO_InitStructure);

    // DAC channel Configuration.
	DAC_InitStructure.DAC_Trigger                      = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration               = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitStructure.DAC_OutputBuffer                 = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1|DAC_Channel_2, &DAC_InitStructure);
		
    // Enable DAC Channel.
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);

    // Enable DMA for DAC Channel.
	DAC_DMACmd(DAC_Channel_1|DAC_Channel_2, ENABLE);
    
    // Set initial voltage to zero.
    dac_SetVoltage1(0.0f);
    dac_SetVoltage2(0.0f);
}

/**
  * @brief  Set the voltage of the channel 1 of the DAC.
  * @param  finalVoltage: the new voltage [V], between +-DAC_FINAL_RANGE (+-9V).
  */
void dac_SetVoltage1(float32_t finalVoltage)
{
    float32_t dacVoltageRatio = (-finalVoltage / DAC_FINAL_RANGE / 2.0f) + 0.5f;
    uint32_t regVal = (uint32_t)(dacVoltageRatio * (float32_t)DAC_MAX);
    
    utils_SaturateU(&regVal, 0, DAC_MAX);
    
    DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)regVal);
}

/**
  * @brief  Set the voltage of the channel 2 of the DAC.
  * @param  finalVoltage: the new voltage [V], between +-DAC_FINAL_RANGE (+-9V).
  */
void dac_SetVoltage2(float32_t finalVoltage)
{
    float32_t dacVoltageRatio = (-finalVoltage / DAC_FINAL_RANGE / 2.0f) + 0.5f;
    uint32_t regVal = (uint32_t)(dacVoltageRatio * (float32_t)DAC_MAX);
    
    utils_SaturateU(&regVal, 0, DAC_MAX);
    
    DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)regVal);
}

/**
  * @brief  Get the current output voltage of the channel 1 of the DAC.
  * @retval The current voltage of the channel 1 of the DAC [V].
  */
float32_t dac_GetVoltage1(void)
{
    uint32_t regVal = DAC_GetDataOutputValue(DAC_Channel_2);
    
    return -((float32_t)regVal / (float32_t)DAC_MAX - 0.5f) * 2.0f * DAC_FINAL_RANGE;
}

/**
  * @brief  Get the current output voltage of the channel 2 of the DAC.
  * @retval The current voltage of the channel 2 of the DAC [V].
  */
float32_t dac_GetVoltage2(void)
{
    uint32_t regVal = DAC_GetDataOutputValue(DAC_Channel_1);
    
    return -((float32_t)regVal / (float32_t)DAC_MAX - 0.5f) * 2.0f * DAC_FINAL_RANGE;
}
