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

#include "adc.h"
#include "../lib/basic_filter.h"
#include "../lib/utils.h"

volatile uint16_t adc_currentValuesBuffer[ADC_BUFFER_SIZE];
bfilt_BasicFilter adc_currentFilter;
float32_t adc_currentSensOffset = 0.0f;

void adc_DmaInit(void);

/**
  * @brief  Initialize the ADC converter (2 analog inputs + current sense).
  */
void adc_Init(void)
{
    ADC_InitTypeDef         ADC_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
    GPIO_InitTypeDef        GPIO_InitStructure;
        
    adc_DmaInit();

    // Periph clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);
        
    // GPIO configuration
    GPIO_InitStructure.GPIO_Pin  = ADC_HALL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_HALL_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = ADC_ANIN1_PIN;
    GPIO_Init(ADC_ANIN1_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = ADC_ANIN2_PIN;
    GPIO_Init(ADC_ANIN2_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = ADC_ANIN3_PIN;
    GPIO_Init(ADC_ANIN3_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = ADC_ANIN4_PIN;
    GPIO_Init(ADC_ANIN4_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ADC_CURRENT_SENSE_PIN;
    GPIO_Init(ADC_CURRENT_SENSE_PORT, &GPIO_InitStructure);

    // ADC Common configuration
    ADC_DeInit();
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // Useless here (only used in dual or triple interleaved modes).
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC 1 configuration for general purpose.
    ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode         = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv     = 0;
    ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion      = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    ADC_RegularChannelConfig(ADC1, ADC_HALL_CHANNEL, 1, ADC_SampleTime_56Cycles);
    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1);

    // ADC 3 configuration for motor current sensing.
    // Synchronous sampling with the PWM replaced by a high-frequency sampling at ~300kHz (over-sampling), because of the commutation noise of the current measuring amplifier at ~120kHz.
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;       
    ADC_Init(ADC3, &ADC_InitStructure);   

    ADC_RegularChannelConfig(ADC3, ADC_CURRENT_SENSE_CHANNEL, 1, ADC_SampleTime_28Cycles);
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
    ADC_DMACmd(ADC3, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
    ADC_SoftwareStartConv(ADC3);

    // Setup the filter of the ADC current samples over time.
    bfilt_Init(&adc_currentFilter, 0.05f, 0.0f);
}

/**
  * @brief  Setup the DMA that copies the current ADC samples to the RAM.
  */
void adc_DmaInit(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&adc_currentValuesBuffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = ADC_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_InitStructure.DMA_Channel            = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream1, ENABLE);
}

/**
  * @brief  Compute the current sense offset.
  * @note   Run before enabling current regulation.
  */
void adc_CalibrateCurrentSens(void)
{
    float32_t offset = 0.0f;
    int32_t i=0;
    
    for(i=0; i<ADC_CALIB_N_SAMPLES; i++)
    {
        offset += adc_GetCurrent() / (float32_t)ADC_CALIB_N_SAMPLES;
    
        utils_DelayUs(400);
    }
        
    adc_currentSensOffset = offset;
}

/**
  * @brief  Compute the current sense offset.
  * @retval The measured current in [mA].
  */
float32_t adc_GetCurrent(void)
{
    int32_t motorCurrentSum;
    float32_t motorCurrentMean;

    int32_t i;
    
    // Compute the average of all the ADC values in the buffer.
    motorCurrentSum = 0; // [ADC raw value].

    for(i=0; i<ADC_BUFFER_SIZE; i++)
        motorCurrentSum += (int32_t)(adc_currentValuesBuffer[i]);
    
    motorCurrentMean = (float32_t)motorCurrentSum / (float32_t)ADC_BUFFER_SIZE;

    // Convert from ADC increments to a current in [A].
    motorCurrentMean *= ADC_CURRENT_SCALE;
    
    // Add the offset obtained during the sense resistor calibration.
    motorCurrentMean -= (float32_t) adc_currentSensOffset;
    
    // Use low-pass filtering to reduce the noise.
    bfilt_Step(&adc_currentFilter, motorCurrentMean);
    
    //return motorCurrentMean;
    return adc_currentFilter.filteredValue;
}

/**
  * @brief Start the ADC conversion of the selected channel.
  * @param channel: the ADC channel to acquire.
  */
void adc_StartConversion(AdcChannel channel)
{
    ADC_Cmd(ADC1, DISABLE);
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_56Cycles);
    ADC_Cmd(ADC1, ENABLE);
    utils_DelayUs(1);
    ADC_SoftwareStartConv(ADC1);
}

/**
  * @brief Gets the ADC conversion status of the selected channel.
  * @retval 1 if the conversion is finished, 0 otherwise.
  */
bool adc_ConversionIsFinished(void)
{
    return (ADC1->SR & ADC_SR_EOC) != 0;
}

/**
  * @brief Gets the voltage measured by the selected ADC channel.
  * @return the measured voltage [V].
  * @note make sure that the conversion is finished before calling.
  */
float32_t adc_GetConversionResult(void)
{
    float32_t adcRawVal, voltage;
    
    adcRawVal = (float32_t)ADC1->DR;
        
    voltage = (adcRawVal / ADC_MAX) * ADC_REF_VOLTAGE;
 
    return voltage;
}

/**
  * @brief Gets the voltage measured by the selected ADC channel.
  * @param channel: the channel of the ADC.
  * @return the measured voltage [V].
  */
float32_t adc_GetChannelVoltage(AdcChannel channel)
{
    int32_t conversionTime = 0;
    
    adc_StartConversion(channel);
    
    while(!adc_ConversionIsFinished())
    {
        if(conversionTime < ADC_MAX_CONVERSION_TIME)
            conversionTime++;
        else
            break;
    }

    return adc_GetConversionResult();
}   
