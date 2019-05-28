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

#include "i2c.h"

#include "stm32f4xx_i2c.h"

#include "../lib/utils.h"

const uint32_t TIMEOUT = 1000; ///< Approximate maximum time for a I2C operation to complete (approx. 100us).

/**
 * @brief Repeats until the expression returns false, or the timeout is reached.
 */
#define WAIT_WITH_TIMEOUT(x) \
do \
{ \
    uint32_t duration = 0;\
    while((x)) \
    { \
        if(duration++ > TIMEOUT) \
        { \
            i2c_Reset(); \
            if(ok != NULL) \
                *ok = false; \
            return; \
        } \
    } \
} while(0)

/**
 * @brief Initializes the I2C bus.
 */
void i2c_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;
    
    // Enable the peripheral clock.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // Setup the pins as I2C SDA/SCL.
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_Pin;
    GPIO_Init(I2C_SDA_Port, &GPIO_InitStruct);
    
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_Pin;
    GPIO_Init(I2C_SCL_Port, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(I2C_SDA_Port, I2C_SDA_PinSource, I2C_GPIO_AF);  
    GPIO_PinAFConfig(I2C_SCL_Port, I2C_SCL_PinSource, I2C_GPIO_AF);
    
    // Setup the I2C peripheral.
    I2C_InitStruct.I2C_ClockSpeed = 400000; // [Hz].
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    
    I2C_DeInit(I2C_PERIPH);
    I2C_Init(I2C_PERIPH, &I2C_InitStruct);
    I2C_Cmd(I2C_PERIPH, ENABLE);
    
    //
    utils_DelayUs(50);
    I2C_GenerateSTART(I2C_PERIPH, ENABLE);
    utils_DelayUs(50);
    I2C_GenerateSTART(I2C_PERIPH, DISABLE);
    utils_DelayUs(50);
    
    I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
    utils_DelayUs(50);
    I2C_GenerateSTOP(I2C_PERIPH, DISABLE);
    utils_DelayUs(50);
}

/**
 * @brief Performs a stop condition, to abort the current transaction.
 */
void i2c_Reset(void)
{
    I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
    utils_DelayUs(50);
    I2C_GenerateSTOP(I2C_PERIPH, DISABLE);
    utils_DelayUs(50);
}

/**
 * @brief Writes a 8-bit value to a register.
 * @param slaveAddress 7-bit I2C slave address.
 * @param registerAddress 8-bit register address.
 * @param registerValue value to write to the 8-bit register.
 * @param ok the value pointed will be set to true if the operation completed
 * successfully, or to false if an error occured. If a NULL pointer is given,
 * then it is ignored.
 */
void i2c_WriteRegister(uint8_t slaveAddress, uint8_t registerAddress,
                       uint8_t registerValue, bool *ok)
{
    i2c_WriteMultiBytesRegister(slaveAddress, registerAddress,
                                &registerValue, 1, ok);
}

/**
 * @brief Writes several bytes to a register.
 * @param slaveAddress 7-bit I2C slave address.
 * @param registerAddress 8-bit register address.
 * @param registerValue values array to write to the 8-bit register.
 * @param registerSize number of bytes to write.
 * @param ok the value pointed will be set to true if the operation completed
 * successfully, or to false if an error occured. If a NULL pointer is given,
 * then it is ignored.
 */
void i2c_WriteMultiBytesRegister(uint8_t slaveAddress, uint8_t registerAddress,
                                 uint8_t const *registerValue,
                                 uint8_t registerSize, bool *ok)
{
    int i;

    // Wait until the I2C peripheral is ready.
    WAIT_WITH_TIMEOUT(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_BUSY));
  
	// Send a start condition, and wait until the slave has acknowledged.
    I2C_GenerateSTART(I2C_PERIPH, ENABLE);
    WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave address and write mode, and wait until the slave has
    // acknowledged.
    I2C_Send7bitAddress(I2C_PERIPH, slaveAddress<<1, I2C_Direction_Transmitter);
    WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // Write the register address.
    I2C_SendData(I2C_PERIPH, registerAddress);
    WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // Write the register content.
    for(i=0; i<registerSize; i++)
    {
        I2C_SendData(I2C_PERIPH, registerValue[i]);
        WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }
    
    // Release the bus.
    I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
    
    // Operation successful.
    if(ok != NULL)
        *ok = true;
}

/**
 * @brief Reads a 8-bit value from a register.
 * @param slaveAddress 7-bit I2C slave address.
 * @param registerAddress 8-bit register address.
 * @param ok the value pointed will be set to true if the operation completed
 * successfully, or to false if an error occured. If a NULL pointer is given,
 * then it is ignored.
 * @return the register value. If the operation failed (*ok set to false), then
 * 0 is returned.
 */
uint8_t i2c_ReadRegister(uint8_t slaveAddress, uint8_t registerAddress,
                         bool *ok)
{
    uint8_t registerValue = 0;

    i2c_ReadMultiBytesRegister(slaveAddress, registerAddress,
                               &registerValue, 1, ok);
                                       
    return registerValue;
}

/**
 * @brief Reads a multi bytes value from a register.
 * @param slaveAddress 7-bit I2C slave address.
 * @param registerAddress 8-bit register address.
 * @param readData array that will hold the read data. Its size should be at
 * least registerSize.
 * @param registerSize number of bytes to read.
 * @param ok the value pointed will be set to true if the operation completed
 * successfully, or to false if an error occured. If a NULL pointer is given,
 * then it is ignored.
 */
void i2c_ReadMultiBytesRegister(uint8_t slaveAddress, uint8_t registerAddress,
                                uint8_t *readData, uint8_t registerSize,
                                bool *ok)
{
    int i;
    
    // Wait until the I2C peripheral is ready.
    WAIT_WITH_TIMEOUT(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_BUSY));
  
	// Send a start condition, and wait until the slave has acknowledged.
	I2C_GenerateSTART(I2C_PERIPH, ENABLE);
	WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave address and write mode, and wait until the slave has
    // acknowledged.
	I2C_Send7bitAddress(I2C_PERIPH, slaveAddress<<1, I2C_Direction_Transmitter);
	WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // Write the register address.
    I2C_SendData(I2C_PERIPH, registerAddress);
    WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
        
    // Restart condition.
    I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
    utils_DelayUs(5);
    I2C_GenerateSTOP(I2C_PERIPH, DISABLE);
    utils_DelayUs(5);
    
    I2C_GenerateSTART(I2C_PERIPH, ENABLE);
    utils_DelayUs(5);
    I2C_GenerateSTART(I2C_PERIPH, DISABLE);
    utils_DelayUs(5);
    
    // Send slave address and write mode, and wait until the slave has
    // acknowledged.
	I2C_Send7bitAddress(I2C_PERIPH, slaveAddress<<1, I2C_Direction_Receiver);
	WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    // Read the register content.
    for(i=0; i<registerSize; i++)
    {
        if(i < registerSize - 1) // Not the last byte, need to acknowledge.
        {
            I2C_AcknowledgeConfig(I2C_PERIPH, ENABLE);
            WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_RECEIVED));
            readData[i] = I2C_ReceiveData(I2C_PERIPH);
        }
        else // Last byte to read, no acknowledge.
        {
            I2C_AcknowledgeConfig(I2C_PERIPH, DISABLE);
            WAIT_WITH_TIMEOUT(!I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_RECEIVED));
            readData[i] = I2C_ReceiveData(I2C_PERIPH);
        }
    }
    
    // Release the bus.
    I2C_GenerateSTOP(I2C_PERIPH, ENABLE);

    // Operation successful.
    if(ok != NULL)
        *ok = true;
}
