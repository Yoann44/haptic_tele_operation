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

#ifndef __I2C_H
#define __I2C_H

#include "../main.h"

#define I2C_SDA_Pin        GPIO_Pin_9
#define I2C_SDA_PinSource  GPIO_PinSource9
#define I2C_SDA_Port       GPIOB
#define I2C_SCL_Pin        GPIO_Pin_8
#define I2C_SCL_PinSource  GPIO_PinSource8
#define I2C_SCL_Port       GPIOB

#define I2C_PERIPH I2C1
#define I2C_GPIO_AF GPIO_AF_I2C1

/** @defgroup I2C Driver / I2C
  * @brief Driver for the I2C bus communication peripheral.
  *
  * This driver controls the I2C peripheral of the STM32, that is accessible
  * from the digital extension connector of the HRI board.
  *
  * Call i2c_Init() first in the initialization code. Then, the communication
  * function i2c_readRegister() or i2c_writeRegister() can be used.
  * Note that both functions are synchronous, hence blocking the execution until
  * the operation completes.
  *
  * @addtogroup I2C
  * @{
  */

void i2c_Init(void);
void i2c_WriteRegister(uint8_t slaveAddress, uint8_t registerAddress,
                       uint8_t registerValue, bool *ok);
void i2c_WriteMultiBytesRegister(uint8_t slaveAddress, uint8_t registerAddress,
                                 uint8_t const *registerValue,
                                 uint8_t registerSize, bool *ok);
uint8_t i2c_ReadRegister(uint8_t slaveAddress, uint8_t registerAddress,
                         bool *ok);
void i2c_ReadMultiBytesRegister(uint8_t slaveAddress, uint8_t registerAddress,
                                uint8_t *readData, uint8_t registerSize,
                                bool *ok);

/**
  * @}
  */

#endif
