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

#ifndef __EXUART_H
#define __EXUART_H

#include "../main.h"
#include "../lib/circular_buffer.h"

/** @defgroup EXT_UART Driver / Extension UART
  * @brief Driver for the UART serial communication peripheral.
  *
  * This driver controls the UART peripheral of the STM32, connected to the
  * digital extension connector.
  *
  * Call exuart_Init() first in the initialization code. To send data, call
  * exuart_SendByteAsync(). To receive data, check first that bytes are
  * available by calling exuart_ReceivedBytesCount(), then call
  * exuart_GetByte().
  *
  * @addtogroup EXT_UART
  * @{
  */

void exuart_Init(uint32_t baudRate);
void exuart_SendByteAsync(uint8_t data);
void exuart_SendBytesAsync(uint8_t *data, int length);
uint16_t exuart_ReceivedBytesCount(void);
uint8_t exuart_GetByte(void);

/**
  * @}
  */

#endif
