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

#ifndef __UART_H
#define __UART_H

#include "../main.h"
#include "../lib/circular_buffer.h"

#define USART_RX_Pin								GPIO_Pin_5
#define USART_RX_PinSource					GPIO_PinSource5
#define USART_RX_Port								GPIOD
#define USART_TX_Pin								GPIO_Pin_6
#define USART_TX_PinSource					GPIO_PinSource6
#define USART_TX_Port								GPIOD

#define USART_PC_COMM  USART2 // UART peripheral used for the comm with the PC.

/** @defgroup UART Driver / UART
  * @brief Driver for the UART serial communication peripheral.
  *
  * This driver controls the UART peripheral of the STM32, connected to the
  * USB-to-serial chip.
  *
  * Call uart_Init() first in the initialization code, specifiying the function
  * to call when a byte arrives (sent from the computer). To send data, call
  * uart_SendByte() or uart_SendBytes().
  *
  * Note that this module should not be used directly. It is a better option to
  * use it through the Communication module, to benefit from the already
  * implemented communication protocol between the board and MATLAB.
  *
  * @addtogroup UART
  * @{
  */

/**
  * Typedef for a pointer to a function to call automatically when a byte
  * arrives from the computer.
  */
typedef void (*uart_rxByteHandlerFunc)(uint8_t rxByte);

void uart_Init(void);
void uart_Step(void);
cb_CircularBuffer* uart_GetRxQueue(void);
void uart_SendByteAsync(uint8_t data);
void uart_SendBytesAsync(uint8_t *data, int length);
void uart_FlushTx(void);

/**
  * @}
  */

#endif
