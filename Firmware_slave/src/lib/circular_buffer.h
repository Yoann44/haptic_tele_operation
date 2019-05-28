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

#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include "../main.h"

/** @defgroup CircularBuffer Lib / Circular buffer
  * @brief Bytes queue (FIFO) implemented with a circular buffer.
  *
  * This bytes container works as a queue, which means "first in, first out".
  * Create a cb_CircularBuffer structure, and initialize it with cb_Init().
  * Then, add bytes using cb_Push(), and extract them with cb_Pull().
  *
  * @ingroup Lib
  * @addtogroup CircularBuffer
  * @{
  */

/**
 * @brief Circular buffer structure.
 */
typedef struct
{
    uint8_t *buffer; ///< Pointer to the byte buffer.
    uint16_t bufferSize; ///< Size of buffer.
    volatile uint16_t readIndex, ///< Index of the element at the front of the queue.
                      writeIndex; ///< Index of the next free location at the end of the queue.
} cb_CircularBuffer;

void cb_Init(cb_CircularBuffer* cb, uint8_t *buffer, uint16_t bufferSize);
uint16_t cb_ItemsCount(cb_CircularBuffer* cb);
bool cb_IsEmpty(cb_CircularBuffer* cb);
bool cb_IsFull(cb_CircularBuffer* cb);
void cb_Push(cb_CircularBuffer* cb, uint8_t newElem);
uint8_t cb_Pull(cb_CircularBuffer* cb);

/**
  * @}
  */

#endif
