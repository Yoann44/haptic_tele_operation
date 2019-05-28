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

#include "circular_buffer.h"
#include "utils.h"

/**
 * @brief Initializes a cb_CircularBuffer structure.
 * Initializes a cb_CircularBuffer structure with the given buffer. The buffer
 * has to be provided by the user, to avoid dynamic memory allocation.
 * @param cb the cb_CircularBuffer structure to initialize.
 * @param buffer pointer to an existing array.
 * @param bufferSize length of the circular buffer. The given buffer should have
 * a size greater or equal to this value.
 */
void cb_Init(cb_CircularBuffer* cb, uint8_t *buffer, uint16_t bufferSize)
{
    cb->buffer = buffer;
    cb->bufferSize = bufferSize;
    cb->readIndex = 0;
    cb->writeIndex = 0;
}

/**
 * @brief Gets the number of bytes stored in the queue.
 * @param cb the cb_CircularBuffer to check.
 * @return the number of bytes stored in the queue.
 */
uint16_t cb_ItemsCount(cb_CircularBuffer* cb)
{
    if (cb->writeIndex >= cb->readIndex)
        return cb->writeIndex - cb->readIndex;
    else
        return cb->bufferSize - cb->readIndex + cb->writeIndex;
}

/**
 * @brief Check if the queue is empty.
 * @param cb the cb_CircularBuffer to check.
 * @return 1 if there are no bytes stored in the queue, 0 otherwise.
 */
bool cb_IsEmpty(cb_CircularBuffer* cb)
{
    return cb->writeIndex == cb->readIndex;
}

/**
 * @brief Check if the queue is full.
 * @param cb the cb_CircularBuffer to check.
 * @return 1 if the queue is full, 0 otherwise.
 */
bool cb_IsFull(cb_CircularBuffer* cb)
{
    if (cb->writeIndex == cb->readIndex)
        return false;
    else if (cb->readIndex < cb->writeIndex)
        return (cb->readIndex == 0 && cb->writeIndex == cb->bufferSize - 1);
    else
        return (cb->writeIndex == cb->readIndex-1);
}

/**
 * @brief Add a item at the back of the queue.
 * @param cb the cb_CircularBuffer to affect.
 * @param newElem the byte to add to the back of the queue.
 * @warning If the queue is already full, this function does nothing if
 * CPU_TRAPS_ENABLED is 0. If CPU_TRAPS_ENABLED is 1, this function will block
 * forever the program execution, so the problem can be found with the
 * debugger.
 */
void cb_Push(cb_CircularBuffer* cb, uint8_t newElem)
{
    if (!cb_IsFull(cb)) // Not full.
    {
        cb->buffer[cb->writeIndex] = newElem;
        cb->writeIndex++;

        if (cb->writeIndex >= cb->bufferSize)
            cb->writeIndex = 0;
    }
    else // Error, can't push an item, because the queue is full.
        utils_TrapCpu();
}

/**
 * @brief Extract the item at the front of the queue.
 * Returns the value of the item at the front of the queue, and remove this item
 * from the queue.
 * @param cb the cb_CircularBuffer to affect.
 * @return the value of the byte that has been extracted from the queue.
 * @warning If the queue is empty and CPU_TRAPS_ENABLED is 0, this function
 * returns 0. If CPU_TRAPS_ENABLED is 1, this function will block forever the
 * program execution, so the problem can be found with the debugger.
 */
uint8_t cb_Pull(cb_CircularBuffer* cb)
{
    if (cb->writeIndex != cb->readIndex) // Not empty.
    {
        uint8_t pulled = cb->buffer[cb->readIndex];
        cb->readIndex++;

        if (cb->readIndex >= cb->bufferSize)
            cb->readIndex = 0;

        return pulled;
    }
    else // Error, can't pull an item, because the queue is empty.
    {
        utils_TrapCpu();
        return 0;
    }
}
