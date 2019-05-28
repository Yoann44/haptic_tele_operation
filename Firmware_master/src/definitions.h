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

#ifndef DEF_DEFINITIONS_H
#define DEF_DEFINITIONS_H

#include <stdbool.h>

// Message IDs sent by host (PC) to the device (STM).
typedef enum
{
    PC_MESSAGE_DO_NOTHING = 0, ///< Do nothing.
    PC_MESSAGE_PING, ///< Request the board an answer, to check the connection status.
    PC_MESSAGE_GET_VARS_LIST, ///< Request the SyncVars list.
    PC_MESSAGE_SET_STREAMED_VAR, ///< Set the variables to be streamed continuously.
    PC_MESSAGE_GET_VAR, ///< Request the device to send the selected value.
    PC_MESSAGE_SET_VAR ///< Set the selected variable.
} comm_PcMessage;

// Message IDs sent by device (STM) to the device (PC).
typedef enum
{
    STM_MESSAGE_PINGBACK = 0, ///< Response to a ping request.
    STM_MESSAGE_VAR, ///< Variable state.
    STM_MESSAGE_STREAMING_PACKET, ///< Streaming packet.
    STM_MESSAGE_DEBUG_TEXT, ///< Debug text message.
    STM_MESSAGE_VARS_LIST, ///< Monitored variables list.
    STM_MESSAGE_START_INFO ///< Notification that the board has (re)started.
} comm_StmMessage;

// SyncVar.
#define N_SYNCVARS_MAX 255 // Maximum number of SyncVars.
#define SYNCVAR_NAME_SIZE 50 // Max size of a SyncVar name, including the '\0' trailing character.

typedef enum
{
    READONLY = 0,
    WRITEONLY,
    READWRITE
} comm_VarAccess;

typedef enum
{
    BOOL = 0,
    UINT8, INT8, UINT16, INT16, UINT32, INT32, UINT64, INT64,
    FLOAT32, FLOAT64
} comm_VarType;

// UART baudrate [b/s].
#define UART_BAUDRATE 1843200

#endif
