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

#include "communication.h"
#include "drivers/callback_timers.h"
#include "drivers/adc.h"
#include "drivers/dac.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/uart.h"
#include "lib/basic_filter.h"
#include "lib/pid.h"
#include "lib/utils.h"

#include <stdio.h>

#include "torque_regulator.h"

#define STREAMING_PERIOD 1000 // [us].

uint32_t selectedVariablesToStream; // Bitfield that indicates for each variable if it should be streamed or not.
uint8_t txBuffer[1024];

#define COMM_BUFFER_SIZE 4096
#define DEBUG_MESSAGE_BUFFER_SIZE 1024

uint8_t comm_packetTxBuffer[COMM_BUFFER_SIZE];
char comm_debugMessageBuffer[DEBUG_MESSAGE_BUFFER_SIZE];

cb_CircularBuffer *comm_rxQueue;
uint8_t rxCurrentMessageType = PC_MESSAGE_DO_NOTHING; // Current message type for RX bytes.
uint32_t rxBytesCount; // Number of received bytes for the current message.
uint8_t firstHalfByte; // First half of the data byte to receive.
uint8_t rxDataBytesBuffer[32]; // Data bytes received (ready to use, bytes already merged).

// SyncVar-related vars.
comm_SyncVar comm_syncVars[N_SYNCVARS_MAX];
uint8_t comm_nSyncVars;
volatile bool comm_varListLocked;
uint8_t comm_streamId;
uint8_t comm_nVarsToStream;
comm_SyncVar const* comm_streamedVars[N_SYNCVARS_MAX];
extern volatile uint32_t hapt_timestamp; // [us].

// Private functions.
void comm_SendPacket(uint8_t messageType, uint8_t *data, uint16_t dataLength);
void comm_SendPacketHeader(uint8_t type);
void comm_SendPacketContent(uint8_t *data, uint16_t dataLength);
void comm_HandleByte(uint8_t rxData);
void comm_Stream(void);
void comm_GetVar(comm_SyncVar const *syncVar, uint8_t *varValueData);
void comm_SetVar(comm_SyncVar *syncVar, uint8_t *varValueData);

/**
  * @brief Init the communication manager.
  */
void comm_Init(void)
{
    comm_nSyncVars = 0;
    comm_varListLocked = false;
    comm_streamId = 0;
    comm_nVarsToStream = 0;
 
    // Setup the UART peripheral, and specify the function that will be called
    // each time a byte is received.
    uart_Init();
    comm_rxQueue = uart_GetRxQueue();
    rxCurrentMessageType = PC_MESSAGE_DO_NOTHING;

    // Make the streaming function periodically called by the timer 7.
    cbt_SetCommLoopTimer(comm_Stream, STREAMING_PERIOD);
}

/**
 * @brief Updates the communication manager.
 * Send the bytes waiting in the TX buffer, and process the bytes received.
 */
void comm_Step(void)
{
    // Send the bytes in the TX queue, even if it is not full, to avoid latency.
    uart_FlushTx();

    // Interpret the bytes in the RX queue.
    uart_Step();

    while(!cb_IsEmpty(comm_rxQueue))
        comm_HandleByte(cb_Pull(comm_rxQueue));
}

/**
 * @brief Sends a packet to notify the PC that the board just (re)started.
 * This informs the PC software that the board is ready, and that the variables
 * list can be retrieved.
 */
void comm_NotifyReady(void)
{
    comm_SendPacket(STM_MESSAGE_START_INFO, NULL, 0);
}

/**
  * @brief Generates and sends a data streaming packet.
  */
void comm_Stream()
{
    // If the data streaming is enabled, send a stream packet to the PC.
    if(comm_nVarsToStream > 0)
    {
        uint8_t i;
        int nDataBytesToSend = 0;
        
        // Stream ID.
        txBuffer[nDataBytesToSend] = comm_streamId;
        nDataBytesToSend++;
        
        // Timestamp.
        memcpy(&txBuffer[nDataBytesToSend], (uint32_t*)&hapt_timestamp,
               sizeof(hapt_timestamp));
        nDataBytesToSend += sizeof(hapt_timestamp);

        // SyncVars values.
        for(i=0; i<comm_nVarsToStream; i++)
        {
            comm_SyncVar const* sv = comm_streamedVars[i];
            
            comm_GetVar(sv, &txBuffer[nDataBytesToSend]);

            nDataBytesToSend += sv->size;
        }

        comm_SendPacket(STM_MESSAGE_STREAMING_PACKET, txBuffer, nDataBytesToSend);
    }
}

/**
 * @brief Gets the value of a SyncVar.
 * @param syncVar: address of the SyncVar to get the value from.
 * @param varValueData: start address of an array to copy the raw bytes of the
 * value of the SyncVar.
 */
void comm_GetVar(comm_SyncVar const *syncVar, uint8_t *varValueData)
{
    comm_SyncVar const *v = syncVar;
    
    if(v->usesVarAddress)
        memcpy(varValueData, v->address, v->size);
    else
    {
        switch(v->type)
        {
        case BOOL:
            {
                bool tmpBool = ((bool (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpBool, v->size);
            }
            break;
        case UINT8:
            {
                uint8_t tmpUint8 = ((uint8_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpUint8, v->size);
            }
            break;
        case INT8:
            {
                int8_t tmpInt8 = ((int8_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpInt8, v->size);
            }
            break;
        case UINT16:
            {
                uint16_t tmpUint16 = ((uint16_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpUint16, v->size);
            }
            break;
        case INT16:
            {
                int16_t tmpInt16 = ((int16_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpInt16, v->size);
            }
            break;
        case UINT32:
            {
                uint32_t tmpUint32 = ((uint32_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpUint32, v->size);
            }
            break;
        case INT32:
            {
                int32_t tmpInt32 = ((int32_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpInt32, v->size);
            }
            break;
        case UINT64:
            {
                uint32_t tmpUint64 = ((uint64_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpUint64, v->size);
            }
            break;
        case INT64:
            {
                int32_t tmpInt64 = ((int64_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpInt64, v->size);
            }
            break;
        case FLOAT32:
            {
                float32_t tmpFloat = ((float32_t (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpFloat, v->size);
            }
            break;
        case FLOAT64:
            {
                double tmpDouble = ((double (*)(void))v->getFunc)();
                memcpy(varValueData, &tmpDouble, v->size);
            }
            break;
        }
    }
}

/**
 * @brief Sets the value of a SyncVar.
 * @param syncVar: address of the SyncVar to set the value.
 * @param varValueData: start address of an array to copy the raw bytes to the
 * SyncVar value.
 */
void comm_SetVar(comm_SyncVar *syncVar, uint8_t *varValueData)
{
    comm_SyncVar *v = syncVar;
    
    if(v->usesVarAddress)
    {
        if(v->access != READONLY)
            memcpy(v->address, varValueData, v->size);
    }
    else
    {
        if(v->setFunc == NULL)
            return;
    
        switch(v->type)
        {
        case BOOL:
            {
                bool tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(bool))v->setFunc)(tmp);
            }
            break;
        case UINT8:
            {
                uint8_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(uint8_t))v->setFunc)(tmp);
            }
            break;
        case INT8:
            {
                int8_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(int8_t))v->setFunc)(tmp);
            }
            break;
        case UINT16:
            {
                uint16_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(uint16_t))v->setFunc)(tmp);
            }
            break;
        case INT16:
            {
                int16_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(int16_t))v->setFunc)(tmp);
            }
            break;
        case UINT32:
            {
                uint32_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(uint32_t))v->setFunc)(tmp);
            }
            break;
        case INT32:
            {
                int32_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(int32_t))v->setFunc)(tmp);
            }
            break;
        case UINT64:
            {
                uint64_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(uint64_t))v->setFunc)(tmp);
            }
            break;
        case INT64:
            {
                int64_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(int64_t))v->setFunc)(tmp);
            }
            break;
        case FLOAT32:
            {
                float32_t tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(float32_t))v->setFunc)(tmp);
            }
            break;
        case FLOAT64:
            {
                double tmp;
                memcpy(&tmp, varValueData, v->size);
                ((void (*)(double))v->setFunc)(tmp);
            }
            break;
        }
    }
}

/**
  * @brief Sends a packet with a header and data bytes.
  * @param type: message type ("header" of the message).
  * @param data: array of data bytes to be sent ("content" of the message).
  * @param dataLength: number of data bytes to be sent.
  * @warning If sending a large amount of data, this function will block until
  * all the data could be written to the write buffer, which can be very long.
  */
void comm_SendPacket(uint8_t type, uint8_t *data, uint16_t dataLength)
{
    comm_SendPacketHeader(type);
    comm_SendPacketContent(data, dataLength);
}

/**
  * @brief Sends manually a packet header.
  * This function should be used along with comm_SendPacketContent(), in order
  * to send very large packets, with minimal memory consumption. Otherwise,
  * comm_SendPacket() should be used instead.
  * @param type: message type ("header" of the message).
  * @warning If the write buffers are full, this function will block until all
  * the data could be written to the write buffer, which can be very long.
  */
void comm_SendPacketHeader(uint8_t type)
{
    comm_packetTxBuffer[0] = ((1<<7) | type);
    uart_SendBytesAsync(comm_packetTxBuffer, 1);
}

/**
  * @brief Sends manually a part of packet content data.
  * In order to send a very large packet that do not fit in memory, the packet
  * is sent incrementally by calling comm_SendPacketHeader() once, then this
  * function several times.
  * Otherwise, comm_SendPacket() should be used instead.
  * @param data: array of data bytes to be sent ("content" of the message).
  * @param dataLength: number of data bytes to be sent.
  * @warning If the write buffers are full, this function will block until all
  * the data could be written to the write buffer, which can be very long.
  */
void comm_SendPacketContent(uint8_t *data, uint16_t dataLength)
{
    int i=0;

    uint8_t *p = comm_packetTxBuffer;
    
    for(i=0; i<dataLength; i++)
    {
        // Every data byte is splitted into two transmitted bytes.
        *p = ((data[i]&0xf0) >> 4); // Most significant bits.
        p++;
        *p = ((data[i]&0x0f)); // Least significant bits.
        p++;
    }

    uart_SendBytesAsync(comm_packetTxBuffer, dataLength*2);
}

/**
  * @brief Processes the received byte to interpret the messages.
  * @param rxData: the byte to be processed and interpreted.
  */
void comm_HandleByte(uint8_t rxData)
{
    if(rxData & (1<<7)) // The start byte has the most significant bit high.
    {
        rxCurrentMessageType = (rxData & ~(1<<7)); // Remove the start bit.
        rxBytesCount = 0;
    }
    else
        rxBytesCount++;
        
    if(rxBytesCount % 2 == 1) // First half of the data byte has been received.
        firstHalfByte = rxData; // Store it until the second half arrives.
    else // Second half of the data byte has been received (or no data bytes yet).
    {
        int dataBytesReady = rxBytesCount/2;
        rxDataBytesBuffer[dataBytesReady-1] = (firstHalfByte<<4) + (rxData & 0xf);
        
        switch(rxCurrentMessageType)
        {
        case PC_MESSAGE_DO_NOTHING:
            if(dataBytesReady == 0)
            {
                // Do nothing.
            }
            break;
            
        case PC_MESSAGE_PING:
            if(dataBytesReady == 0)
                comm_SendPacket(STM_MESSAGE_PINGBACK, NULL, 0);
            break;
            
        case PC_MESSAGE_GET_VARS_LIST:
            if(dataBytesReady == 0 && comm_varListLocked)
            {
                int16_t i;

                // Send the packet ID.
                comm_SendPacketHeader(STM_MESSAGE_VARS_LIST);

                // Send the packet content, incrementally.
                txBuffer[0] = (uint8_t)comm_nSyncVars;
                comm_SendPacketContent(txBuffer, 1); // SyncVars list size.

                for(i=0; i<comm_nSyncVars; i++)
                {
                    uint8_t *p = &txBuffer[0];

                    memcpy(p, comm_syncVars[i].name, SYNCVAR_NAME_SIZE);
                    p += SYNCVAR_NAME_SIZE;
                    
                    *p = (uint8_t)comm_syncVars[i].type;
                    p++;
                    
                    *p = (uint8_t)comm_syncVars[i].access;
                    p++;
                    
                    *p = (uint8_t)comm_syncVars[i].size;
                    p++;

                    comm_SendPacketContent(txBuffer, SYNCVAR_NAME_SIZE + 3);
                }
            }
            break;

        case PC_MESSAGE_GET_VAR:
            if(dataBytesReady == 1)
            {
                comm_SyncVar *v;

                // Extract the index, that indicates which variable will be sent.
                uint8_t variableIndex = rxDataBytesBuffer[0];

                // If the variable index is out of range, ignore the request.
                if(variableIndex >= comm_nSyncVars)
                    return;
                    
                v = &comm_syncVars[variableIndex];
                    
                // If the variable is not readable, ignore the request.
                if((v->usesVarAddress && v->access == WRITEONLY) ||
                   (!v->usesVarAddress && v->getFunc == NULL))
                {
                    break;
                }

                // Prepare the message to be sent to the PC.
                // First byte: variable index.
                txBuffer[0] = variableIndex;

                // Following bytes: actual value of the variable.
                comm_GetVar(v, &txBuffer[1]);

                // Send the message to the PC.
                comm_SendPacket(STM_MESSAGE_VAR, txBuffer, 1 + v->size);
            }
            break;

        case PC_MESSAGE_SET_VAR:
            if(dataBytesReady >= 1)
            {
                comm_SyncVar *v;
                uint8_t variableIndex;

                // Extract the index (that indicates which variable will change),
                // and the new value of the variable.
                variableIndex = rxDataBytesBuffer[0];
                
                // If the variable index is out of range, ignore the request.
                if(variableIndex >= comm_nSyncVars)
                    return;
                    
                v = &comm_syncVars[variableIndex];

                // Set the selected variable with the new value.
                if(dataBytesReady == 1 + v->size)
                    comm_SetVar(v, &rxDataBytesBuffer[1]);
            }
            break;

        case PC_MESSAGE_SET_STREAMED_VAR:
            if(dataBytesReady >= 1)
            {
                int i;
                comm_nVarsToStream = rxDataBytesBuffer[0];
                
                if(dataBytesReady != 1 + 1 + comm_nVarsToStream)
                    return;
                    
                comm_streamId = rxDataBytesBuffer[1];
                    
                for(i=0; i<comm_nVarsToStream; i++)
                    comm_streamedVars[i] = &comm_syncVars[rxDataBytesBuffer[2+i]];
            }
            break;
            
        default: // No data bytes for the other message types.
            break;
        }
    }
}

/**
 * @brief Adds a monitored variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address: a pointer to the variable to monitor.
 * @param type: the type of the variable.
 * @param size: the size of the variable [bytes].
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorVar(const char name[], void *address, comm_VarType type,
                     uint8_t size, comm_VarAccess access)
{
    comm_SyncVar v;
    
    // Adding a variable to the list is not allowed if it has been locked.
    if(comm_varListLocked)
    {
        comm_SendDebugMessage("Warning: can't add the \"%s\" SyncVar, because "
                              "the list has already been locked.", name);
        return;
    }
    
    // Adding a variable to the list is not allowed if it is full.
    if(comm_nSyncVars >= N_SYNCVARS_MAX)
    {
        comm_SendDebugMessage("Warning: can't add the \"%s\" SyncVar, because "
                              "the list is full.", name);
        return;
    }
    
    // Trim the name if it is too long.
    if(strlen(name) > SYNCVAR_NAME_SIZE - 1)
    {
        memcpy(v.name, name, SYNCVAR_NAME_SIZE - 1);
        v.name[SYNCVAR_NAME_SIZE - 1] = '\0';
    }
    else
        strcpy(v.name, name);

    // Build the SyncVar.
    v.address = address;
    v.type = type;
    v.size = size;
    v.access = access;
    v.usesVarAddress = true;

    // Add the SyncVar to the list.
    comm_syncVars[comm_nSyncVars] = v;
    comm_nSyncVars++;
}

/**
 * @brief Adds a monitored variable to the list, with getter/setter functions.
 * If the getter and setter are set to an actual function address (not NULL),
 * the variable will be READWRITE. If the getter is NULL but setter is not NULL,
 * the variable will be WRITEONLY. If the getter is not NULL the setter is NULL,
 * the variable will be READONLY. If the getter and setter are both NULL, the
 * variable will not be added to the variables list.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param type: the type of the variable.
 * @param size: the size of the variable [bytes].
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorVarFunc(const char name[], comm_VarType type, uint8_t size,
                         void (*getFunc)(void), void (*setFunc)(void))
{
    comm_SyncVar v;
    
    // Adding a variable to the list is not allowed if it has been locked.
    if(comm_varListLocked)
    {
        comm_SendDebugMessage("Warning: can't add the \"%s\" SyncVar, because "
                              "the list has already been locked.", name);
        return;
    }
    
    // Adding a variable to the list is not allowed if it is full.
    if(comm_nSyncVars >= N_SYNCVARS_MAX)
    {
        comm_SendDebugMessage("Warning: can't add the \"%s\" SyncVar, because "
                              "the list is full.", name);
        return;
    }

    // Trim the name if it is too long.
    if(strlen(name) > SYNCVAR_NAME_SIZE - 1)
    {
        memcpy(&v.name, &name, SYNCVAR_NAME_SIZE - 1);
        v.name[SYNCVAR_NAME_SIZE - 1] = '\0';
    }
    else
        strcpy(v.name, name);

    // Build the SyncVar.
    v.getFunc = getFunc;
    v.setFunc = setFunc;
    v.type = type;
    v.size = size;
    v.usesVarAddress = false;
    
    // Determine the variable access.
    if(getFunc != NULL && setFunc == NULL)
        v.access = READONLY;
    else if(getFunc == NULL && setFunc != NULL)
        v.access = WRITEONLY;
    else if(getFunc != NULL && setFunc != NULL)
        v.access = READWRITE;
    else
        return; // No function provided at all, ignoring var.

    // Add the SyncVar to the list.
    comm_syncVars[comm_nSyncVars] = v;
    comm_nSyncVars++;
}

/**
 * @brief Adds a monitored bool variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorBool(const char name[], bool *address,
                      comm_VarAccess access)
{
    comm_monitorVar(name, address, BOOL, 1, access);
}

/**
 * @brief Adds a monitored uint8_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorUint8(const char name[], uint8_t *address,
                       comm_VarAccess access)
{
    comm_monitorVar(name, address, UINT8, 1, access);
}

/**
 * @brief Adds a monitored int8_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorInt8(const char name[], int8_t *address,
                      comm_VarAccess access)
{
    comm_monitorVar(name, address, INT8, 1, access);
}

/**
 * @brief Adds a monitored uint16_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorUint16(const char name[], uint16_t *address,
                        comm_VarAccess access)
{
    comm_monitorVar(name, address, UINT16, 2, access);
}

/**
 * @brief Adds a monitored int16_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorInt16(const char name[], int16_t *address,
                       comm_VarAccess access)
{
    comm_monitorVar(name, address, INT16, 2, access);
}

/**
 * @brief Adds a monitored uint32_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorUint32(const char name[], uint32_t *address,
                        comm_VarAccess access)
{
    comm_monitorVar(name, address, UINT32, 4, access);
}

/**
 * @brief Adds a monitored int32_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorInt32(const char name[], int32_t *address,
                       comm_VarAccess access)
{
    comm_monitorVar(name, address, INT32, 4, access);
}

/**
 * @brief Adds a monitored uint64_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorUint64(const char name[], uint64_t *address,
                        comm_VarAccess access)
{
    comm_monitorVar(name, address, UINT64, 8, access);
}

/**
 * @brief Adds a monitored int64_t variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorInt64(const char name[], int64_t *address,
                       comm_VarAccess access)
{
    comm_monitorVar(name, address, INT64, 8, access);
}

/**
 * @brief Adds a monitored float variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorFloat(const char name[], float *address,
                       comm_VarAccess access)
{
    comm_monitorVar(name, address, FLOAT32, 4, access);
}

/**
 * @brief Adds a monitored double variable to the list, with its address.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param address a pointer to the variable to monitor.
 * @param access: the access rights to this variable (READONLY, WRITEONLY, or
 * READWRITE).
 */
void comm_monitorDouble(const char name[], double *address,
                        comm_VarAccess access)
{
    comm_monitorVar(name, address, FLOAT64, 8, access);
}

/**
 * @brief Adds a monitored bool variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorBoolFunc(const char name[],
                          bool (*getFunc)(void), void (*setFunc)(bool))
{
    comm_monitorVarFunc(name, BOOL, 1, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored uint8_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorUint8Func(const char name[],
                           uint8_t (*getFunc)(void), void (*setFunc)(uint8_t))
{
    comm_monitorVarFunc(name, UINT8, 1, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored int8_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorInt8Func(const char name[],
                          int8_t (*getFunc)(void), void (*setFunc)(int8_t))
{
    comm_monitorVarFunc(name, INT8, 1, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored uint16_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorUint16Func(const char name[],
                            uint16_t (*getFunc)(void),
                            void (*setFunc)(uint16_t))
{
    comm_monitorVarFunc(name, UINT16, 2, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored int16_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorInt16Func(const char name[],
                           int16_t (*getFunc)(void), void (*setFunc)(int16_t))
{
    comm_monitorVarFunc(name, INT16, 2, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored uint32_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorUint32Func(const char name[],
                            uint32_t (*getFunc)(void),
                            void (*setFunc)(uint32_t))
{
    comm_monitorVarFunc(name, UINT32, 4, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored int32_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorInt32Func(const char name[],
                           int32_t (*getFunc)(void), void (*setFunc)(int32_t))
{
    comm_monitorVarFunc(name, INT32, 4, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored uint64_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorUint64Func(const char name[],
                            uint64_t (*getFunc)(void),
                            void (*setFunc)(uint64_t))
{
    comm_monitorVarFunc(name, UINT64, 8, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored int64_t variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorInt64Func(const char name[],
                           int64_t (*getFunc)(void), void (*setFunc)(int64_t))
{
    comm_monitorVarFunc(name, INT64, 8, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Adds a monitored float variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorFloatFunc(const char name[],
                           float (*getFunc)(void), void (*setFunc)(float))
{
    comm_monitorVarFunc(name, FLOAT32, 4, (void (*)(void))getFunc, (void (*)(void))setFunc);
}


/**
 * @brief Adds a monitored double variable to the list, with getter/setter.
 * @param name: the description of the variable, as it should be displayed to
 * the user. It should also include the unit, if relevant.
 * @param getFunc: function pointer on the getter function.
 * @param setFunc: function pointer on the setter function.
 */
void comm_monitorDoubleFunc(const char name[],
                            double (*getFunc)(void), void (*setFunc)(double))
{
    comm_monitorVarFunc(name, FLOAT64, 8, (void (*)(void))getFunc, (void (*)(void))setFunc);
}

/**
 * @brief Locks the monitored variables, so it can be used.
 * After the call to this function, adding variables will not be possible
 * anymore. The PC will not be able to get the variables list until this
 * function is called.
 */
void comm_LockSyncVarsList(void)
{
    comm_varListLocked = true;
}

/**
 * @brief Sends a debug message to the computer.
 * @param format format string.
 * @param ... variables to be printed in the format string.
 */
void comm_SendDebugMessage(const char* format, ...)
{
    uint16_t length;
    va_list args;
    
    va_start(args, format);
    
	length = vsnprintf(comm_debugMessageBuffer, DEBUG_MESSAGE_BUFFER_SIZE,
                       format, args);
    
    comm_SendPacket(STM_MESSAGE_DEBUG_TEXT, (uint8_t*)comm_debugMessageBuffer,
                    length+1);

    va_end(args);
}

