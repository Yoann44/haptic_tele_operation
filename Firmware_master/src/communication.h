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

#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "main.h"
#include "definitions.h"

#include <stdarg.h>

/** @defgroup Communication Main / Communication
  * @brief Control the communication with the computer.
  *
  * This module controls all the communication logic between the board and the
  * computer. It uses a specific communication protocol between the
  * computer and the board, with a system of messages. Thanks to this, the
  * the MATLAB application is able to get the value of selected variables on the
  * STM, and is even capable of modifiying them remotely.
  *
  * Make sure that the files communication.h/.c are up-to-date with the Excel
  * spreadsheet "Protocol description.xlsx".
  *
  * Call comm_Init() to setup this module. Its interrupt function will be called
  * automatically when a message arrives, or periodically when the data
  * streaming is enabled.
  *
  * @addtogroup Communication
  * @{
  */

typedef struct
{
    char name[SYNCVAR_NAME_SIZE];
    void *address;
    comm_VarType type;
    uint8_t size;
    comm_VarAccess access;
    bool usesVarAddress;
    void (*getFunc)(void);
    void (*setFunc)(void);
} comm_SyncVar;


void comm_Init(void);
void comm_Step(void);

void comm_NotifyReady(void);

void comm_monitorVar(const char name[], void *address, comm_VarType type,
                     uint8_t size, comm_VarAccess access);
void comm_monitorVarFunc(const char name[], comm_VarType type, uint8_t size,
                         void (*getFunc)(void), void (*setFunc)(void));
                     
void comm_monitorBool(const char name[], bool *address,
                      comm_VarAccess access);
void comm_monitorUint8(const char name[], uint8_t *address,
                       comm_VarAccess access);
void comm_monitorInt8(const char name[], int8_t *address,
                      comm_VarAccess access);
void comm_monitorUint16(const char name[], uint16_t *address,
                        comm_VarAccess access);
void comm_monitorInt16(const char name[], int16_t *address,
                       comm_VarAccess access);
void comm_monitorUint32(const char name[], uint32_t *address,
                        comm_VarAccess access);
void comm_monitorInt32(const char name[], int32_t *address,
                       comm_VarAccess access);
void comm_monitorUint64(const char name[], uint64_t *address,
                        comm_VarAccess access);
void comm_monitorInt64(const char name[], int64_t *address,
                       comm_VarAccess access);
void comm_monitorFloat(const char name[], float *address,
                       comm_VarAccess access);
void comm_monitorDouble(const char name[], double *address,
                        comm_VarAccess access);
                        
void comm_monitorBoolFunc(const char name[],
                          bool (*getFunc)(void), void (*setFunc)(bool));
void comm_monitorUint8Func(const char name[],
                           uint8_t (*getFunc)(void), void (*setFunc)(uint8_t));
void comm_monitorInt8Func(const char name[],
                          int8_t (*getFunc)(void), void (*setFunc)(int8_t));
void comm_monitorUint16Func(const char name[],
                            uint16_t (*getFunc)(void),
                            void (*setFunc)(uint16_t));
void comm_monitorInt16Func(const char name[],
                           int16_t (*getFunc)(void), void (*setFunc)(int16_t));
void comm_monitorUint32Func(const char name[],
                            uint32_t (*getFunc)(void),
                            void (*setFunc)(uint32_t));
void comm_monitorInt32Func(const char name[],
                           int32_t (*getFunc)(void), void (*setFunc)(int32_t));
void comm_monitorUint64Func(const char name[],
                            uint64_t (*getFunc)(void),
                            void (*setFunc)(uint64_t));
void comm_monitorInt64Func(const char name[],
                           int64_t (*getFunc)(void), void (*setFunc)(int64_t));
void comm_monitorFloatFunc(const char name[],
                           float (*getFunc)(void), void (*setFunc)(float));
void comm_monitorDoubleFunc(const char name[],
                            double (*getFunc)(void), void (*setFunc)(double));

void comm_LockSyncVarsList(void);

void comm_SendDebugMessage(const char *format, ...);

/**
 * @brief Sends a debug message to the computer, with decimation.
 * This macro is useful to print human-readable text, in a fast loop, to avoid
 * overloading the communication bus, or the computer.
 * @param decimation this macro will actually print once out of decimation, and
 * do nothing otherwise.
 * @param format format string. See the printf() documentation for format
 * specification.
 * @param ... variables to be printed in the format string.
 */ 
#define comm_SendDebugMessageDecimated(decimation, format, ...) \
do \
{ \
    static int comm_decim##__COUNTER__ = 0; \
    if(comm_decim##__COUNTER__++ % decimation == 0) \
    { \
        comm_SendDebugMessage(format, ##__VA_ARGS__); \
    } \
} while(0)

/**
  * @}
  */

#endif
