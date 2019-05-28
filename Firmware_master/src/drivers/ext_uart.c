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

#include "ext_uart.h"
#include "../lib/utils.h"

#define EXUART_RX_Pin GPIO_Pin_7
#define EXUART_RX_PinSource GPIO_PinSource7
#define EXUART_RX_Port GPIOB
#define EXUART_TX_Pin GPIO_Pin_6
#define EXUART_TX_PinSource GPIO_PinSource6
#define EXUART_TX_Port GPIOB

#define EXUART_PERIPH USART1

#define RX_DMA DMA2_Stream2
#define RX_DMA_CHANNEL DMA_Channel_4
#define TX_DMA DMA2_Stream7
#define TX_DMA_CHANNEL DMA_Channel_4

#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 1024
#define USER_RX_QUEUE_SIZE 2048

uint8_t exuart_txBuffer[2][TX_BUFFER_SIZE];
uint8_t exuart_currentTxBufferToWriteTo;
uint16_t exuart_txBufferIndex;
uint16_t exuart_nBytesToTransferDma;

uint8_t exuart_rxBuffer[RX_BUFFER_SIZE];
uint8_t const * exuart_rxBuffTail;

uint8_t exuart_userRxQueue[USER_RX_QUEUE_SIZE];
cb_CircularBuffer exuart_rxQueue;

#define UART_DMA_TX_IS_BUSY (DMA_GetCmdStatus(TX_DMA) == ENABLE && DMA_GetCurrDataCounter(TX_DMA) > 0)

void exuart_FlushTx(void);

/**
  * @brief Initializes the UART module.
  * @param baudRate: UART communication frequency (baud rate) [b/s].
  */
void exuart_Init(uint32_t baudRate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;   
    DMA_InitTypeDef DMA_InitStruct;
    
    // Enable UART and DMA peripherals clocks.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    // Setup GPIOs as UART pins.
    GPIO_InitStruct.GPIO_Pin   = EXUART_TX_Pin;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(EXUART_TX_Port, &GPIO_InitStruct);
    GPIO_PinAFConfig(EXUART_TX_Port, EXUART_TX_PinSource, GPIO_AF_USART1);
    
    GPIO_InitStruct.GPIO_Pin = EXUART_RX_Pin;
    GPIO_Init(EXUART_RX_Port, &GPIO_InitStruct);     
    GPIO_PinAFConfig(EXUART_RX_Port, EXUART_RX_PinSource, GPIO_AF_USART1);

    // Setup the UART peripheral.
    USART_InitStruct.USART_BaudRate            = baudRate;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(EXUART_PERIPH, &USART_InitStruct);
    
    USART_Cmd(EXUART_PERIPH, ENABLE);
    
    // Setup the DMA for RX.
    DMA_DeInit(RX_DMA);
    
    DMA_InitStruct.DMA_Channel = RX_DMA_CHANNEL;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&EXUART_PERIPH->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&exuart_rxBuffer[0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = RX_BUFFER_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(RX_DMA, &DMA_InitStruct);

    USART_DMACmd(EXUART_PERIPH, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    DMA_ITConfig(RX_DMA, DMA_IT_TC, ENABLE);
    DMA_Cmd(RX_DMA, ENABLE);
    
    exuart_rxBuffTail = &exuart_rxBuffer[0];
    
    // Setup the DMA for TX.
    DMA_DeInit(TX_DMA);
    
    DMA_InitStruct.DMA_Channel = TX_DMA_CHANNEL;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&EXUART_PERIPH->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&exuart_txBuffer[0][0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = TX_BUFFER_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(TX_DMA, &DMA_InitStruct);

    USART_DMACmd(EXUART_PERIPH, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    DMA_ITConfig(TX_DMA, DMA_IT_TC, ENABLE);
    
    // Initialize the RX circular buffer.
    cb_Init(&exuart_rxQueue, exuart_userRxQueue, USER_RX_QUEUE_SIZE);
    
    // Initialize the variables for the UART TX.
    exuart_currentTxBufferToWriteTo = 0;
    exuart_txBufferIndex = 0;
    exuart_nBytesToTransferDma = 0;
}

/**
 * @brief Gets the number of bytes received and ready to read.
 * @return the number of bytes ready to be acquired with exuart_GetByte().
 * @remark This function must called often, otherwise the DMA RX buffer may be
 * full, and data will be lost.
 */
uint16_t exuart_ReceivedBytesCount(void)
{
    // Get the location of the location currently pointed by the DMA.
    uint8_t const * head = exuart_rxBuffer + RX_BUFFER_SIZE
                           - DMA_GetCurrDataCounter(RX_DMA);
                           
    // Even if the STM32F4 reference manual (RM0090) states that the NDTR
    // register is decremented after the transfer, this is not the case.
    // So we wait a few cycles to be sure that the DMA actually performed the
    // transfer.
    utils_DelayUs(1);

    // RX: add the received bytes into the user queue.
    while(exuart_rxBuffTail != head)
    {
        uint8_t b =  *exuart_rxBuffTail;
        cb_Push(&exuart_rxQueue, b);

        exuart_rxBuffTail++;

        if(exuart_rxBuffTail >= exuart_rxBuffer + RX_BUFFER_SIZE)
            exuart_rxBuffTail -= RX_BUFFER_SIZE;
    }
    
    // Return the number of bytes ready to get.
    return cb_ItemsCount(&exuart_rxQueue);
}

/**
 * @brief Gets the next received byte.
 * @return the value of the received byte.
 * @remark call exuart_ReceivedBytesCount() before calling this function.
 * @warning this function return 0 if no byte is available.
 */
uint8_t exuart_GetByte(void)
{
    if(cb_ItemsCount(&exuart_rxQueue) > 0)
        return cb_Pull(&exuart_rxQueue);
    else
        return 0;
}

/**
 * @brief Starts the DMA transfer to send bytes to UART peripheral.
 */
void exuart_StartDma(void)
{
    DMA_InitTypeDef DMA_InitStruct;
    
    //
    exuart_nBytesToTransferDma = exuart_txBufferIndex;
    exuart_currentTxBufferToWriteTo = !exuart_currentTxBufferToWriteTo;
    exuart_txBufferIndex = 0;

    // Start the DMA transfer.
    DMA_DeInit(TX_DMA);
    
    DMA_InitStruct.DMA_Channel = TX_DMA_CHANNEL;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&EXUART_PERIPH->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&exuart_txBuffer[!exuart_currentTxBufferToWriteTo][0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = exuart_nBytesToTransferDma;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(TX_DMA, &DMA_InitStruct);
                          
    DMA_Cmd(TX_DMA, ENABLE);
}


/**
 * @brief Asynchronously sends the given byte through the UART bus.
 * @param data the data byte to send.
 */
void exuart_SendByteAsync(uint8_t data)
{
    // Check that it is possible to write in the current buffer.
    if(exuart_txBufferIndex >= TX_BUFFER_SIZE)
    {
        // If the DMA is idle, switch to the other buffer.
        if(UART_DMA_TX_IS_BUSY)
        {
            utils_TrapCpu(); // Error, can't write anywhere!
            return;
        }
        else
            exuart_StartDma();
    }

    // Write the byte in the buffer.
    exuart_txBuffer[exuart_currentTxBufferToWriteTo][exuart_txBufferIndex] = data;
    exuart_txBufferIndex++;

    // If the buffer is full, start the DMA transfer.
    if(exuart_txBufferIndex == TX_BUFFER_SIZE)
        exuart_StartDma();
    
    //   
    exuart_FlushTx();
}

/**
 * @brief Asynchronously sends the given bytes through the UART bus.
 * @param data pointer to the data bytes array to send.
 * @param length number of bytes to send (array size).
 */
void exuart_SendBytesAsync(uint8_t *data, int length)
{
    while(length > 0)
    {
        uint16_t nBytesToWriteInBuf;
    
        // Check that it is possible to write in the current buffer.
        if(exuart_txBufferIndex >= TX_BUFFER_SIZE)
        {
            // If the DMA is idle, switch to the other buffer.
            if(UART_DMA_TX_IS_BUSY)
            {
                utils_TrapCpu(); // Error, can't write anywhere!
                return;
            }
            else
                exuart_StartDma();
        }

        // Write as many bytes as possible in the current buffer.
        nBytesToWriteInBuf = TX_BUFFER_SIZE - exuart_txBufferIndex;

        if(nBytesToWriteInBuf > length)
            nBytesToWriteInBuf = length;

        memcpy(&exuart_txBuffer[exuart_currentTxBufferToWriteTo][exuart_txBufferIndex],
               data, nBytesToWriteInBuf);
        exuart_txBufferIndex += nBytesToWriteInBuf;

        data += nBytesToWriteInBuf;
        length -= nBytesToWriteInBuf;

        // If the buffer is full, start the DMA transfer.
        if(exuart_txBufferIndex == TX_BUFFER_SIZE)
            exuart_StartDma();
    }
    
    //
    exuart_FlushTx();
}

/**
 * @brief Start the DMA to send the bytes waiting in the intermediate buffer.
 */
void exuart_FlushTx(void)
{
    if(exuart_txBufferIndex > 0 && !UART_DMA_TX_IS_BUSY)
        exuart_StartDma();
}
