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

#include "uart.h"
#include "../lib/utils.h"
#include "../communication.h" // TODO: REMOVE. DEBUG ONLY.

#define UART_RX_DMA DMA1_Stream5
#define UART_RX_DMA_CHANNEL DMA_Channel_4
#define UART_TX_DMA DMA1_Stream6
#define UART_TX_DMA_CHANNEL DMA_Channel_4

#define UART_TX_BUFFER_SIZE 4096
#define UART_RX_BUFFER_SIZE 512
#define UART_USER_RX_QUEUE_SIZE 512

uint8_t uart_txBuffer[2][UART_TX_BUFFER_SIZE];
uint8_t uart_currentTxBufferToWriteTo;
uint16_t uart_txBufferIndex;
uint16_t uart_nBytesToTransferDma;

uint8_t uart_rxBuffer[UART_RX_BUFFER_SIZE];
uint8_t const * uart_rxBuffTail;

uint8_t uart_userRxQueue[UART_USER_RX_QUEUE_SIZE];
cb_CircularBuffer uart_rxQueue;

#define UART_DMA_TX_IS_BUSY (DMA_GetCmdStatus(UART_TX_DMA) == ENABLE && DMA_GetCurrDataCounter(UART_TX_DMA) > 0)

/**
  * @brief Initializes the UART module.
  */
void uart_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;   
    DMA_InitTypeDef DMA_InitStruct;
    
    // Enable UART and DMA peripherals clocks.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    // Setup GPIOs as UART pins.
    GPIO_InitStruct.GPIO_Pin   = USART_TX_Pin;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(USART_TX_Port, &GPIO_InitStruct);
    GPIO_PinAFConfig(USART_TX_Port, USART_TX_PinSource, GPIO_AF_USART2);
    
    GPIO_InitStruct.GPIO_Pin = USART_RX_Pin;
    GPIO_Init(USART_RX_Port, &GPIO_InitStruct);     
    GPIO_PinAFConfig(USART_RX_Port, USART_RX_PinSource, GPIO_AF_USART2);

    // Setup the UART peripheral.
    USART_InitStruct.USART_BaudRate            = UART_BAUDRATE;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART_PC_COMM, &USART_InitStruct);
    
    USART_Cmd(USART_PC_COMM, ENABLE);
    
    // Setup the DMA for RX.
    DMA_DeInit(UART_RX_DMA);
    
    DMA_InitStruct.DMA_Channel = UART_RX_DMA_CHANNEL;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&uart_rxBuffer[0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = UART_RX_BUFFER_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(UART_RX_DMA, &DMA_InitStruct);

    USART_DMACmd(USART_PC_COMM, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    DMA_ITConfig(UART_RX_DMA, DMA_IT_TC, ENABLE);
    DMA_Cmd(UART_RX_DMA, ENABLE);
    
    uart_rxBuffTail = &uart_rxBuffer[0];
    
    // Setup the DMA for TX.
    DMA_DeInit(UART_TX_DMA);
    
    DMA_InitStruct.DMA_Channel = UART_TX_DMA_CHANNEL;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&uart_txBuffer[0][0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = UART_TX_BUFFER_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(UART_TX_DMA, &DMA_InitStruct);

    USART_DMACmd(USART_PC_COMM, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    DMA_ITConfig(UART_TX_DMA, DMA_IT_TC, ENABLE);
    //DMA_Cmd(UART_TX_DMA, ENABLE);
    
    // Initialize the RX circular buffer.
    cb_Init(&uart_rxQueue, uart_userRxQueue, UART_USER_RX_QUEUE_SIZE);
    
    // Initialize the variables for the UART TX.
    uart_currentTxBufferToWriteTo = 0;
    uart_txBufferIndex = 0;
    uart_nBytesToTransferDma = 0;
}

/**
 * @brief Copies the received bytes into the user-accessible queue.
 * Reads all the available bytes in the DMA RX buffer, and copies them to the
 * user-accessible queue.
 * @remark This function must called often, otherwise the DMA RX buffer may be
 * full.
 */
void uart_Step(void)
{
    // Get the location of the location currently pointed by the DMA.
    uint8_t const * head = uart_rxBuffer + UART_RX_BUFFER_SIZE
                           - DMA_GetCurrDataCounter(UART_RX_DMA);
                           
    // Even if the STM32F4 reference manual (RM0090) states that the NDTR
    // register is decremented after the transfer, this is not the case.
    // So we wait a few cycles to be sure that the DMA actually performed the
    // transfer.
    utils_DelayUs(1);

    // RX: add the received bytes into the user queue.
    while(uart_rxBuffTail != head)
    {
        uint8_t b =  *uart_rxBuffTail;
        cb_Push(&uart_rxQueue, b);

        uart_rxBuffTail++;

        if(uart_rxBuffTail >= uart_rxBuffer + UART_RX_BUFFER_SIZE)
            uart_rxBuffTail -= UART_RX_BUFFER_SIZE;
    }
}

/**
 * @brief Gets the user-accessible queue of the received bytes.
 * @return the address of the queue.
 */
cb_CircularBuffer* uart_GetRxQueue(void)
{
    return &uart_rxQueue;
}

/**
 * @brief Starts the DMA transfer to send bytes to UART peripheral.
 */
void uart_StartDma(void)
{
    DMA_InitTypeDef DMA_InitStruct;
    
    //
    uart_nBytesToTransferDma = uart_txBufferIndex;
    uart_currentTxBufferToWriteTo = !uart_currentTxBufferToWriteTo;
    uart_txBufferIndex = 0;

    // Start the DMA transfer.
    DMA_DeInit(UART_TX_DMA);
    
    DMA_InitStruct.DMA_Channel = UART_TX_DMA_CHANNEL;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&uart_txBuffer[!uart_currentTxBufferToWriteTo][0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = uart_nBytesToTransferDma;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(UART_TX_DMA, &DMA_InitStruct);
                          
    DMA_Cmd(UART_TX_DMA, ENABLE);
}


/**
 * @brief Asynchronously sends the given byte through the UART bus.
 * @param data the data byte to send.
 * @remark The byte may not be sent immediately, they are stored temporarily in
 * an intermediate buffer that the DMA will copy to UART peripheral, when it is
 * full. Call uart_FlushTx() to start the DMA transfer immediately.
 * @warning This function will block until a write buffer is ready.
 */
void uart_SendByteAsync(uint8_t data)
{
    // Check that it is possible to write in the current buffer.
    if(uart_txBufferIndex >= UART_TX_BUFFER_SIZE)
    {
        // Switch to the other buffer when it is ready.
        while(UART_DMA_TX_IS_BUSY)
        {
            // Wait until the DMA is not busy (no ongoing transfer, so the other
            // buffer can be used).
        }

        uart_StartDma();
    }

    // Write the byte in the buffer.
    uart_txBuffer[uart_currentTxBufferToWriteTo][uart_txBufferIndex] = data;
    uart_txBufferIndex++;

    // If the buffer is full, start the DMA transfer.
    if(uart_txBufferIndex == UART_TX_BUFFER_SIZE)
        uart_StartDma();
}

/**
 * @brief Asynchronously sends the given bytes through the UART bus.
 * @param data pointer to the data bytes array to send.
 * @param length number of bytes to send (array size).
 * @remark The bytes may not be sent immediately, they are stored temporarily in
 * an intermediate buffer that the DMA will copy to UART peripheral, when it is
 * full. Call uart_FlushTx() to start the DMA transfer immediately.
 * @warning This function will block until a write buffer is ready, which can be
 * very long.
 */
void uart_SendBytesAsync(uint8_t *data, int length)
{
    while(length > 0)
    {
        uint16_t nBytesToWriteInBuf;
    
        // Check that it is possible to write in the current buffer.
        if(uart_txBufferIndex >= UART_TX_BUFFER_SIZE)
        {
            // Switch to the other buffer when it is ready.
            while(UART_DMA_TX_IS_BUSY)
            {
                // Wait until the DMA is not busy (no ongoing transfer, so the
                // other buffer can be used).
            }

            uart_StartDma();
        }

        // Write as many bytes as possible in the current buffer.
        nBytesToWriteInBuf = UART_TX_BUFFER_SIZE - uart_txBufferIndex;

        if(nBytesToWriteInBuf > length)
            nBytesToWriteInBuf = length;

        memcpy(&uart_txBuffer[uart_currentTxBufferToWriteTo][uart_txBufferIndex],
               data, nBytesToWriteInBuf);
        uart_txBufferIndex += nBytesToWriteInBuf;

        data += nBytesToWriteInBuf;
        length -= nBytesToWriteInBuf;

        // If the buffer is full, start the DMA transfer.
        if(uart_txBufferIndex == UART_TX_BUFFER_SIZE)
            uart_StartDma();
    }
}

/**
 * @brief Start the DMA to send the bytes waiting in the intermediate buffer.
 */
void uart_FlushTx(void)
{
    if(uart_txBufferIndex > 0 && !UART_DMA_TX_IS_BUSY)
        uart_StartDma();
}
