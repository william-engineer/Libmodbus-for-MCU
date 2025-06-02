/*******************************************************************
** Project Name :    Libmodbus for MCU                            **
********************************************************************
** Product Name :                                                 **
**                                                                **
** File Name    :    modbus-rtu-hal.h                             **
**                                                                **
** Version      :    V1.00                                        **
**                                                                **
** Created By   :    william chen                                 **
**                                                                **
** Create Date  :    2025-05-18                                   **
**                                                                **
********************************************************************/
/*
 * Libmodbus for MCU - Version 1.0.0
 * Based on libmodbus 3.1.11
 *
 * Copyright © William Chen <william_engineer@outlook.com>
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * This file is part of Libmodbus for MCU, an adaptation of libmodbus
 * for MCU platforms with hardware abstraction and optimization.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 */
#include "modbus-rtu-hal.h"
#include "./BSP/PCF8574/pcf8574.h"
static int modbus_uart_init(pstUartDevice l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit);
static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char * l_pu8Datas, unsigned int l_u32Len, unsigned int l_u32Timeout);
static int modbus_uart_recv(pstUartDevice l_pstDev, unsigned char * l_pu8Datas, unsigned int l_u32Len, unsigned int l_u32Timeout);
static int modbus_uart_flush(pstUartDevice l_pstDev);
static void modbus_rs485_dir_switch(pstUartDevice l_pstDev, unsigned char l_u8TxDir);


/* Max between RTU and TCP max adu length */
#define MODBUS_RECV_BUFF_SIZE 300
static unsigned char sArM_u8ModbusSlaveRecvBuff[MODBUS_RECV_BUFF_SIZE];
static stUartDevice s_stSlaveDev = {
    .m_pi8Name = "uart2",
    .m_enDevId = UART2,
    .m_pu8RecvBuff = sArM_u8ModbusSlaveRecvBuff,
    .m_u16RecvBuffSize = MODBUS_RECV_BUFF_SIZE,
    .m_u16RecvLen = 0,
    .m_u8RecvDone = 0,
    .m_pvSendSem=NULL,
    .m_pvRecvSem=NULL,
    .m_pFnInit = modbus_uart_init,
    .m_pFnSend = modbus_uart_send,
    .m_pFnRecv = modbus_uart_recv,
    .m_pFnFlush = modbus_uart_flush,
    .m_pFnDirSwitch = modbus_rs485_dir_switch
};

static unsigned char sArM_u8ModbusMasterRecvBuff[MODBUS_RECV_BUFF_SIZE];
static stUartDevice s_stMasterDev = {
    .m_pi8Name = "uart3",
    .m_enDevId = UART3,
    .m_pu8RecvBuff = sArM_u8ModbusMasterRecvBuff,
    .m_u16RecvBuffSize = MODBUS_RECV_BUFF_SIZE,
    .m_u16RecvLen = 0,
    .m_u8RecvDone = 0,
    .m_pvSendSem=NULL,
    .m_pvRecvSem=NULL,
    .m_pFnInit = modbus_uart_init,
    .m_pFnSend = modbus_uart_send,
    .m_pFnRecv = modbus_uart_recv,
    .m_pFnFlush = modbus_uart_flush,
    .m_pFnDirSwitch = modbus_rs485_dir_switch
};

/* device management base on device id */
static pstUartDevice sArM_pstUartDevs[MAX_UART_DEV_ID] = {
  NULL,          //UART1
  &s_stSlaveDev, //UART2
  &s_stMasterDev //UART3
};

/**
 * @brief  find the UART device by name
 * @param  l_pi8Name: Name of the UART device
 * @retval Pointer to the UART device structure if found, NULL otherwise
 */
pstUartDevice find_uart_device(char *l_pi8Name)
{
  for (unsigned char l_u8Index = 0; l_u8Index < MAX_UART_DEV_ID; l_u8Index++)
  {
    if (sArM_pstUartDevs[l_u8Index] != NULL)
    {
      if (strcmp(sArM_pstUartDevs[l_u8Index]->m_pi8Name, l_pi8Name) == 0)
      {
        return sArM_pstUartDevs[l_u8Index];
      }
    }
  }
  return NULL;
}

/**
 * @brief  Initialize the UART device
 * @param  l_pstDev: Pointer to the UART device structure
 * @param  l_i32Baud: Baud rate for the UART communication
 * @param  l_i8Parity: Parity setting ('N' for none, 'E' for even, 'O' for odd)
 * @param  l_i32DataBit: Number of data bits (7 or 8)
 * @param  l_i32StopBit: Number of stop bits (1 or 2)
 * @retval 0 on success, -1 on failure
 */
static int modbus_uart_init(pstUartDevice l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit)
{
  GPIO_InitTypeDef l_stGpioInit;

  switch (l_pstDev->m_enDevId) {
    case UART2:
      /* enable peripheral clocks for UART2, GPIO and DMA1 */
      __HAL_RCC_GPIOD_CLK_ENABLE(); 
      __HAL_RCC_USART2_CLK_ENABLE(); 
      __HAL_RCC_DMA1_CLK_ENABLE();

      /* configure PD5, PD6 as alternate function for USART2 TX/RX */
      l_stGpioInit.Pin = GPIO_PIN_5 | GPIO_PIN_6; 
      l_stGpioInit.Mode = GPIO_MODE_AF_PP; 
      l_stGpioInit.Pull = GPIO_PULLUP;
      l_stGpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH; 
      l_stGpioInit.Alternate = GPIO_AF7_USART2; 
      HAL_GPIO_Init(GPIOD, &l_stGpioInit); 

      /* configure USART2 */
      l_pstDev->m_stHalHandle.Instance = USART2;
      l_pstDev->m_stHalHandle.Init.BaudRate = l_i32Baud;
      l_pstDev->m_stHalHandle.Init.WordLength = (l_i32DataBit == 8) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
      l_pstDev->m_stHalHandle.Init.StopBits = (l_i32StopBit == 1) ? UART_STOPBITS_1 : UART_STOPBITS_2;
      l_pstDev->m_stHalHandle.Init.Parity = (l_i8Parity == 'E') ? UART_PARITY_EVEN : ((l_i8Parity == 'O') ? UART_PARITY_ODD : UART_PARITY_NONE);
      l_pstDev->m_stHalHandle.Init.Mode = UART_MODE_TX_RX;
      l_pstDev->m_stHalHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      l_pstDev->m_stHalHandle.Init.OverSampling = UART_OVERSAMPLING_16;
      HAL_UART_Init(&l_pstDev->m_stHalHandle);

      /* configure interrupt */
      HAL_NVIC_EnableIRQ(USART2_IRQn);
      HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
      __HAL_UART_ENABLE_IT(&(l_pstDev->m_stHalHandle), UART_IT_IDLE);
      __HAL_UART_CLEAR_FLAG(&(l_pstDev->m_stHalHandle), UART_FLAG_TC);

      /* configure DMA1 Stream6, Channel4 for USART2 TX (Memory to Peripheral) */
      l_pstDev->m_stTxDmaHandle.Instance = DMA1_Stream6;
      l_pstDev->m_stTxDmaHandle.Init.Channel = DMA_CHANNEL_4;
      l_pstDev->m_stTxDmaHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
      l_pstDev->m_stTxDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE; // peripheral address not incremented
      l_pstDev->m_stTxDmaHandle.Init.MemInc = DMA_MINC_ENABLE; // memory address incremented
      l_pstDev->m_stTxDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      l_pstDev->m_stTxDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      l_pstDev->m_stTxDmaHandle.Init.Mode = DMA_NORMAL; // DMA mode: normal
      l_pstDev->m_stTxDmaHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
      l_pstDev->m_stTxDmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE; // FIFO mode disabled
      l_pstDev->m_stTxDmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
      l_pstDev->m_stTxDmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
      l_pstDev->m_stTxDmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
      HAL_DMA_DeInit(&l_pstDev->m_stTxDmaHandle);
      HAL_DMA_Init(&l_pstDev->m_stTxDmaHandle);

      /* configure DMA1 Stream5, Channel4 for USART2 RX (Peripheral to Memory) */
      l_pstDev->m_stRxDmaHandle.Instance = DMA1_Stream5;
      l_pstDev->m_stRxDmaHandle.Init.Channel = DMA_CHANNEL_4;
      l_pstDev->m_stRxDmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
      l_pstDev->m_stRxDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE; // peripheral address not incremented
      l_pstDev->m_stRxDmaHandle.Init.MemInc = DMA_MINC_ENABLE; // memory address incremented
      l_pstDev->m_stRxDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      l_pstDev->m_stRxDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      l_pstDev->m_stRxDmaHandle.Init.Mode = DMA_CIRCULAR; // DMA mode: circular
      l_pstDev->m_stRxDmaHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
      l_pstDev->m_stRxDmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE; // FIFO mode disabled
      l_pstDev->m_stRxDmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
      l_pstDev->m_stRxDmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
      l_pstDev->m_stRxDmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
      HAL_DMA_DeInit(&l_pstDev->m_stRxDmaHandle);
      HAL_DMA_Init(&l_pstDev->m_stRxDmaHandle);

      /* Associate the DMA handle */
      __HAL_LINKDMA(&l_pstDev->m_stHalHandle, hdmatx, l_pstDev->m_stTxDmaHandle);
      __HAL_LINKDMA(&l_pstDev->m_stHalHandle, hdmarx, l_pstDev->m_stRxDmaHandle);

      /* NVIC configuration for DMA uart TX */
      HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

      /* NVIC configuration for DMA uart RX */
      HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

      /* Enable DMA for USART2 RX */
      l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
      HAL_UART_Receive_DMA(&l_pstDev->m_stHalHandle, l_pstDev->m_pu8RecvBuff, MODBUS_RECV_BUFF_SIZE);

      /* Create semaphores for send and receive */
      l_pstDev->m_pvSendSem = xSemaphoreCreateBinary();
      if (l_pstDev->m_pvSendSem != NULL)
      {
        xSemaphoreGive((pstModbusSem_t)(l_pstDev->m_pvSendSem));
      }
      l_pstDev->m_pvRecvSem = xSemaphoreCreateBinary();
      break;
    case UART3:
      /* enable peripheral clocks for UART3, GPIO and DMA1 */
      __HAL_RCC_GPIOD_CLK_ENABLE();
      __HAL_RCC_USART3_CLK_ENABLE();
      __HAL_RCC_DMA1_CLK_ENABLE();

      /* configure PD8, PD9 as alternate function for USART3 TX/RX */
      l_stGpioInit.Pin = GPIO_PIN_8 | GPIO_PIN_9; 
      l_stGpioInit.Mode = GPIO_MODE_AF_PP;
      l_stGpioInit.Pull = GPIO_PULLUP;
      l_stGpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      l_stGpioInit.Alternate = GPIO_AF7_USART3;
      HAL_GPIO_Init(GPIOD, &l_stGpioInit);

      /* configure USART3 */
      l_pstDev->m_stHalHandle.Instance = USART3;
      l_pstDev->m_stHalHandle.Init.BaudRate = l_i32Baud;
      l_pstDev->m_stHalHandle.Init.WordLength = (l_i32DataBit == 8) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
      l_pstDev->m_stHalHandle.Init.StopBits = (l_i32StopBit == 1) ? UART_STOPBITS_1 : UART_STOPBITS_2;
      l_pstDev->m_stHalHandle.Init.Parity = (l_i8Parity == 'E') ? UART_PARITY_EVEN : ((l_i8Parity == 'O') ? UART_PARITY_ODD : UART_PARITY_NONE);
      l_pstDev->m_stHalHandle.Init.Mode = UART_MODE_TX_RX;
      l_pstDev->m_stHalHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      l_pstDev->m_stHalHandle.Init.OverSampling = UART_OVERSAMPLING_16;
      HAL_UART_Init(&l_pstDev->m_stHalHandle);

      /* configure interrupt */
      HAL_NVIC_EnableIRQ(USART3_IRQn);
      HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
      __HAL_UART_ENABLE_IT(&(l_pstDev->m_stHalHandle), UART_IT_IDLE);
      __HAL_UART_CLEAR_FLAG(&(l_pstDev->m_stHalHandle), UART_FLAG_TC);

      /* configure DMA1 Stream3, Channel4 for USART3 TX (Memory to Peripheral) */
      l_pstDev->m_stTxDmaHandle.Instance = DMA1_Stream3;
      l_pstDev->m_stTxDmaHandle.Init.Channel = DMA_CHANNEL_4;
      l_pstDev->m_stTxDmaHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
      l_pstDev->m_stTxDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
      l_pstDev->m_stTxDmaHandle.Init.MemInc = DMA_MINC_ENABLE;
      l_pstDev->m_stTxDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      l_pstDev->m_stTxDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      l_pstDev->m_stTxDmaHandle.Init.Mode = DMA_NORMAL;
      l_pstDev->m_stTxDmaHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
      l_pstDev->m_stTxDmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
      l_pstDev->m_stTxDmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
      l_pstDev->m_stTxDmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
      l_pstDev->m_stTxDmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
      HAL_DMA_DeInit(&l_pstDev->m_stTxDmaHandle);
      HAL_DMA_Init(&l_pstDev->m_stTxDmaHandle);

      /* configure DMA1 Stream1, Channel4 for USART3 RX (Peripheral to Memory) */
      l_pstDev->m_stRxDmaHandle.Instance = DMA1_Stream1;
      l_pstDev->m_stRxDmaHandle.Init.Channel = DMA_CHANNEL_4;
      l_pstDev->m_stRxDmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
      l_pstDev->m_stRxDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
      l_pstDev->m_stRxDmaHandle.Init.MemInc = DMA_MINC_ENABLE;
      l_pstDev->m_stRxDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      l_pstDev->m_stRxDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
      l_pstDev->m_stRxDmaHandle.Init.Mode = DMA_CIRCULAR;
      l_pstDev->m_stRxDmaHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
      l_pstDev->m_stRxDmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
      l_pstDev->m_stRxDmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
      l_pstDev->m_stRxDmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
      l_pstDev->m_stRxDmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
      HAL_DMA_DeInit(&l_pstDev->m_stRxDmaHandle);
      HAL_DMA_Init(&l_pstDev->m_stRxDmaHandle);

      /* Associate the DMA handle */
      __HAL_LINKDMA(&l_pstDev->m_stHalHandle, hdmatx, l_pstDev->m_stTxDmaHandle);
      __HAL_LINKDMA(&l_pstDev->m_stHalHandle, hdmarx, l_pstDev->m_stRxDmaHandle);

      /* NVIC configuration for DMA uart TX */
      HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

      /* NVIC configuration for DMA uart RX */
      HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

      /* Enable DMA for USART3 RX */
      l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
      HAL_UART_Receive_DMA(&l_pstDev->m_stHalHandle, l_pstDev->m_pu8RecvBuff, MODBUS_RECV_BUFF_SIZE);

      /* Create semaphores for send and receive */
      l_pstDev->m_pvSendSem = xSemaphoreCreateBinary();
      if (l_pstDev->m_pvSendSem != NULL)
      {
        xSemaphoreGive((pstModbusSem_t)(l_pstDev->m_pvSendSem));
      }
      l_pstDev->m_pvRecvSem = xSemaphoreCreateBinary();
    break;
    default:
      break;
  }

	return 0;
}

/**
 * @brief  Send data via UART
 * @param  l_pstDev: Pointer to the UART device structure
 * @param  l_pu8Datas: Pointer to the data buffer to be sent
 * @param  l_u32Len: Length of the data to be sent
 * @param  l_u32Timeout: Timeout for sending data
 * @retval Number of bytes sent on success, -1 on failure
 */
static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char * l_pu8Datas, unsigned int l_u32Len, unsigned int l_u32Timeout)
{
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_TX);
  if (xSemaphoreTake((pstModbusSem_t)(l_pstDev->m_pvSendSem), l_u32Timeout) == pdTRUE) {
    /* get semaphore successfully, then transmit data */
    HAL_UART_Transmit_DMA(&l_pstDev->m_stHalHandle, l_pu8Datas, l_u32Len);
  }
  else
  {
    /* get semaphore failed */
    return -1;
  }
  return l_u32Len;
}

/**
 * @brief  Receive data via UART
 * @param  l_pstDev: Pointer to the UART device structure
 * @param  l_pu8Datas: Pointer to the buffer to store received data
 * @param  l_u32Len: Length of the data to be read
 * @param  l_u32Timeout: Timeout for receiving data
 * @retval Number of bytes received on success, -1 on failure
 */
static int modbus_uart_recv(pstUartDevice l_pstDev, unsigned char * l_pu8Datas, unsigned int l_u32Len, unsigned int l_u32Timeout)
{
  if (l_pstDev->m_u8RecvDone)
  {
    /* data has been received, copy data to user buffer */
    memcpy(l_pu8Datas, l_pstDev->m_pu8RecvBuff + l_pstDev->m_u16ReadLen, l_u32Len);
    l_pstDev->m_u16ReadLen += l_u32Len;
    if (l_pstDev->m_u16ReadLen >= l_pstDev->m_u16RecvLen)
    {
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u8RecvDone = 0;
      HAL_UART_Receive_DMA(&(l_pstDev->m_stHalHandle), l_pstDev->m_pu8RecvBuff,MODBUS_RECV_BUFF_SIZE);
    }
		return l_u32Len;
  }
  else
  {
    if (xSemaphoreTake((pstModbusSem_t)(l_pstDev->m_pvRecvSem), l_u32Timeout) == pdTRUE)
    {
      /* get semaphore successfully, data has been received */
      l_pstDev->m_u8RecvDone = 1;
      l_pstDev->m_u16ReadLen = 0;
      return 0;
    }
    else
    {
      /* get semaphore failed, flush the receive buffer */
      l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      HAL_UART_DMAStop(&(l_pstDev->m_stHalHandle));
      HAL_UART_Receive_DMA(&(l_pstDev->m_stHalHandle), l_pstDev->m_pu8RecvBuff,MODBUS_RECV_BUFF_SIZE);
      return -1;
    }
  }
}

/**
 * @brief  Flush the UART receive buffer
 * @param  l_pstDev: Pointer to the UART device structure
 * @retval 0 on success, -1 on failure
 */
static int modbus_uart_flush(pstUartDevice l_pstDev)
{
  memset(l_pstDev->m_pu8RecvBuff, 0, l_pstDev->m_u16RecvBuffSize);
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  HAL_UART_DMAStop(&(l_pstDev->m_stHalHandle));
  HAL_UART_Receive_DMA(&(l_pstDev->m_stHalHandle), l_pstDev->m_pu8RecvBuff,MODBUS_RECV_BUFF_SIZE);
  return 0;
}

/**
 * @brief  Switch the RS485 direction
 * @param  l_pstDev: Pointer to the UART device structure
 * @param  l_enDir: Direction to switch to (MODBUS_DIR_RX or MODBUS_DIR_TX)
 * @retval None
 */
static void modbus_rs485_dir_switch(pstUartDevice l_pstDev, enModbusDir_t l_enDir)
{
  switch (l_pstDev->m_enDevId)
  {
    case UART2:
      pcf8574_write_bit(RS485_RE_IO, l_enDir);
      break;
    default:
      break;
  }
}

void USART2_IRQHandler(void)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;

  if (sArM_pstUartDevs[UART2] == NULL)
    return;

  HAL_UART_IRQHandler(&(sArM_pstUartDevs[UART2]->m_stHalHandle));

  /* check if IDLE flag is set */
  if (((sArM_pstUartDevs[UART2]->m_stHalHandle.Instance->SR & USART_SR_IDLE) != RESET))
  {
    int l_i32Dummy = 0;
    /* clear idle flag with an read to the USART_SR register followed by a read to the USART_DR register */
    l_i32Dummy = sArM_pstUartDevs[UART2]->m_stHalHandle.Instance->SR;
    l_i32Dummy = sArM_pstUartDevs[UART2]->m_stHalHandle.Instance->DR;

    /* calculate the number of bytes received */
    unsigned short remaining = __HAL_DMA_GET_COUNTER(&(sArM_pstUartDevs[UART2]->m_stRxDmaHandle));
    sArM_pstUartDevs[UART2]->m_u16RecvLen = MODBUS_RECV_BUFF_SIZE - remaining;

    /* to avoid empty IDLE trigger */
    if (sArM_pstUartDevs[UART2]->m_u16RecvLen > 0)
    {
      /* stop DMA reception */
      HAL_UART_DMAStop(&(sArM_pstUartDevs[UART2]->m_stHalHandle));

      /* Was the message posted successfully? */
      if (xSemaphoreGiveFromISR((pstModbusSem_t)(sArM_pstUartDevs[UART2]->m_pvRecvSem), &l_xHigherPriorityTaskWoken) != pdFAIL)
      {
        /* If l_xHigherPriorityTaskWoken is now set to pdTRUE then a context
        switch should be requested. */
        portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
      }
    }
  }
}

//tx dma interrupt
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&(sArM_pstUartDevs[UART2]->m_stTxDmaHandle));
}
//rx dma interrupt
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&(sArM_pstUartDevs[UART2]->m_stRxDmaHandle));
}



void USART3_IRQHandler(void)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;

  if (sArM_pstUartDevs[UART3] == NULL)
    return;

  HAL_UART_IRQHandler(&(sArM_pstUartDevs[UART3]->m_stHalHandle));

  /* check if IDLE flag is set */
  if (((sArM_pstUartDevs[UART3]->m_stHalHandle.Instance->SR & USART_SR_IDLE) != RESET))
  {
    int l_i32Dummy = 0;
    /* clear idle flag with an read to the USART_SR register followed by a read to the USART_DR register */
    l_i32Dummy = sArM_pstUartDevs[UART3]->m_stHalHandle.Instance->SR;
    l_i32Dummy = sArM_pstUartDevs[UART3]->m_stHalHandle.Instance->DR;

    /* calculate the number of bytes received */
    unsigned short remaining = __HAL_DMA_GET_COUNTER(&(sArM_pstUartDevs[UART3]->m_stRxDmaHandle));
    sArM_pstUartDevs[UART3]->m_u16RecvLen = MODBUS_RECV_BUFF_SIZE - remaining;

    /* to avoid empty IDLE trigger */
    if (sArM_pstUartDevs[UART3]->m_u16RecvLen > 0)
    {
      /* stop DMA reception */
      HAL_UART_DMAStop(&(sArM_pstUartDevs[UART3]->m_stHalHandle));

      /* Was the message posted successfully? */
      if (xSemaphoreGiveFromISR((pstModbusSem_t)(sArM_pstUartDevs[UART3]->m_pvRecvSem), &l_xHigherPriorityTaskWoken) != pdFAIL)
      {
        /* If l_xHigherPriorityTaskWoken is now set to pdTRUE then a context
        switch should be requested. */
        portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
      }
    }
  }
}

//tx dma interrupt
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&(sArM_pstUartDevs[UART3]->m_stTxDmaHandle));
}
//rx dma interrupt
void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&(sArM_pstUartDevs[UART3]->m_stRxDmaHandle));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;

  if (huart == &(sArM_pstUartDevs[UART2]->m_stHalHandle))
  {//check if the UART handle is for UART2

    /* Disable IDLE interrupt to prevent immediate trigger after DMA transfer completion */
    __HAL_UART_DISABLE_IT(&(sArM_pstUartDevs[UART2]->m_stHalHandle), UART_IT_IDLE);
    HAL_UART_DMAStop(&(sArM_pstUartDevs[UART2]->m_stHalHandle));

    /* Calculate the number of bytes received */
    sArM_pstUartDevs[UART2]->m_u16RecvLen = MODBUS_RECV_BUFF_SIZE;

    /* Enable IDLE interrupt */
    __HAL_UART_ENABLE_IT(&(sArM_pstUartDevs[UART2]->m_stHalHandle), UART_IT_IDLE);

    /* Was the message posted successfully? */
    if (xSemaphoreGiveFromISR((pstModbusSem_t)(sArM_pstUartDevs[UART2]->m_pvRecvSem), &l_xHigherPriorityTaskWoken) != pdFAIL)
    {
      /* If l_xHigherPriorityTaskWoken is now set to pdTRUE then a context
      switch should be requested. */
      portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
    }
  }
	else if (huart == &(sArM_pstUartDevs[UART3]->m_stHalHandle))
  { //check if the UART handle is for UART3  

     /* Disable IDLE interrupt to prevent immediate trigger after DMA transfer completion */
    __HAL_UART_DISABLE_IT(&(sArM_pstUartDevs[UART3]->m_stHalHandle), UART_IT_IDLE);
    HAL_UART_DMAStop(&(sArM_pstUartDevs[UART3]->m_stHalHandle)); 

    /* Calculate the number of bytes received */
    sArM_pstUartDevs[UART3]->m_u16RecvLen = MODBUS_RECV_BUFF_SIZE;
 
    /* Enable IDLE interrupt */
    __HAL_UART_ENABLE_IT(&(sArM_pstUartDevs[UART3]->m_stHalHandle), UART_IT_IDLE);

    /* Was the message posted successfully? */
    if (xSemaphoreGiveFromISR((pstModbusSem_t)(sArM_pstUartDevs[UART3]->m_pvRecvSem), &l_xHigherPriorityTaskWoken) != pdFAIL)
    {
      /* If l_xHigherPriorityTaskWoken is now set to pdTRUE then a context
      switch should be requested. */
      portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  if (huart == &(sArM_pstUartDevs[UART2]->m_stHalHandle))
  { // check if the UART handle is for UART2

    /* Clear transmission complete flag and switch direction */
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
    sArM_pstUartDevs[UART2]->m_pFnDirSwitch(sArM_pstUartDevs[UART2], MODBUS_DIR_RX);

    /* Was the message posted successfully? */
    if (xSemaphoreGiveFromISR((pstModbusSem_t)(sArM_pstUartDevs[UART2]->m_pvSendSem), &l_xHigherPriorityTaskWoken) != pdFAIL)
    {
      /* If l_xHigherPriorityTaskWoken is now set to pdTRUE then a context
      switch should be requested. */
      portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
    }
  }
	else if (huart == &(sArM_pstUartDevs[UART3]->m_stHalHandle))
  { // check if the UART handle is for UART3
		
    /* Clear transmission complete flag and switch direction */
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
	  sArM_pstUartDevs[UART3]->m_pFnDirSwitch(sArM_pstUartDevs[UART3], MODBUS_DIR_RX);

    /* Was the message posted successfully? */
    if (xSemaphoreGiveFromISR((pstModbusSem_t)(sArM_pstUartDevs[UART3]->m_pvSendSem), &l_xHigherPriorityTaskWoken) != pdFAIL)
    {
      /* If l_xHigherPriorityTaskWoken is now set to pdTRUE then a context
      switch should be requested. */
      portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
    }
  }
}



