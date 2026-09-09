/*******************************************************************
** Project Name :    Libmodbus for MCU                            **
********************************************************************
** Product Name :                                                 **
**                                                                **
** File Name    :    modbus-rtu-hal.h                             **
**                                                                **
** Version      :    V1.1.0                                       **
**                                                                **
** Created By   :    william chen                                 **
**                                                                **
** Create Date  :    2025-05-18                                   **
**                                                                **
********************************************************************/
/*
 * Libmodbus for MCU - Version 1.1.0
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
#include "stm32f4xx_hal.h"

#include "../../modbus-rtu-hal.h"
#include "./BSP/PCF8574/pcf8574.h"

/* STM32F429 board port: USART2/USART3 + DMA1 + PCF8574 RS485 direction. */
static int modbus_uart_init(pstUartDevice l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit);
static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout);
static int modbus_uart_recv(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout);
static int modbus_uart_flush(pstUartDevice l_pstDev);
static void modbus_rs485_dir_switch(pstUartDevice l_pstDev, enModbusDir_t l_enDir);
static pstUartDevice find_uart_device_by_handle(UART_HandleTypeDef *l_pstHalHandle);
static int modbus_uart_rx_dma_restart(pstUartDevice l_pstDev);
static int modbus_uart_rx_dma_stop(pstUartDevice l_pstDev);
static void modbus_uart_rx_dma_stop_from_isr(pstUartDevice l_pstDev);
static void modbus_uart_tx_abort(pstUartDevice l_pstDev);
static void modbus_uart_wait_rtu_guard(pstUartDevice l_pstDev);
static TickType_t modbus_uart_interval_ticks(uint32_t l_u32IntervalUs);
static TickType_t modbus_uart_tx_timeout_ticks(pstUartDevice l_pstDev, unsigned int l_u32Len,
                                               unsigned int l_u32TimeoutMs);
static void modbus_uart_rx_idle_isr(pstUartDevice l_pstDev, BaseType_t *l_pxHigherPriorityTaskWoken);

typedef struct
{
  UART_HandleTypeDef m_stHalHandle;
  DMA_HandleTypeDef m_stTxDmaHandle;
  DMA_HandleTypeDef m_stRxDmaHandle;
} stUartContext;

#define UART_CONTEXT(dev)  ((stUartContext *)((dev)->m_pvPortContext))
#define UART_HANDLE(dev)   (UART_CONTEXT(dev)->m_stHalHandle)
#define TX_DMA_HANDLE(dev) (UART_CONTEXT(dev)->m_stTxDmaHandle)
#define RX_DMA_HANDLE(dev) (UART_CONTEXT(dev)->m_stRxDmaHandle)

/* Max between RTU and TCP max adu length */
#define MODBUS_RECV_BUFF_SIZE 300
static stUartContext s_stUart2Context;
static unsigned char sArM_u8ModbusSlaveRecvBuff[MODBUS_RECV_BUFF_SIZE];
static stUartDevice s_stSlaveDev = {.m_pi8Name = "uart2",
                                    .m_enDevId = MODBUS_UART2,
                                    .m_pu8RecvBuff = sArM_u8ModbusSlaveRecvBuff,
                                    .m_u16RecvBuffSize = MODBUS_RECV_BUFF_SIZE,
                                    .m_u16RecvLen = 0,
                                    .m_u16ReadLen = 0,
                                    .m_u8RecvDone = 0,
                                    .m_u8Initialized = 0,
                                    .m_u32Baud = 0,
                                    .m_u8BitsPerChar = 0,
                                    .m_u8TxBusy = 0,
                                    .m_u8TxError = 0,
                                    .m_u32TxCompleteCount = 0,
                                    .m_u32TxTimeoutCount = 0,
                                    .m_u32TxDmaErrorCount = 0,
                                    .m_u32RxToTxRemainTicks = 0,
                                    .m_u32FullFrameGapTicks = 0,
                                    .m_u32NextTxAllowedTick = 0,
                                    .m_u8TxGapLocked = 0,
                                    .m_pvSendSem = NULL,
                                    .m_pvRecvSem = NULL,
                                    .m_pvPortContext = &s_stUart2Context,
                                    .m_pFnInit = modbus_uart_init,
                                    .m_pFnSend = modbus_uart_send,
                                    .m_pFnRecv = modbus_uart_recv,
                                    .m_pFnFlush = modbus_uart_flush,
                                    .m_pFnDirSwitch = modbus_rs485_dir_switch};

static stUartContext s_stUart3Context;
static unsigned char sArM_u8ModbusMasterRecvBuff[MODBUS_RECV_BUFF_SIZE];
static stUartDevice s_stMasterDev = {.m_pi8Name = "uart3",
                                     .m_enDevId = MODBUS_UART3,
                                     .m_pu8RecvBuff = sArM_u8ModbusMasterRecvBuff,
                                     .m_u16RecvBuffSize = MODBUS_RECV_BUFF_SIZE,
                                     .m_u16RecvLen = 0,
                                     .m_u16ReadLen = 0,
                                     .m_u8RecvDone = 0,
                                     .m_u8Initialized = 0,
                                     .m_u32Baud = 0,
                                     .m_u8BitsPerChar = 0,
                                     .m_u8TxBusy = 0,
                                     .m_u8TxError = 0,
                                     .m_u32TxCompleteCount = 0,
                                     .m_u32TxTimeoutCount = 0,
                                     .m_u32TxDmaErrorCount = 0,
                                     .m_u32RxToTxRemainTicks = 0,
                                     .m_u32FullFrameGapTicks = 0,
                                     .m_u32NextTxAllowedTick = 0,
                                     .m_u8TxGapLocked = 0,
                                     .m_pvSendSem = NULL,
                                     .m_pvRecvSem = NULL,
                                     .m_pvPortContext = &s_stUart3Context,
                                     .m_pFnInit = modbus_uart_init,
                                     .m_pFnSend = modbus_uart_send,
                                     .m_pFnRecv = modbus_uart_recv,
                                     .m_pFnFlush = modbus_uart_flush,
                                     .m_pFnDirSwitch = modbus_rs485_dir_switch};

/* device management base on device id */
static pstUartDevice sArM_pstUartDevs[MODBUS_UART_COUNT] = {NULL, &s_stSlaveDev, &s_stMasterDev};

/**
 * @brief  find the UART device by name
 * @param  l_pi8Name: Name of the UART device
 * @retval Pointer to the UART device structure if found, NULL otherwise
 */
pstUartDevice find_uart_device(const char *l_pi8Name)
{
  if (l_pi8Name == NULL)
  {
    return NULL;
  }

  for (unsigned char l_u8Index = 0; l_u8Index < MODBUS_UART_COUNT; l_u8Index++)
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
  uint32_t l_u32CharUs;
  uint32_t l_u32FullGuardUs;
  int l_i32InitErrno = EIO;

  if ((l_pstDev == NULL) || (l_i32Baud <= 0) || ((l_i8Parity != 'N') && (l_i8Parity != 'E') && (l_i8Parity != 'O')) ||
      ((l_i32DataBit != 7) && (l_i32DataBit != 8)) || ((l_i32DataBit == 7) && (l_i8Parity == 'N')) ||
      ((l_i32StopBit != 1) && (l_i32StopBit != 2)))
  {
    errno = EINVAL;
    return -1;
  }
  if (l_pstDev->m_pvPortContext == NULL)
  {
    errno = ENODEV;
    return -1;
  }

  l_pstDev->m_u8Initialized = 0;

  switch (l_pstDev->m_enDevId)
  {
    case MODBUS_UART2:
      /* enable peripheral clocks for USART2, GPIO and DMA1 */
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

      UART_HANDLE(l_pstDev).Instance = USART2;
      TX_DMA_HANDLE(l_pstDev).Instance = DMA1_Stream6;
      RX_DMA_HANDLE(l_pstDev).Instance = DMA1_Stream5;
      break;

    case MODBUS_UART3:
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

      UART_HANDLE(l_pstDev).Instance = USART3;
      TX_DMA_HANDLE(l_pstDev).Instance = DMA1_Stream3;
      RX_DMA_HANDLE(l_pstDev).Instance = DMA1_Stream1;
      break;

    default:
      errno = ENODEV;
      return -1;
  }

  UART_HANDLE(l_pstDev).Init.BaudRate = (uint32_t)l_i32Baud;
  UART_HANDLE(l_pstDev).Init.WordLength =
      ((l_i32DataBit + ((l_i8Parity == 'N') ? 0 : 1)) > 8) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
  UART_HANDLE(l_pstDev).Init.StopBits = (l_i32StopBit == 1) ? UART_STOPBITS_1 : UART_STOPBITS_2;
  UART_HANDLE(l_pstDev).Init.Parity =
      (l_i8Parity == 'E') ? UART_PARITY_EVEN : ((l_i8Parity == 'O') ? UART_PARITY_ODD : UART_PARITY_NONE);
  UART_HANDLE(l_pstDev).Init.Mode = UART_MODE_TX_RX;
  UART_HANDLE(l_pstDev).Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART_HANDLE(l_pstDev).Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&UART_HANDLE(l_pstDev)) != HAL_OK)
  {
    goto init_failed;
  }

  TX_DMA_HANDLE(l_pstDev).Init.Channel = DMA_CHANNEL_4;
  TX_DMA_HANDLE(l_pstDev).Init.Direction = DMA_MEMORY_TO_PERIPH;
  TX_DMA_HANDLE(l_pstDev).Init.PeriphInc = DMA_PINC_DISABLE;
  TX_DMA_HANDLE(l_pstDev).Init.MemInc = DMA_MINC_ENABLE;
  TX_DMA_HANDLE(l_pstDev).Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  TX_DMA_HANDLE(l_pstDev).Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  TX_DMA_HANDLE(l_pstDev).Init.Mode = DMA_NORMAL;
  TX_DMA_HANDLE(l_pstDev).Init.Priority = DMA_PRIORITY_HIGH;
  TX_DMA_HANDLE(l_pstDev).Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  TX_DMA_HANDLE(l_pstDev).Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  TX_DMA_HANDLE(l_pstDev).Init.MemBurst = DMA_MBURST_SINGLE;
  TX_DMA_HANDLE(l_pstDev).Init.PeriphBurst = DMA_PBURST_SINGLE;
  (void)HAL_DMA_DeInit(&TX_DMA_HANDLE(l_pstDev));
  if (HAL_DMA_Init(&TX_DMA_HANDLE(l_pstDev)) != HAL_OK)
  {
    goto init_failed;
  }

  RX_DMA_HANDLE(l_pstDev).Init.Channel = DMA_CHANNEL_4;
  RX_DMA_HANDLE(l_pstDev).Init.Direction = DMA_PERIPH_TO_MEMORY;
  RX_DMA_HANDLE(l_pstDev).Init.PeriphInc = DMA_PINC_DISABLE;
  RX_DMA_HANDLE(l_pstDev).Init.MemInc = DMA_MINC_ENABLE;
  RX_DMA_HANDLE(l_pstDev).Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  RX_DMA_HANDLE(l_pstDev).Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  RX_DMA_HANDLE(l_pstDev).Init.Mode = DMA_NORMAL;
  RX_DMA_HANDLE(l_pstDev).Init.Priority = DMA_PRIORITY_MEDIUM;
  RX_DMA_HANDLE(l_pstDev).Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  RX_DMA_HANDLE(l_pstDev).Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  RX_DMA_HANDLE(l_pstDev).Init.MemBurst = DMA_MBURST_SINGLE;
  RX_DMA_HANDLE(l_pstDev).Init.PeriphBurst = DMA_PBURST_SINGLE;
  (void)HAL_DMA_DeInit(&RX_DMA_HANDLE(l_pstDev));
  if (HAL_DMA_Init(&RX_DMA_HANDLE(l_pstDev)) != HAL_OK)
  {
    goto init_failed;
  }

  __HAL_LINKDMA(&UART_HANDLE(l_pstDev), hdmatx, TX_DMA_HANDLE(l_pstDev));
  __HAL_LINKDMA(&UART_HANDLE(l_pstDev), hdmarx, RX_DMA_HANDLE(l_pstDev));

  if (l_pstDev->m_enDevId == MODBUS_UART2)
  {
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
  else
  {
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }

  __HAL_UART_DISABLE_IT(&UART_HANDLE(l_pstDev), UART_IT_IDLE);
  __HAL_UART_CLEAR_IDLEFLAG(&UART_HANDLE(l_pstDev));
  __HAL_UART_CLEAR_FLAG(&UART_HANDLE(l_pstDev), UART_FLAG_TC);

  if (l_pstDev->m_pvSendSem == NULL)
  {
    l_pstDev->m_pvSendSem = xSemaphoreCreateBinary();
  }
  if (l_pstDev->m_pvRecvSem == NULL)
  {
    l_pstDev->m_pvRecvSem = xSemaphoreCreateBinary();
  }
  if ((l_pstDev->m_pvSendSem == NULL) || (l_pstDev->m_pvRecvSem == NULL))
  {
    l_i32InitErrno = ENOMEM;
    goto init_failed;
  }

  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, 0);
  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, 0);

  l_pstDev->m_u32Baud = (unsigned int)l_i32Baud;
  l_pstDev->m_u8BitsPerChar = (unsigned char)(1 + l_i32DataBit + l_i32StopBit + ((l_i8Parity == 'N') ? 0 : 1));
  l_pstDev->m_u8TxBusy = 0;
  l_pstDev->m_u8TxError = 0;
  l_pstDev->m_u8TxGapLocked = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u8RecvDone = 0;

  l_u32CharUs = ((uint32_t)l_pstDev->m_u8BitsPerChar * 1000000U + l_pstDev->m_u32Baud - 1U) / l_pstDev->m_u32Baud;
  if (l_pstDev->m_u32Baud > 19200U)
  {
    l_u32FullGuardUs = 1750U;
  }
  else
  {
    l_u32FullGuardUs =
        ((uint32_t)l_pstDev->m_u8BitsPerChar * 3500000U + l_pstDev->m_u32Baud - 1U) / l_pstDev->m_u32Baud;
  }
  l_pstDev->m_u32FullFrameGapTicks = modbus_uart_interval_ticks(l_u32FullGuardUs);
  l_pstDev->m_u32RxToTxRemainTicks =
      modbus_uart_interval_ticks((l_u32FullGuardUs > l_u32CharUs) ? (l_u32FullGuardUs - l_u32CharUs) : 1U);

  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  l_pstDev->m_u8Initialized = 1;
  if (modbus_uart_rx_dma_restart(l_pstDev) != 0)
  {
    goto init_failed;
  }

  return 0;

init_failed:
  l_pstDev->m_u8Initialized = 0;
  if (UART_HANDLE(l_pstDev).Instance != NULL)
  {
    __HAL_UART_DISABLE_IT(&UART_HANDLE(l_pstDev), UART_IT_IDLE);
    (void)HAL_UART_AbortTransmit(&UART_HANDLE(l_pstDev));
    (void)HAL_UART_AbortReceive(&UART_HANDLE(l_pstDev));
    l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  }
  errno = l_i32InitErrno;
  return -1;
}

/**
 * @brief  Send data via UART
 * @param  l_pstDev: Pointer to the UART device structure
 * @param  l_pu8Datas: Pointer to the data buffer to be sent
 * @param  l_u32Len: Length of the data to be sent
 * @param  l_u32Timeout: Timeout for sending data
 * @retval Number of bytes sent on success, -1 on failure
 */
static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout)
{
  HAL_StatusTypeDef l_enHalStatus;
  TickType_t l_xTimeoutTicks;
  unsigned char l_u8TxError;

  if ((l_pstDev == NULL) || (l_pu8Datas == NULL) || (l_u32Len == 0U) || (l_u32Len > 0xFFFFU) ||
      !l_pstDev->m_u8Initialized || (l_pstDev->m_pvSendSem == NULL))
  {
    errno = EINVAL;
    return -1;
  }

  /* Claim the transmitter before draining the completion event. This prevents
   * another caller from consuming this transfer's completion. */
  taskENTER_CRITICAL();
  if (l_pstDev->m_u8TxBusy)
  {
    taskEXIT_CRITICAL();
    errno = EBUSY;
    return -1;
  }
  l_pstDev->m_u8TxBusy = 1;
  l_pstDev->m_u8TxError = 0;
  taskEXIT_CRITICAL();

  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, 0);

  /* USART IDLE is raised after about one character of silence. Wait only the
   * remaining part of t3.5; TX completion and recovery use a full t3.5. */
  modbus_uart_wait_rtu_guard(l_pstDev);

  if (modbus_uart_rx_dma_stop(l_pstDev) != 0)
  {
    l_pstDev->m_u8TxError = 1;
    modbus_uart_tx_abort(l_pstDev);
    errno = EIO;
    return -1;
  }

  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_TX);

  __HAL_UART_CLEAR_FLAG(&UART_HANDLE(l_pstDev), UART_FLAG_TC);
  l_enHalStatus = HAL_UART_Transmit_DMA(&UART_HANDLE(l_pstDev), l_pu8Datas, (uint16_t)l_u32Len);
  if (l_enHalStatus != HAL_OK)
  {
    l_pstDev->m_u8TxError = 1;
    modbus_uart_tx_abort(l_pstDev);
    errno = (l_enHalStatus == HAL_BUSY) ? EBUSY : EIO;
    return -1;
  }

  /* In STM32F4 HAL normal-DMA mode, HAL_UART_TxCpltCallback is dispatched by
   * the USART TC interrupt, not by the earlier DMA transfer-complete event. */
  l_xTimeoutTicks = modbus_uart_tx_timeout_ticks(l_pstDev, l_u32Len, l_u32Timeout);
  if (xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, l_xTimeoutTicks) != pdTRUE)
  {
    l_pstDev->m_u32TxTimeoutCount++;
    l_pstDev->m_u8TxError = 1;
    modbus_uart_tx_abort(l_pstDev);
    (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, 0);
    errno = ETIMEDOUT;
    return -1;
  }

  l_u8TxError = l_pstDev->m_u8TxError;
  if (l_u8TxError)
  {
    (void)HAL_UART_AbortTransmit(&UART_HANDLE(l_pstDev));
    __HAL_UART_CLEAR_FLAG(&UART_HANDLE(l_pstDev), UART_FLAG_TC);
  }

  /* This board controls RS485 DE/RE through the PCF8574.  The PCF8574 driver
   * uses software I2C and contains a 10 ms delay, so the direction switch must
   * remain in task context instead of HAL_UART_TxCpltCallback(). */
  if (modbus_uart_rx_dma_restart(l_pstDev) != 0)
  {
    l_u8TxError = 1;
  }
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);

  taskENTER_CRITICAL();
  l_pstDev->m_u8TxBusy = 0;
  taskEXIT_CRITICAL();

  if (l_u8TxError)
  {
    errno = EIO;
    return -1;
  }
  return (int)l_u32Len;
}

/**
 * @brief Stop reception without disturbing an active TX DMA transfer.
 */
static int modbus_uart_rx_dma_stop(pstUartDevice l_pstDev)
{
  __HAL_UART_DISABLE_IT(&UART_HANDLE(l_pstDev), UART_IT_IDLE);
  return (HAL_UART_AbortReceive(&UART_HANDLE(l_pstDev)) == HAL_OK) ? 0 : -1;
}

/**
 * @brief Stop RX DMA from an ISR without using HAL's blocking abort routine.
 */
static void modbus_uart_rx_dma_stop_from_isr(pstUartDevice l_pstDev)
{
  UART_HandleTypeDef *l_pstHalHandle = &UART_HANDLE(l_pstDev);
  DMA_HandleTypeDef *l_pstDmaHandle = &RX_DMA_HANDLE(l_pstDev);

  __HAL_UART_DISABLE_IT(l_pstHalHandle, UART_IT_IDLE);
  ATOMIC_CLEAR_BIT(l_pstHalHandle->Instance->CR1, USART_CR1_PEIE);
  ATOMIC_CLEAR_BIT(l_pstHalHandle->Instance->CR3, USART_CR3_EIE | USART_CR3_DMAR);
  __HAL_DMA_DISABLE(l_pstDmaHandle);
  __HAL_DMA_DISABLE_IT(l_pstDmaHandle, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(l_pstDmaHandle, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(l_pstDmaHandle, DMA_IT_TE);
  __HAL_DMA_DISABLE_IT(l_pstDmaHandle, DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(l_pstDmaHandle, DMA_IT_FE);
  __HAL_DMA_CLEAR_FLAG(l_pstDmaHandle,
                       __HAL_DMA_GET_TC_FLAG_INDEX(l_pstDmaHandle) | __HAL_DMA_GET_HT_FLAG_INDEX(l_pstDmaHandle) |
                       __HAL_DMA_GET_TE_FLAG_INDEX(l_pstDmaHandle) | __HAL_DMA_GET_DME_FLAG_INDEX(l_pstDmaHandle) |
                       __HAL_DMA_GET_FE_FLAG_INDEX(l_pstDmaHandle));

  l_pstDmaHandle->State = HAL_DMA_STATE_READY;
  __HAL_UNLOCK(l_pstDmaHandle);
  l_pstHalHandle->RxXferCount = 0U;
  l_pstHalHandle->RxState = HAL_UART_STATE_READY;
  l_pstHalHandle->ReceptionType = HAL_UART_RECEPTION_STANDARD;
}

/**
 * @brief Restart reception from the beginning of the RTU frame buffer.
 */
static int modbus_uart_rx_dma_restart(pstUartDevice l_pstDev)
{
  if (modbus_uart_rx_dma_stop(l_pstDev) != 0)
  {
    return -1;
  }

  __HAL_UART_CLEAR_OREFLAG(&UART_HANDLE(l_pstDev));
  __HAL_UART_CLEAR_IDLEFLAG(&UART_HANDLE(l_pstDev));
  if (HAL_UART_Receive_DMA(&UART_HANDLE(l_pstDev), l_pstDev->m_pu8RecvBuff, l_pstDev->m_u16RecvBuffSize) != HAL_OK)
  {
    return -1;
  }

  /* A half-buffer event cannot delimit an RTU frame and only adds ISR load. */
  __HAL_DMA_DISABLE_IT(&RX_DMA_HANDLE(l_pstDev), DMA_IT_HT);
  __HAL_UART_ENABLE_IT(&UART_HANDLE(l_pstDev), UART_IT_IDLE);
  return 0;
}

/**
 * @brief Abort TX and return the half-duplex interface to a recoverable RX state.
 */
static void modbus_uart_tx_abort(pstUartDevice l_pstDev)
{
  (void)HAL_UART_AbortTransmit(&UART_HANDLE(l_pstDev));
  __HAL_UART_CLEAR_FLAG(&UART_HANDLE(l_pstDev), UART_FLAG_TC);

  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  (void)modbus_uart_rx_dma_restart(l_pstDev);
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);

  taskENTER_CRITICAL();
  l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCount() + l_pstDev->m_u32FullFrameGapTicks;
  l_pstDev->m_u8TxGapLocked = 1;
  l_pstDev->m_u8TxBusy = 0;
  taskEXIT_CRITICAL();
}

/**
 * @brief Wait until the RTU inter-frame silent interval has elapsed.
 */
static void modbus_uart_wait_rtu_guard(pstUartDevice l_pstDev)
{
  for (;;)
  {
    int32_t l_i32TicksRemaining;

    taskENTER_CRITICAL();
    if (!l_pstDev->m_u8TxGapLocked)
    {
      taskEXIT_CRITICAL();
      return;
    }

    l_i32TicksRemaining = (int32_t)(l_pstDev->m_u32NextTxAllowedTick - xTaskGetTickCount());
    if (l_i32TicksRemaining <= 0)
    {
      l_pstDev->m_u8TxGapLocked = 0;
      taskEXIT_CRITICAL();
      return;
    }
    taskEXIT_CRITICAL();

    vTaskDelay((TickType_t)l_i32TicksRemaining);
  }
}

/**
 * @brief Convert microseconds to a conservative, non-zero number of OS ticks.
 */
static TickType_t modbus_uart_interval_ticks(uint32_t l_u32IntervalUs)
{
  uint32_t l_u32IntervalMs = (l_u32IntervalUs + 999U) / 1000U;
  TickType_t l_xTicks = pdMS_TO_TICKS(l_u32IntervalMs);

  return (l_xTicks == 0) ? 1 : l_xTicks;
}

/**
 * @brief Calculate a TX wait that cannot expire before the frame leaves the wire.
 */
static TickType_t modbus_uart_tx_timeout_ticks(pstUartDevice l_pstDev, unsigned int l_u32Len,
                                               unsigned int l_u32TimeoutMs)
{
  uint32_t l_u32Baud = l_pstDev->m_u32Baud ? l_pstDev->m_u32Baud : 9600U;
  uint32_t l_u32BitsPerChar = l_pstDev->m_u8BitsPerChar ? l_pstDev->m_u8BitsPerChar : 11U;
  uint32_t l_u32WireMs;
  uint32_t l_u32GuardMs;
  uint32_t l_u32MinimumMs;
  TickType_t l_xTicks;

  l_u32WireMs = ((l_u32Len * l_u32BitsPerChar * 1000U) + l_u32Baud - 1U) / l_u32Baud;
  l_u32GuardMs = ((4U * l_u32BitsPerChar * 1000U) + l_u32Baud - 1U) / l_u32Baud;
  if (l_u32GuardMs < 20U)
  {
    l_u32GuardMs = 20U;
  }
  l_u32MinimumMs = l_u32WireMs + l_u32GuardMs;
  if (l_u32TimeoutMs < l_u32MinimumMs)
  {
    l_u32TimeoutMs = l_u32MinimumMs;
  }

  l_xTicks = pdMS_TO_TICKS(l_u32TimeoutMs);
  return (l_xTicks == 0) ? 1 : l_xTicks;
}

/**
 * @brief  Receive data via UART
 * @param  l_pstDev: Pointer to the UART device structure
 * @param  l_pu8Datas: Pointer to the buffer to store received data
 * @param  l_u32Len: Length of the data to be read
 * @param  l_u32Timeout: Timeout for receiving data
 * @retval Number of bytes received on success, -1 on failure
 */
static int modbus_uart_recv(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout)
{
  TickType_t l_xTimeoutTicks;

  if ((l_pstDev == NULL) || (l_pu8Datas == NULL) || (l_u32Len == 0U) || !l_pstDev->m_u8Initialized ||
      (l_pstDev->m_pvRecvSem == NULL))
  {
    errno = EINVAL;
    return -1;
  }

  if (l_pstDev->m_u8RecvDone)
  {
    if ((l_pstDev->m_u16ReadLen > l_pstDev->m_u16RecvLen) ||
        (l_u32Len > (unsigned int)(l_pstDev->m_u16RecvLen - l_pstDev->m_u16ReadLen)))
    {
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u8RecvDone = 0;
      (void)modbus_uart_rx_dma_restart(l_pstDev);
      errno = EMSGSIZE;
      return -1;
    }

    /* data has been received, copy data to user buffer */
    memcpy(l_pu8Datas, l_pstDev->m_pu8RecvBuff + l_pstDev->m_u16ReadLen, l_u32Len);
    l_pstDev->m_u16ReadLen += l_u32Len;
    if (l_pstDev->m_u16ReadLen >= l_pstDev->m_u16RecvLen)
    {
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u8RecvDone = 0;
      if (modbus_uart_rx_dma_restart(l_pstDev) != 0)
      {
        errno = EIO;
        return -1;
      }
    }
    return (int)l_u32Len;
  }
  else
  {
    if (l_u32Timeout == MODBUS_MAX_DELAY)
    {
      l_xTimeoutTicks = portMAX_DELAY;
    }
    else
    {
      l_xTimeoutTicks = pdMS_TO_TICKS(l_u32Timeout);
      if (l_xTimeoutTicks == 0)
      {
        l_xTimeoutTicks = 1;
      }
    }

    if (xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, l_xTimeoutTicks) == pdTRUE)
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
      l_pstDev->m_u8RecvDone = 0;
      (void)modbus_uart_rx_dma_restart(l_pstDev);
      errno = ETIMEDOUT;
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
  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    errno = EINVAL;
    return -1;
  }
  if (l_pstDev->m_u8TxBusy)
  {
    errno = EBUSY;
    return -1;
  }

  memset(l_pstDev->m_pu8RecvBuff, 0, l_pstDev->m_u16RecvBuffSize);
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, 0);
  if (modbus_uart_rx_dma_restart(l_pstDev) != 0)
  {
    errno = EIO;
    return -1;
  }
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
    case MODBUS_UART2:
      pcf8574_write_bit(RS485_RE_IO, l_enDir);
      break;
    default:
      break;
  }
}

static pstUartDevice find_uart_device_by_handle(UART_HandleTypeDef *l_pstHalHandle)
{
  unsigned char l_u8Index;

  for (l_u8Index = 0; l_u8Index < MODBUS_UART_COUNT; l_u8Index++)
  {
    if ((sArM_pstUartDevs[l_u8Index] != NULL) && (&UART_HANDLE(sArM_pstUartDevs[l_u8Index]) == l_pstHalHandle))
    {
      return sArM_pstUartDevs[l_u8Index];
    }
  }
  return NULL;
}

static void modbus_uart_rx_idle_isr(pstUartDevice l_pstDev, BaseType_t *l_pxHigherPriorityTaskWoken)
{
  unsigned short l_u16Remaining;
  unsigned short l_u16Received;

  if ((__HAL_UART_GET_FLAG(&UART_HANDLE(l_pstDev), UART_FLAG_IDLE) == RESET) ||
      (__HAL_UART_GET_IT_SOURCE(&UART_HANDLE(l_pstDev), UART_IT_IDLE) == RESET))
  {
    return;
  }

  l_u16Remaining = (unsigned short)__HAL_DMA_GET_COUNTER(&RX_DMA_HANDLE(l_pstDev));
  __HAL_UART_CLEAR_IDLEFLAG(&UART_HANDLE(l_pstDev));
  l_u16Received = l_pstDev->m_u16RecvBuffSize - l_u16Remaining;
  if (l_u16Received == 0U)
  {
    return;
  }

  modbus_uart_rx_dma_stop_from_isr(l_pstDev);
  l_pstDev->m_u16RecvLen = l_u16Received;
  l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32RxToTxRemainTicks;
  l_pstDev->m_u8TxGapLocked = 1;

  if (l_pstDev->m_pvRecvSem != NULL)
  {
    (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvRecvSem, l_pxHigherPriorityTaskWoken);
  }
}

void USART2_IRQHandler(void)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  pstUartDevice l_pstDev = sArM_pstUartDevs[MODBUS_UART2];

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }

  HAL_UART_IRQHandler(&UART_HANDLE(l_pstDev));
  modbus_uart_rx_idle_isr(l_pstDev, &l_xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

void DMA1_Stream6_IRQHandler(void)
{
  if ((sArM_pstUartDevs[MODBUS_UART2] != NULL) && sArM_pstUartDevs[MODBUS_UART2]->m_u8Initialized)
  {
    HAL_DMA_IRQHandler(&TX_DMA_HANDLE(sArM_pstUartDevs[MODBUS_UART2]));
  }
}

void DMA1_Stream5_IRQHandler(void)
{
  if ((sArM_pstUartDevs[MODBUS_UART2] != NULL) && sArM_pstUartDevs[MODBUS_UART2]->m_u8Initialized)
  {
    HAL_DMA_IRQHandler(&RX_DMA_HANDLE(sArM_pstUartDevs[MODBUS_UART2]));
  }
}

void USART3_IRQHandler(void)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  pstUartDevice l_pstDev = sArM_pstUartDevs[MODBUS_UART3];

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }

  HAL_UART_IRQHandler(&UART_HANDLE(l_pstDev));
  modbus_uart_rx_idle_isr(l_pstDev, &l_xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

void DMA1_Stream3_IRQHandler(void)
{
  if ((sArM_pstUartDevs[MODBUS_UART3] != NULL) && sArM_pstUartDevs[MODBUS_UART3]->m_u8Initialized)
  {
    HAL_DMA_IRQHandler(&TX_DMA_HANDLE(sArM_pstUartDevs[MODBUS_UART3]));
  }
}

void DMA1_Stream1_IRQHandler(void)
{
  if ((sArM_pstUartDevs[MODBUS_UART3] != NULL) && sArM_pstUartDevs[MODBUS_UART3]->m_u8Initialized)
  {
    HAL_DMA_IRQHandler(&RX_DMA_HANDLE(sArM_pstUartDevs[MODBUS_UART3]));
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  pstUartDevice l_pstDev = find_uart_device_by_handle(huart);

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }

  /* RX DMA is normal-mode. Keep IDLE disabled until the task consumes this
   * full-size frame and explicitly rearms the receiver. */
  __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
  l_pstDev->m_u16RecvLen = l_pstDev->m_u16RecvBuffSize;
  l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32FullFrameGapTicks;
  l_pstDev->m_u8TxGapLocked = 1;

  if (l_pstDev->m_pvRecvSem != NULL)
  {
    (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvRecvSem, &l_xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  pstUartDevice l_pstDev = find_uart_device_by_handle(huart);

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized || !l_pstDev->m_u8TxBusy)
  {
    return;
  }

  l_pstDev->m_u32TxCompleteCount++;
  l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32FullFrameGapTicks;
  l_pstDev->m_u8TxGapLocked = 1;

  /* Direct-GPIO RS485 porting note:
   * HAL calls this callback from the UART TC interrupt after the final stop
   * bit has left the shift register.  Therefore this is the preferred place
   * to switch an ISR-safe, directly connected DE/RE pin back to RX.  Do not
   * switch it in the earlier DMA transfer-complete interrupt.
   *
   * Example (DE high = TX, DE low = RX; replace the port/pin names):
   *
   *   HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
   *
   * For a direct-GPIO implementation, arm RX DMA before releasing DE when the
   * minimum response turnaround must be supported.  That RX rearm must itself
   * be ISR-safe.  The current PCF8574 implementation cannot use this path, so
   * it only wakes the task here and performs RX rearm/direction switching in
   * modbus_uart_send().
   */
  if (l_pstDev->m_pvSendSem != NULL)
  {
    (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvSendSem, &l_xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  pstUartDevice l_pstDev = find_uart_device_by_handle(huart);

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }

  if (l_pstDev->m_u8TxBusy)
  {
    l_pstDev->m_u8TxError = 1;
    if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U)
    {
      l_pstDev->m_u32TxDmaErrorCount++;
    }
    l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32FullFrameGapTicks;
    l_pstDev->m_u8TxGapLocked = 1;
    if (l_pstDev->m_pvSendSem != NULL)
    {
      (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvSendSem, &l_xHigherPriorityTaskWoken);
    }
  }
  else
  {
    /* A blocking RX error leaves the HAL receive state stopped. Recover the
     * receiver immediately so a transient line error cannot wedge RTU. */
    (void)modbus_uart_rx_dma_restart(l_pstDev);
  }

  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}
