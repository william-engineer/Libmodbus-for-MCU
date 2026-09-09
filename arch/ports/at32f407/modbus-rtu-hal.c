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
#include "at32f403a_407.h"

#include "../../modbus-rtu-hal.h"

static int modbus_uart_init(pstUartDevice l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit);
static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout);
static int modbus_uart_recv(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout);
static int modbus_uart_flush(pstUartDevice l_pstDev);
static void modbus_rs485_dir_switch(pstUartDevice l_pstDev, enModbusDir_t l_enDir);
static void modbus_uart_rx_dma_restart(pstUartDevice l_pstDev);
static void modbus_uart_tx_abort(pstUartDevice l_pstDev);
static void modbus_uart_wait_rtu_guard(pstUartDevice l_pstDev);
static TickType_t modbus_uart_interval_ticks(uint32_t l_u32IntervalUs);
static TickType_t modbus_uart_tx_timeout_ticks(pstUartDevice l_pstDev, unsigned int l_u32Len,
                                               unsigned int l_u32TimeoutMs);

typedef struct
{
  usart_type *m_pstUsartPeripheral;
  dma_channel_type *m_pstTxDmaChannel;
  dma_channel_type *m_pstRxDmaChannel;
  dma_type *m_pstDmaPeripheral;
} stUartContext;

#define UART_CONTEXT(dev)   ((stUartContext *)((dev)->m_pvPortContext))
#define UART_HANDLE(dev)    (UART_CONTEXT(dev)->m_pstUsartPeripheral)
#define TX_DMA_HANDLE(dev)  (UART_CONTEXT(dev)->m_pstTxDmaChannel)
#define RX_DMA_HANDLE(dev)  (UART_CONTEXT(dev)->m_pstRxDmaChannel)
#define DMA_PERIPHERAL(dev) (UART_CONTEXT(dev)->m_pstDmaPeripheral)

/* Max between RTU and TCP max adu length */
#define MODBUS_RECV_BUFF_SIZE 300
static stUartContext s_stUart2Context = {.m_pstUsartPeripheral = USART2,
                                         .m_pstTxDmaChannel = DMA1_CHANNEL7,
                                         .m_pstRxDmaChannel = DMA1_CHANNEL6,
                                         .m_pstDmaPeripheral = DMA1};
static unsigned char sArM_u8ModbusRecvBuff[MODBUS_RECV_BUFF_SIZE];
static stUartDevice s_stModbusRtuDev = {.m_pi8Name = "uart2",
                                        .m_enDevId = MODBUS_UART2,
                                        .m_pu8RecvBuff = sArM_u8ModbusRecvBuff,
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

/* Device table indexes are private to this board port. */
static pstUartDevice sArM_pstUartDevs[MODBUS_UART_COUNT] = {
  NULL,
  &s_stModbusRtuDev
};

/**
 * @brief  find the UART device by name
 * @param  l_pi8Name: Name of the UART device
 * @retval Pointer to the UART device structure if found, NULL otherwise
 */
pstUartDevice find_uart_device(const char *l_pi8Name)
{
  unsigned char l_u8Index;

  if (l_pi8Name == NULL)
  {
    return NULL;
  }

  for (l_u8Index = 0; l_u8Index < MODBUS_UART_COUNT; l_u8Index++)
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
  gpio_init_type l_stGpioInit;
  dma_init_type l_stDmaInit;

  if (l_pstDev == NULL)
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
      /* enable peripheral clocks for UART2, GPIO and DMA1 */
      crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
      crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
      crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
      crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

      /* Create ISR-visible objects before enabling USART/DMA interrupts. */
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
        return -1;
      }

      /* Remove a completion left by a previous peripheral reinitialization. */
      (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, 0);
      (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, 0);
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u8RecvDone = 0;

      /* configure PD7 as RS485 DE pin */
      gpio_default_para_init(&l_stGpioInit);
      l_stGpioInit.gpio_mode = GPIO_MODE_OUTPUT;
      l_stGpioInit.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
      l_stGpioInit.gpio_pull = GPIO_PULL_NONE;
      l_stGpioInit.gpio_pins = GPIO_PINS_7;
      gpio_init(GPIOD, &l_stGpioInit);

      /* configure PD5, PD6 as alternate function for USART2 TX/RX */
      gpio_default_para_init(&l_stGpioInit);
      gpio_pin_remap_config(USART2_GMUX_0001, TRUE);

      l_stGpioInit.gpio_mode = GPIO_MODE_MUX;
      l_stGpioInit.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
      l_stGpioInit.gpio_pull = GPIO_PULL_NONE;
      l_stGpioInit.gpio_pins = GPIO_PINS_5;
      gpio_init(GPIOD, &l_stGpioInit);

      l_stGpioInit.gpio_mode = GPIO_MODE_INPUT;
      l_stGpioInit.gpio_pins = GPIO_PINS_6;
      gpio_init(GPIOD, &l_stGpioInit);

      /* configure USART2 */
      usart_init(UART_HANDLE(l_pstDev), l_i32Baud,
                 ((l_i32DataBit + ((l_i8Parity == 'N') ? 0 : 1)) > 8) ? USART_DATA_9BITS : USART_DATA_8BITS,
                 (l_i32StopBit == 1) ? USART_STOP_1_BIT : USART_STOP_2_BIT);

      usart_parity_selection_config(UART_HANDLE(l_pstDev),
                                    (l_i8Parity == 'E') ? USART_PARITY_EVEN
                                                        : ((l_i8Parity == 'O') ? USART_PARITY_ODD : USART_PARITY_NONE));

      usart_transmitter_enable(UART_HANDLE(l_pstDev), TRUE);
      usart_receiver_enable(UART_HANDLE(l_pstDev), TRUE);
      usart_enable(UART_HANDLE(l_pstDev), TRUE);

      l_pstDev->m_u32Baud = (unsigned int)l_i32Baud;
      l_pstDev->m_u8BitsPerChar = (unsigned char)(1 + l_i32DataBit + l_i32StopBit + ((l_i8Parity == 'N') ? 0 : 1));
      l_pstDev->m_u8TxBusy = 0;
      l_pstDev->m_u8TxError = 0;
      l_pstDev->m_u8TxGapLocked = 0;

      {
        uint32_t l_u32CharUs =
            ((uint32_t)l_pstDev->m_u8BitsPerChar * 1000000U + l_pstDev->m_u32Baud - 1U) / l_pstDev->m_u32Baud;
        uint32_t l_u32FullGuardUs;

        /* Modbus Serial Line V1.02 recommends fixed 1.75 ms timers
         * above 19200 baud; below that, frame separation is 3.5 characters. */
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
      }

      /* configure interrupt */
      usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_IDLE_INT, FALSE);
      usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_TDC_INT, FALSE);
      usart_flag_clear(UART_HANDLE(l_pstDev), USART_TDC_FLAG);

      /* configure DMA1 Channel7 for USART2 TX (Memory to Peripheral) */
      dma_reset(TX_DMA_HANDLE(l_pstDev));
      dma_default_para_init(&l_stDmaInit);
      l_stDmaInit.buffer_size = 0;
      l_stDmaInit.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
      l_stDmaInit.memory_base_addr = 0;
      l_stDmaInit.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
      l_stDmaInit.memory_inc_enable = TRUE;
      l_stDmaInit.peripheral_base_addr = (uint32_t)&UART_HANDLE(l_pstDev)->dt;
      l_stDmaInit.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
      l_stDmaInit.peripheral_inc_enable = FALSE;
      l_stDmaInit.priority = DMA_PRIORITY_HIGH;
      l_stDmaInit.loop_mode_enable = FALSE;
      dma_init(TX_DMA_HANDLE(l_pstDev), &l_stDmaInit);
      dma_flexible_config(DMA_PERIPHERAL(l_pstDev), FLEX_CHANNEL7, DMA_FLEXIBLE_UART2_TX);
      dma_interrupt_enable(TX_DMA_HANDLE(l_pstDev), DMA_FDT_INT | DMA_DTERR_INT, TRUE);
      nvic_irq_enable(DMA1_Channel7_IRQn, 4, 0);
      usart_dma_transmitter_enable(UART_HANDLE(l_pstDev), FALSE);
      dma_channel_enable(TX_DMA_HANDLE(l_pstDev), FALSE);

      /* configure DMA1 Channel6 for USART2 RX (Peripheral to Memory) */
      dma_reset(RX_DMA_HANDLE(l_pstDev));
      dma_default_para_init(&l_stDmaInit);
      l_stDmaInit.buffer_size = l_pstDev->m_u16RecvBuffSize;
      l_stDmaInit.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
      l_stDmaInit.memory_base_addr = (uint32_t)l_pstDev->m_pu8RecvBuff;
      l_stDmaInit.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
      l_stDmaInit.memory_inc_enable = TRUE;
      l_stDmaInit.peripheral_base_addr = (uint32_t)&UART_HANDLE(l_pstDev)->dt;
      l_stDmaInit.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
      l_stDmaInit.peripheral_inc_enable = FALSE;
      l_stDmaInit.priority = DMA_PRIORITY_MEDIUM;
      l_stDmaInit.loop_mode_enable = FALSE;
      dma_init(RX_DMA_HANDLE(l_pstDev), &l_stDmaInit);
      /* Associate the DMA handle */
      dma_flexible_config(DMA_PERIPHERAL(l_pstDev), FLEX_CHANNEL6, DMA_FLEXIBLE_UART2_RX);

      /* NVIC configuration for DMA uart RX */
      dma_interrupt_enable(RX_DMA_HANDLE(l_pstDev), DMA_FDT_INT, TRUE);
      nvic_irq_enable(DMA1_Channel6_IRQn, 4, 0);

      /* Enable DMA for USART2 RX */
      l_pstDev->m_u8Initialized = 1;
      l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
      usart_dma_receiver_enable(UART_HANDLE(l_pstDev), TRUE);
      dma_channel_enable(RX_DMA_HANDLE(l_pstDev), TRUE);
      usart_flag_clear(UART_HANDLE(l_pstDev), USART_IDLEF_FLAG);
      usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_IDLE_INT, TRUE);
      nvic_irq_enable(USART2_IRQn, 6, 0);

      break;
    default:
      errno = ENODEV;
      return -1;
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
static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout)
{
  TickType_t l_xTimeoutTicks;
  unsigned char l_u8TxError;

  if ((l_pstDev == NULL) || (l_pu8Datas == NULL) || (l_u32Len == 0) || (l_u32Len > 0xFFFFU) ||
      !l_pstDev->m_u8Initialized || (l_pstDev->m_pvSendSem == NULL) || (TX_DMA_HANDLE(l_pstDev) == NULL) ||
      (RX_DMA_HANDLE(l_pstDev) == NULL))
  {
    errno = EINVAL;
    return -1;
  }

  /* Claim the transmitter before draining the completion event. This prevents
   * a second caller from consuming the first caller's completion. */
  taskENTER_CRITICAL();
  if (l_pstDev->m_u8TxBusy)
  {
    taskEXIT_CRITICAL();
    errno = EBUSY;
    return -1;
  }

  usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_TDC_INT, FALSE);
  usart_dma_transmitter_enable(UART_HANDLE(l_pstDev), FALSE);
  dma_channel_enable(TX_DMA_HANDLE(l_pstDev), FALSE);
  dma_flag_clear(DMA1_GL7_FLAG | DMA1_FDT7_FLAG | DMA1_HDT7_FLAG | DMA1_DTERR7_FLAG);
  usart_flag_clear(UART_HANDLE(l_pstDev), USART_TDC_FLAG);
  l_pstDev->m_u8TxError = 0;
  l_pstDev->m_u8TxBusy = 1;
  taskEXIT_CRITICAL();

  /* A stale completion must never make a new transfer return early. No TX
   * completion source is enabled while this event is drained. */
  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, 0);

  /* USART IDLE is raised after roughly one character of silence. Wait the
   * remaining portion of RTU t3.5 without blocking interrupts or the network. */
  modbus_uart_wait_rtu_guard(l_pstDev);

  /* Only register programming is protected; interrupts remain enabled while
   * the frame is physically transmitted. */
  taskENTER_CRITICAL();
  /* Do not receive local echo while DE is asserted. The RX DMA is armed again
   * before DE returns to receive mode in the TDC ISR. */
  dma_channel_enable(RX_DMA_HANDLE(l_pstDev), FALSE);
  usart_dma_receiver_enable(UART_HANDLE(l_pstDev), FALSE);

  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_TX);
  TX_DMA_HANDLE(l_pstDev)->maddr = (uint32_t)l_pu8Datas;
  dma_data_number_set(TX_DMA_HANDLE(l_pstDev), (uint16_t)l_u32Len);
  dma_channel_enable(TX_DMA_HANDLE(l_pstDev), TRUE);
  usart_dma_transmitter_enable(UART_HANDLE(l_pstDev), TRUE);
  taskEXIT_CRITICAL();

  /* Preserve libmodbus write semantics: return only after the final stop bit
   * has left the UART and RS-485 has switched back to receive. */
  l_xTimeoutTicks = modbus_uart_tx_timeout_ticks(l_pstDev, l_u32Len, l_u32Timeout);
  if (xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, l_xTimeoutTicks) != pdTRUE)
  {
    modbus_uart_tx_abort(l_pstDev);
    (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, 0);
    errno = ETIMEDOUT;
    return -1;
  }

  taskENTER_CRITICAL();
  l_u8TxError = l_pstDev->m_u8TxError;
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
 * @brief Restart reception from the beginning of the RTU frame buffer.
 */
static void modbus_uart_rx_dma_restart(pstUartDevice l_pstDev)
{
  dma_channel_enable(RX_DMA_HANDLE(l_pstDev), FALSE);
  usart_dma_receiver_enable(UART_HANDLE(l_pstDev), FALSE);
  dma_flag_clear(DMA1_GL6_FLAG | DMA1_FDT6_FLAG | DMA1_HDT6_FLAG | DMA1_DTERR6_FLAG);
  dma_data_number_set(RX_DMA_HANDLE(l_pstDev), l_pstDev->m_u16RecvBuffSize);
  usart_dma_receiver_enable(UART_HANDLE(l_pstDev), TRUE);
  dma_channel_enable(RX_DMA_HANDLE(l_pstDev), TRUE);
}

/**
 * @brief Stop a failed TX transaction and return the half-duplex bus to RX.
 */
static void modbus_uart_tx_abort(pstUartDevice l_pstDev)
{
  taskENTER_CRITICAL();
  l_pstDev->m_u32TxTimeoutCount++;
  l_pstDev->m_u8TxBusy = 0;
  l_pstDev->m_u8TxError = 1;
  usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_TDC_INT, FALSE);
  usart_dma_transmitter_enable(UART_HANDLE(l_pstDev), FALSE);
  dma_channel_enable(TX_DMA_HANDLE(l_pstDev), FALSE);
  dma_flag_clear(DMA1_GL7_FLAG | DMA1_FDT7_FLAG | DMA1_HDT7_FLAG | DMA1_DTERR7_FLAG);
  usart_flag_clear(UART_HANDLE(l_pstDev), USART_TDC_FLAG);

  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  modbus_uart_rx_dma_restart(l_pstDev);
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCount() + l_pstDev->m_u32FullFrameGapTicks;
  l_pstDev->m_u8TxGapLocked = 1;
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
 * @brief Convert microseconds to a conservative non-zero number of OS ticks.
 */
static TickType_t modbus_uart_interval_ticks(uint32_t l_u32IntervalUs)
{
  uint32_t l_u32IntervalMs = (l_u32IntervalUs + 999U) / 1000U;
  TickType_t l_xTicks = pdMS_TO_TICKS(l_u32IntervalMs);

  return (l_xTicks == 0) ? 1 : l_xTicks;
}

/**
 * @brief Calculate a timeout that cannot expire before the frame can leave the wire.
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
      modbus_uart_rx_dma_restart(l_pstDev);
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
      l_pstDev->m_u8RecvDone = 0;
      modbus_uart_rx_dma_restart(l_pstDev);
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
  modbus_uart_rx_dma_restart(l_pstDev);
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
  if (l_pstDev == NULL)
  {
    return;
  }

  switch (l_pstDev->m_enDevId)
  {
    case MODBUS_UART2:
      if (l_enDir == MODBUS_DIR_TX)
      {
        /* Assert the RS-485 driver-enable pin. */
        gpio_bits_set(GPIOD, GPIO_PINS_7);
      }
      else
      {
        /* Release the bus and enable reception. */
        gpio_bits_reset(GPIOD, GPIO_PINS_7);
      }
      break;
    default:
      break;
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

  /* DMA completion only means the final byte reached USART->DT. TDC is the
   * event that guarantees the final stop bit has left the pin. */
  if (usart_interrupt_flag_get(UART_HANDLE(l_pstDev), USART_TDC_FLAG) != RESET)
  {
    usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_TDC_INT, FALSE);
    usart_flag_clear(UART_HANDLE(l_pstDev), USART_TDC_FLAG);

    if (l_pstDev->m_u8TxBusy)
    {
      usart_dma_transmitter_enable(UART_HANDLE(l_pstDev), FALSE);
      dma_channel_enable(TX_DMA_HANDLE(l_pstDev), FALSE);

      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u8RecvDone = 0;
      modbus_uart_rx_dma_restart(l_pstDev);
      l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
      l_pstDev->m_u32TxCompleteCount++;
      l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32FullFrameGapTicks;
      l_pstDev->m_u8TxGapLocked = 1;

      if (l_pstDev->m_pvSendSem != NULL)
      {
        (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvSendSem, &l_xHigherPriorityTaskWoken);
      }
    }
  }

  if (usart_interrupt_flag_get(UART_HANDLE(l_pstDev), USART_IDLEF_FLAG) != RESET)
  {
    usart_flag_clear(UART_HANDLE(l_pstDev), USART_IDLEF_FLAG);

    l_pstDev->m_u16RecvLen = MODBUS_RECV_BUFF_SIZE - dma_data_number_get(RX_DMA_HANDLE(l_pstDev));

    /* to avoid empty IDLE trigger */
    if (l_pstDev->m_u16RecvLen > 0)
    {
      /* stop DMA reception */
      dma_channel_enable(RX_DMA_HANDLE(l_pstDev), FALSE);
      usart_dma_receiver_enable(UART_HANDLE(l_pstDev), FALSE);

      l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32RxToTxRemainTicks;
      l_pstDev->m_u8TxGapLocked = 1;

      (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvRecvSem, &l_xHigherPriorityTaskWoken);
    }
  }

  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

// tx dma interrupt
void DMA1_Channel7_IRQHandler(void)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  pstUartDevice l_pstDev = sArM_pstUartDevs[MODBUS_UART2];

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }

  if (dma_interrupt_flag_get(DMA1_DTERR7_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_GL7_FLAG | DMA1_FDT7_FLAG | DMA1_HDT7_FLAG | DMA1_DTERR7_FLAG);
    dma_channel_enable(TX_DMA_HANDLE(l_pstDev), FALSE);
    usart_dma_transmitter_enable(UART_HANDLE(l_pstDev), FALSE);
    usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_TDC_INT, FALSE);
    usart_flag_clear(UART_HANDLE(l_pstDev), USART_TDC_FLAG);

    if (l_pstDev->m_u8TxBusy)
    {
      l_pstDev->m_u32TxDmaErrorCount++;
      l_pstDev->m_u8TxError = 1;
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u8RecvDone = 0;
      modbus_uart_rx_dma_restart(l_pstDev);
      l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
      l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32FullFrameGapTicks;
      l_pstDev->m_u8TxGapLocked = 1;

      if (l_pstDev->m_pvSendSem != NULL)
      {
        (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvSendSem, &l_xHigherPriorityTaskWoken);
      }
    }
  }
  else if (dma_interrupt_flag_get(DMA1_FDT7_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_GL7_FLAG | DMA1_FDT7_FLAG | DMA1_HDT7_FLAG | DMA1_DTERR7_FLAG);
    dma_channel_enable(TX_DMA_HANDLE(l_pstDev), FALSE);
    usart_dma_transmitter_enable(UART_HANDLE(l_pstDev), FALSE);

    if (l_pstDev->m_u8TxBusy)
    {
      usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_TDC_INT, TRUE);
    }
  }

  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

// rx dma interrupt
void DMA1_Channel6_IRQHandler(void)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  pstUartDevice l_pstDev = sArM_pstUartDevs[MODBUS_UART2];

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }

  if ((dma_interrupt_flag_get(DMA1_FDT6_FLAG) != RESET) || (dma_interrupt_flag_get(DMA1_HDT6_FLAG) != RESET))
  {
    dma_flag_clear(DMA1_FDT6_FLAG | DMA1_HDT6_FLAG);

    /* Disable IDLE interrupt to prevent immediate trigger after DMA transfer completion */
    dma_channel_enable(RX_DMA_HANDLE(l_pstDev), FALSE);
    usart_dma_receiver_enable(UART_HANDLE(l_pstDev), FALSE);
    usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_IDLE_INT, FALSE);

    /* Calculate the number of bytes received */
    l_pstDev->m_u16RecvLen = MODBUS_RECV_BUFF_SIZE;

    /* Enable IDLE interrupt */
    usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_IDLE_INT, TRUE);

    /* Was the message posted successfully? */
    if (xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvRecvSem, &l_xHigherPriorityTaskWoken) != pdFAIL)
    {
      /* If l_xHigherPriorityTaskWoken is now set to pdTRUE then a context
      switch should be requested. */
      portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
    }
  }
}
