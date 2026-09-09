/*******************************************************************
** Project Name :    Libmodbus for MCU                            **
********************************************************************
** File Name    :    modbus-rtu-hal.c                             **
** Version      :    V1.1.0                                       **
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

#include "gd32f527.h"

#include "../../modbus-rtu-hal.h"

#define MODBUS_RECV_BUFF_SIZE             300U
#define MODBUS_DMA_IRQ_PREEMPT_PRIORITY   4U
#define MODBUS_USART_IRQ_PREEMPT_PRIORITY 6U

typedef struct
{
  uint32_t m_u32UsartPeripheral;
  dma_channel_enum m_enTxDmaChannel;
  dma_channel_enum m_enRxDmaChannel;
  uint32_t m_u32DmaPeripheral;
  rcu_periph_enum m_enGpioClock;
  rcu_periph_enum m_enUsartClock;
  uint32_t m_u32GpioPort;
  uint32_t m_u32TxPin;
  uint32_t m_u32RxPin;
  uint32_t m_u32DirPort;
  uint32_t m_u32DirPin;
  IRQn_Type m_enUsartIrq;
  IRQn_Type m_enTxDmaIrq;
  IRQn_Type m_enRxDmaIrq;
} stUartContext;

#define UART_CONTEXT(dev)   ((stUartContext *)((dev)->m_pvPortContext))
#define UART_HANDLE(dev)    (UART_CONTEXT(dev)->m_u32UsartPeripheral)
#define TX_DMA_HANDLE(dev)  (UART_CONTEXT(dev)->m_enTxDmaChannel)
#define RX_DMA_HANDLE(dev)  (UART_CONTEXT(dev)->m_enRxDmaChannel)
#define DMA_PERIPHERAL(dev) (UART_CONTEXT(dev)->m_u32DmaPeripheral)

static int modbus_uart_init(pstUartDevice l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit);
static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout);
static int modbus_uart_recv(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout);
static int modbus_uart_flush(pstUartDevice l_pstDev);
static void modbus_rs485_dir_switch(pstUartDevice l_pstDev, enModbusDir_t l_enDir);
static void modbus_uart_dma_flags_clear(uint32_t l_u32Dma, dma_channel_enum l_enChannel);
static void modbus_uart_clear_rx_status(uint32_t l_u32Usart);
static void modbus_uart_rx_dma_stop(pstUartDevice l_pstDev);
static void modbus_uart_rx_dma_restart(pstUartDevice l_pstDev);
static void modbus_uart_tx_abort(pstUartDevice l_pstDev);
static void modbus_uart_wait_rtu_guard(pstUartDevice l_pstDev);
static TickType_t modbus_uart_interval_ticks(uint32_t l_u32IntervalUs);
static TickType_t modbus_uart_tx_timeout_ticks(pstUartDevice l_pstDev, unsigned int l_u32Len,
                                               unsigned int l_u32TimeoutMs);
static void modbus_uart_irq_handler(pstUartDevice l_pstDev);
static void modbus_uart_tx_dma_irq_handler(pstUartDevice l_pstDev);
static void modbus_uart_rx_dma_irq_handler(pstUartDevice l_pstDev);

static stUartContext s_stUart1Context = {.m_u32UsartPeripheral = USART1,
                                         .m_enTxDmaChannel = DMA_CH6,
                                         .m_enRxDmaChannel = DMA_CH5,
                                         .m_u32DmaPeripheral = DMA0,
                                         .m_enGpioClock = RCU_GPIOD,
                                         .m_enUsartClock = RCU_USART1,
                                         .m_u32GpioPort = GPIOD,
                                         .m_u32TxPin = GPIO_PIN_5,
                                         .m_u32RxPin = GPIO_PIN_6,
                                         .m_u32DirPort = GPIOD,
                                         .m_u32DirPin = GPIO_PIN_4,
                                         .m_enUsartIrq = USART1_IRQn,
                                         .m_enTxDmaIrq = DMA0_Channel6_IRQn,
                                         .m_enRxDmaIrq = DMA0_Channel5_IRQn};

static stUartContext s_stUart2Context = {.m_u32UsartPeripheral = USART2,
                                         .m_enTxDmaChannel = DMA_CH3,
                                         .m_enRxDmaChannel = DMA_CH1,
                                         .m_u32DmaPeripheral = DMA0,
                                         .m_enGpioClock = RCU_GPIOC,
                                         .m_enUsartClock = RCU_USART2,
                                         .m_u32GpioPort = GPIOC,
                                         .m_u32TxPin = GPIO_PIN_10,
                                         .m_u32RxPin = GPIO_PIN_11,
                                         .m_u32DirPort = GPIOC,
                                         .m_u32DirPin = GPIO_PIN_12,
                                         .m_enUsartIrq = USART2_IRQn,
                                         .m_enTxDmaIrq = DMA0_Channel3_IRQn,
                                         .m_enRxDmaIrq = DMA0_Channel1_IRQn};

static unsigned char sArM_u8ModbusRecvBuff1[MODBUS_RECV_BUFF_SIZE];
static unsigned char sArM_u8ModbusRecvBuff2[MODBUS_RECV_BUFF_SIZE];

static stUartDevice s_stModbusDev1 = {.m_pi8Name = "uart1",
                                      .m_enDevId = MODBUS_UART1,
                                      .m_pu8RecvBuff = sArM_u8ModbusRecvBuff1,
                                      .m_u16RecvBuffSize = MODBUS_RECV_BUFF_SIZE,
                                      .m_u8Initialized = 0,
                                      .m_pvPortContext = &s_stUart1Context,
                                      .m_pFnInit = modbus_uart_init,
                                      .m_pFnSend = modbus_uart_send,
                                      .m_pFnRecv = modbus_uart_recv,
                                      .m_pFnFlush = modbus_uart_flush,
                                      .m_pFnDirSwitch = modbus_rs485_dir_switch};

static stUartDevice s_stModbusDev2 = {.m_pi8Name = "uart2",
                                      .m_enDevId = MODBUS_UART2,
                                      .m_pu8RecvBuff = sArM_u8ModbusRecvBuff2,
                                      .m_u16RecvBuffSize = MODBUS_RECV_BUFF_SIZE,
                                      .m_u8Initialized = 0,
                                      .m_pvPortContext = &s_stUart2Context,
                                      .m_pFnInit = modbus_uart_init,
                                      .m_pFnSend = modbus_uart_send,
                                      .m_pFnRecv = modbus_uart_recv,
                                      .m_pFnFlush = modbus_uart_flush,
                                      .m_pFnDirSwitch = modbus_rs485_dir_switch};

static pstUartDevice sArM_pstUartDevs[MODBUS_UART_COUNT] = {
  &s_stModbusDev1, 
  &s_stModbusDev2
};

pstUartDevice find_uart_device(const char *l_pi8Name)
{
  unsigned char l_u8Index;

  if (l_pi8Name == NULL)
  {
    return NULL;
  }

  for (l_u8Index = 0; l_u8Index < MODBUS_UART_COUNT; l_u8Index++)
  {
    if ((sArM_pstUartDevs[l_u8Index] != NULL) && (strcmp(sArM_pstUartDevs[l_u8Index]->m_pi8Name, l_pi8Name) == 0))
    {
      return sArM_pstUartDevs[l_u8Index];
    }
  }
  return NULL;
}

static void modbus_uart_dma_flags_clear(uint32_t l_u32Dma, dma_channel_enum l_enChannel)
{
  dma_flag_clear(l_u32Dma, l_enChannel, DMA_FLAG_FTF);
  dma_flag_clear(l_u32Dma, l_enChannel, DMA_FLAG_HTF);
  dma_flag_clear(l_u32Dma, l_enChannel, DMA_FLAG_TAE);
  dma_flag_clear(l_u32Dma, l_enChannel, DMA_FLAG_SDE);
  dma_flag_clear(l_u32Dma, l_enChannel, DMA_FLAG_FEE);
}

/* IDLE and receive-error flags are cleared by a STAT0 read followed by DATA. */
static void modbus_uart_clear_rx_status(uint32_t l_u32Usart)
{
  volatile uint32_t l_u32Dummy;

  l_u32Dummy = USART_STAT0(l_u32Usart);
  l_u32Dummy = USART_DATA(l_u32Usart);
  (void)l_u32Dummy;
}

static void modbus_uart_rx_dma_interrupts_disable(uint32_t l_u32Dma, dma_channel_enum l_enChannel)
{
  dma_interrupt_disable(l_u32Dma, l_enChannel, DMA_INT_FTF);
  dma_interrupt_disable(l_u32Dma, l_enChannel, DMA_INT_TAE);
  dma_interrupt_disable(l_u32Dma, l_enChannel, DMA_INT_SDE);
}

static void modbus_uart_rx_dma_interrupts_enable(uint32_t l_u32Dma, dma_channel_enum l_enChannel)
{
  dma_interrupt_enable(l_u32Dma, l_enChannel, DMA_INT_FTF);
  dma_interrupt_enable(l_u32Dma, l_enChannel, DMA_INT_TAE);
  dma_interrupt_enable(l_u32Dma, l_enChannel, DMA_INT_SDE);
}

static void modbus_uart_rx_dma_stop(pstUartDevice l_pstDev)
{
  const stUartContext *l_pstContext = UART_CONTEXT(l_pstDev);
  dma_channel_enum l_enRxChannel = RX_DMA_HANDLE(l_pstDev);

  usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_IDLE);
  usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_ERR);
  usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_PERR);

  /* On GD32F527, a software clear of CHEN also produces FTF after the
   * in-flight transfer/FIFO drain completes. Mask the RX DMA sources first so
   * an IDLE-triggered stop cannot be mistaken for a full receive. */
  modbus_uart_rx_dma_interrupts_disable(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);
  usart_dma_receive_config(UART_HANDLE(l_pstDev), USART_DENR_DISABLE);
  dma_channel_disable(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);

  /* CHEN can remain set briefly until the current access and FIFO drain have
   * completed. Clear flags only after hardware has finished the stop. */
  while ((DMA_CHCTL(DMA_PERIPHERAL(l_pstDev), l_enRxChannel) & DMA_CHXCTL_CHEN) != 0U)
  {
  }
  modbus_uart_dma_flags_clear(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);
  __DSB();

  NVIC_ClearPendingIRQ(l_pstContext->m_enRxDmaIrq);
  __DSB();
}

static void modbus_uart_rx_dma_restart(pstUartDevice l_pstDev)
{
  dma_channel_enum l_enRxChannel = RX_DMA_HANDLE(l_pstDev);

  modbus_uart_rx_dma_stop(l_pstDev);
  modbus_uart_clear_rx_status(UART_HANDLE(l_pstDev));
  dma_memory_address_config(DMA_PERIPHERAL(l_pstDev), l_enRxChannel, DMA_MEMORY_0, (uint32_t)l_pstDev->m_pu8RecvBuff);
  dma_transfer_number_config(DMA_PERIPHERAL(l_pstDev), l_enRxChannel, l_pstDev->m_u16RecvBuffSize);
  modbus_uart_dma_flags_clear(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);
  modbus_uart_rx_dma_interrupts_enable(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);
  dma_channel_enable(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);
  usart_dma_receive_config(UART_HANDLE(l_pstDev), USART_DENR_ENABLE);
  usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_INT_ERR);
  usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_INT_PERR);
  usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_INT_IDLE);
}

static TickType_t modbus_uart_interval_ticks(uint32_t l_u32IntervalUs)
{
  uint32_t l_u32IntervalMs = (l_u32IntervalUs + 999U) / 1000U;
  TickType_t l_xTicks = pdMS_TO_TICKS(l_u32IntervalMs);

  /* A delay started just before the next OS tick can be almost one tick
   * shorter in wall-clock time.  Keep one extra tick so t3.5 is never cut
   * short by scheduler quantization. */
  return l_xTicks + 1U;
}

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
  return (l_xTicks == 0U) ? 1U : l_xTicks;
}

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

static int modbus_uart_init(pstUartDevice l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit)
{
  const stUartContext *l_pstContext;
  dma_single_data_parameter_struct l_stDmaInit;
  uint32_t l_u32WordLength;
  uint32_t l_u32Parity;
  uint32_t l_u32CharUs;
  uint32_t l_u32FullGuardUs;

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
  l_pstContext = UART_CONTEXT(l_pstDev);

  l_pstDev->m_u8Initialized = 0;
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
    errno = ENOMEM;
    return -1;
  }
  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvSendSem, 0);
  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, 0);

  nvic_irq_disable(l_pstContext->m_enUsartIrq);
  nvic_irq_disable(l_pstContext->m_enTxDmaIrq);
  nvic_irq_disable(l_pstContext->m_enRxDmaIrq);
  NVIC_ClearPendingIRQ(l_pstContext->m_enUsartIrq);
  NVIC_ClearPendingIRQ(l_pstContext->m_enTxDmaIrq);
  NVIC_ClearPendingIRQ(l_pstContext->m_enRxDmaIrq);

  rcu_periph_clock_enable(l_pstContext->m_enGpioClock);
  rcu_periph_clock_enable(l_pstContext->m_enUsartClock);
  rcu_periph_clock_enable(RCU_DMA0);

  gpio_bit_reset(l_pstContext->m_u32DirPort, l_pstContext->m_u32DirPin);
  gpio_mode_set(l_pstContext->m_u32DirPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, l_pstContext->m_u32DirPin);
  gpio_output_options_set(l_pstContext->m_u32DirPort, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                          l_pstContext->m_u32DirPin);

  gpio_af_set(l_pstContext->m_u32GpioPort, GPIO_AF_7, l_pstContext->m_u32TxPin | l_pstContext->m_u32RxPin);
  gpio_mode_set(l_pstContext->m_u32GpioPort, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                l_pstContext->m_u32TxPin | l_pstContext->m_u32RxPin);
  gpio_output_options_set(l_pstContext->m_u32GpioPort, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                          l_pstContext->m_u32TxPin | l_pstContext->m_u32RxPin);

  l_u32WordLength = ((l_i32DataBit + ((l_i8Parity == 'N') ? 0 : 1)) > 8) ? USART_WL_9BIT : USART_WL_8BIT;
  l_u32Parity = (l_i8Parity == 'E') ? USART_PM_EVEN : ((l_i8Parity == 'O') ? USART_PM_ODD : USART_PM_NONE);

  usart_deinit(l_pstContext->m_u32UsartPeripheral);
  usart_baudrate_set(l_pstContext->m_u32UsartPeripheral, (uint32_t)l_i32Baud);
  usart_word_length_set(l_pstContext->m_u32UsartPeripheral, l_u32WordLength);
  usart_stop_bit_set(l_pstContext->m_u32UsartPeripheral, (l_i32StopBit == 1) ? USART_STB_1BIT : USART_STB_2BIT);
  usart_parity_config(l_pstContext->m_u32UsartPeripheral, l_u32Parity);
  usart_oversample_config(l_pstContext->m_u32UsartPeripheral, USART_OVSMOD_16);
  usart_hardware_flow_rts_config(l_pstContext->m_u32UsartPeripheral, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(l_pstContext->m_u32UsartPeripheral, USART_CTS_DISABLE);
  usart_transmit_config(l_pstContext->m_u32UsartPeripheral, USART_TRANSMIT_ENABLE);
  usart_receive_config(l_pstContext->m_u32UsartPeripheral, USART_RECEIVE_ENABLE);
  usart_enable(l_pstContext->m_u32UsartPeripheral);

  usart_interrupt_disable(l_pstContext->m_u32UsartPeripheral, USART_INT_IDLE);
  usart_interrupt_disable(l_pstContext->m_u32UsartPeripheral, USART_INT_TC);
  usart_interrupt_disable(l_pstContext->m_u32UsartPeripheral, USART_INT_ERR);
  usart_interrupt_disable(l_pstContext->m_u32UsartPeripheral, USART_INT_PERR);
  usart_dma_receive_config(l_pstContext->m_u32UsartPeripheral, USART_DENR_DISABLE);
  usart_dma_transmit_config(l_pstContext->m_u32UsartPeripheral, USART_DENT_DISABLE);
  modbus_uart_clear_rx_status(l_pstContext->m_u32UsartPeripheral);
  usart_flag_clear(l_pstContext->m_u32UsartPeripheral, USART_FLAG_TC);

  dma_deinit(DMA0, l_pstContext->m_enRxDmaChannel);
  dma_single_data_para_struct_init(&l_stDmaInit);
  l_stDmaInit.periph_addr = (uint32_t)&USART_DATA(l_pstContext->m_u32UsartPeripheral);
  l_stDmaInit.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  l_stDmaInit.memory0_addr = (uint32_t)l_pstDev->m_pu8RecvBuff;
  l_stDmaInit.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  l_stDmaInit.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
  l_stDmaInit.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
  l_stDmaInit.direction = DMA_PERIPH_TO_MEMORY;
  l_stDmaInit.number = l_pstDev->m_u16RecvBuffSize;
  l_stDmaInit.priority = DMA_PRIORITY_MEDIUM;
  dma_single_data_mode_init(DMA0, l_pstContext->m_enRxDmaChannel, &l_stDmaInit);
  dma_channel_subperipheral_select(DMA0, l_pstContext->m_enRxDmaChannel, DMA_SUBPERI4);

  modbus_uart_rx_dma_interrupts_disable(DMA0, l_pstContext->m_enRxDmaChannel);
  dma_channel_disable(DMA0, l_pstContext->m_enRxDmaChannel);
  modbus_uart_dma_flags_clear(DMA0, l_pstContext->m_enRxDmaChannel);

  dma_deinit(DMA0, l_pstContext->m_enTxDmaChannel);
  dma_single_data_para_struct_init(&l_stDmaInit);
  l_stDmaInit.periph_addr = (uint32_t)&USART_DATA(l_pstContext->m_u32UsartPeripheral);
  l_stDmaInit.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  l_stDmaInit.memory0_addr = 0U;
  l_stDmaInit.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  l_stDmaInit.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
  l_stDmaInit.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
  l_stDmaInit.direction = DMA_MEMORY_TO_PERIPH;
  l_stDmaInit.number = 0U;
  l_stDmaInit.priority = DMA_PRIORITY_HIGH;
  dma_single_data_mode_init(DMA0, l_pstContext->m_enTxDmaChannel, &l_stDmaInit);
  dma_channel_subperipheral_select(DMA0, l_pstContext->m_enTxDmaChannel, DMA_SUBPERI4);
  dma_interrupt_enable(DMA0, l_pstContext->m_enTxDmaChannel, DMA_INT_FTF);
  dma_interrupt_enable(DMA0, l_pstContext->m_enTxDmaChannel, DMA_INT_TAE);
  dma_interrupt_enable(DMA0, l_pstContext->m_enTxDmaChannel, DMA_INT_SDE);
  dma_channel_disable(DMA0, l_pstContext->m_enTxDmaChannel);
  modbus_uart_dma_flags_clear(DMA0, l_pstContext->m_enTxDmaChannel);

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
  modbus_uart_rx_dma_restart(l_pstDev);

  NVIC_ClearPendingIRQ(l_pstContext->m_enUsartIrq);
  NVIC_ClearPendingIRQ(l_pstContext->m_enTxDmaIrq);
  NVIC_ClearPendingIRQ(l_pstContext->m_enRxDmaIrq);
  nvic_irq_enable(l_pstContext->m_enTxDmaIrq, MODBUS_DMA_IRQ_PREEMPT_PRIORITY, 0U);
  nvic_irq_enable(l_pstContext->m_enRxDmaIrq, MODBUS_DMA_IRQ_PREEMPT_PRIORITY, 0U);
  nvic_irq_enable(l_pstContext->m_enUsartIrq, MODBUS_USART_IRQ_PREEMPT_PRIORITY, 0U);
  return 0;
}

static int modbus_uart_send(pstUartDevice l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len,
                            unsigned int l_u32Timeout)
{
  TickType_t l_xTimeoutTicks;
  unsigned char l_u8TxError;

  if ((l_pstDev == NULL) || (l_pu8Datas == NULL) || (l_u32Len == 0U) || (l_u32Len > 0xFFFFU) ||
      !l_pstDev->m_u8Initialized || (l_pstDev->m_pvSendSem == NULL))
  {
    errno = EINVAL;
    return -1;
  }

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
  modbus_uart_wait_rtu_guard(l_pstDev);

  taskENTER_CRITICAL();
  modbus_uart_rx_dma_stop(l_pstDev);
  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_TX);

  usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_TC);
  usart_dma_transmit_config(UART_HANDLE(l_pstDev), USART_DENT_DISABLE);
  dma_channel_disable(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev));
  modbus_uart_dma_flags_clear(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev));
  usart_flag_clear(UART_HANDLE(l_pstDev), USART_FLAG_TC);
  dma_memory_address_config(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev), DMA_MEMORY_0,
                            (uint32_t)l_pu8Datas);
  dma_transfer_number_config(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev), l_u32Len);
  dma_channel_enable(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev));
  usart_dma_transmit_config(UART_HANDLE(l_pstDev), USART_DENT_ENABLE);
  taskEXIT_CRITICAL();

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

static void modbus_uart_tx_abort(pstUartDevice l_pstDev)
{
  taskENTER_CRITICAL();
  usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_TC);
  usart_dma_transmit_config(UART_HANDLE(l_pstDev), USART_DENT_DISABLE);
  dma_channel_disable(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev));
  modbus_uart_dma_flags_clear(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev));
  usart_flag_clear(UART_HANDLE(l_pstDev), USART_FLAG_TC);

  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  modbus_uart_rx_dma_restart(l_pstDev);
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCount() + l_pstDev->m_u32FullFrameGapTicks;
  l_pstDev->m_u8TxGapLocked = 1;
  l_pstDev->m_u8TxBusy = 0;
  taskEXIT_CRITICAL();
}

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
      taskENTER_CRITICAL();
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u8RecvDone = 0;
      modbus_uart_rx_dma_restart(l_pstDev);
      taskEXIT_CRITICAL();
      errno = EMSGSIZE;
      return -1;
    }

    memcpy(l_pu8Datas, l_pstDev->m_pu8RecvBuff + l_pstDev->m_u16ReadLen, l_u32Len);
    l_pstDev->m_u16ReadLen += (unsigned short)l_u32Len;
    if (l_pstDev->m_u16ReadLen >= l_pstDev->m_u16RecvLen)
    {
      taskENTER_CRITICAL();
      l_pstDev->m_u16ReadLen = 0;
      l_pstDev->m_u16RecvLen = 0;
      l_pstDev->m_u8RecvDone = 0;
      modbus_uart_rx_dma_restart(l_pstDev);
      taskEXIT_CRITICAL();
    }
    return (int)l_u32Len;
  }

  if (l_u32Timeout == MODBUS_MAX_DELAY)
  {
    l_xTimeoutTicks = portMAX_DELAY;
  }
  else
  {
    l_xTimeoutTicks = pdMS_TO_TICKS(l_u32Timeout);
    if (l_xTimeoutTicks == 0U)
    {
      l_xTimeoutTicks = 1U;
    }
  }

  if (xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, l_xTimeoutTicks) == pdTRUE)
  {
    l_pstDev->m_u8RecvDone = 1;
    l_pstDev->m_u16ReadLen = 0;
    return 0;
  }

  taskENTER_CRITICAL();
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, 0);
  modbus_uart_rx_dma_restart(l_pstDev);
  taskEXIT_CRITICAL();
  errno = ETIMEDOUT;
  return -1;
}

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

  taskENTER_CRITICAL();
  modbus_uart_rx_dma_stop(l_pstDev);
  memset(l_pstDev->m_pu8RecvBuff, 0, l_pstDev->m_u16RecvBuffSize);
  l_pstDev->m_pFnDirSwitch(l_pstDev, MODBUS_DIR_RX);
  l_pstDev->m_u16ReadLen = 0;
  l_pstDev->m_u16RecvLen = 0;
  l_pstDev->m_u8RecvDone = 0;
  (void)xSemaphoreTake((pstModbusSem_t)l_pstDev->m_pvRecvSem, 0);
  modbus_uart_rx_dma_restart(l_pstDev);
  taskEXIT_CRITICAL();
  return 0;
}

static void modbus_rs485_dir_switch(pstUartDevice l_pstDev, enModbusDir_t l_enDir)
{
  const stUartContext *l_pstContext;

  if ((l_pstDev == NULL) || (l_pstDev->m_pvPortContext == NULL))
  {
    return;
  }
  l_pstContext = UART_CONTEXT(l_pstDev);
  if (l_enDir == MODBUS_DIR_TX)
  {
    gpio_bit_set(l_pstContext->m_u32DirPort, l_pstContext->m_u32DirPin);
  }
  else
  {
    gpio_bit_reset(l_pstContext->m_u32DirPort, l_pstContext->m_u32DirPin);
  }
}

static void modbus_uart_tx_complete_from_isr(pstUartDevice l_pstDev, BaseType_t *l_pxHigherPriorityTaskWoken)
{
  usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_TC);
  usart_flag_clear(UART_HANDLE(l_pstDev), USART_FLAG_TC);
  usart_dma_transmit_config(UART_HANDLE(l_pstDev), USART_DENT_DISABLE);
  dma_channel_disable(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev));
  modbus_uart_dma_flags_clear(DMA_PERIPHERAL(l_pstDev), TX_DMA_HANDLE(l_pstDev));

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
    (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvSendSem, l_pxHigherPriorityTaskWoken);
  }
}

static unsigned char modbus_uart_rx_error_pending(pstUartDevice l_pstDev)
{
  uint32_t l_u32Usart = UART_HANDLE(l_pstDev);

  return (unsigned char)((usart_interrupt_flag_get(l_u32Usart, USART_INT_FLAG_ERR_ORERR) != RESET) ||
                         (usart_interrupt_flag_get(l_u32Usart, USART_INT_FLAG_ERR_NERR) != RESET) ||
                         (usart_interrupt_flag_get(l_u32Usart, USART_INT_FLAG_ERR_FERR) != RESET) ||
                         (usart_interrupt_flag_get(l_u32Usart, USART_INT_FLAG_PERR) != RESET));
}

static void modbus_uart_rx_error_from_isr(pstUartDevice l_pstDev)
{
  l_pstDev->m_u16ReadLen = 0U;
  l_pstDev->m_u16RecvLen = 0U;
  l_pstDev->m_u8RecvDone = 0U;
  modbus_uart_rx_dma_restart(l_pstDev);
}

static void modbus_uart_rx_idle_from_isr(pstUartDevice l_pstDev, BaseType_t *l_pxHigherPriorityTaskWoken)
{
  dma_channel_enum l_enRxChannel = RX_DMA_HANDLE(l_pstDev);
  unsigned short l_u16Remaining;
  unsigned short l_u16Received;

  l_u16Remaining = (unsigned short)dma_transfer_number_get(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);
  modbus_uart_clear_rx_status(UART_HANDLE(l_pstDev));

  l_u16Received = l_pstDev->m_u16RecvBuffSize - l_u16Remaining;
  if (l_u16Received == 0U)
  {
    return;
  }

  modbus_uart_rx_dma_stop(l_pstDev);
  l_pstDev->m_u16RecvLen = l_u16Received;
  l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32RxToTxRemainTicks;
  l_pstDev->m_u8TxGapLocked = 1;
  if (l_pstDev->m_pvRecvSem != NULL)
  {
    (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvRecvSem, l_pxHigherPriorityTaskWoken);
  }
}

static void modbus_uart_irq_handler(pstUartDevice l_pstDev)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }

  if (usart_interrupt_flag_get(UART_HANDLE(l_pstDev), USART_INT_FLAG_TC) != RESET)
  {
    if (l_pstDev->m_u8TxBusy)
    {
      modbus_uart_tx_complete_from_isr(l_pstDev, &l_xHigherPriorityTaskWoken);
    }
    else
    {
      usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_TC);
      usart_flag_clear(UART_HANDLE(l_pstDev), USART_FLAG_TC);
    }
  }

  if (modbus_uart_rx_error_pending(l_pstDev))
  {
    modbus_uart_rx_error_from_isr(l_pstDev);
  }

  if (usart_interrupt_flag_get(UART_HANDLE(l_pstDev), USART_INT_FLAG_IDLE) != RESET)
  {
    modbus_uart_rx_idle_from_isr(l_pstDev, &l_xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

static unsigned char modbus_uart_dma_error_pending(uint32_t l_u32Dma, dma_channel_enum l_enChannel)
{
  return (unsigned char)((dma_interrupt_flag_get(l_u32Dma, l_enChannel, DMA_INT_FLAG_TAE) != RESET) ||
                         (dma_interrupt_flag_get(l_u32Dma, l_enChannel, DMA_INT_FLAG_SDE) != RESET));
}

static void modbus_uart_tx_dma_irq_handler(pstUartDevice l_pstDev)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  dma_channel_enum l_enTxChannel;

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }
  l_enTxChannel = TX_DMA_HANDLE(l_pstDev);

  if (modbus_uart_dma_error_pending(DMA_PERIPHERAL(l_pstDev), l_enTxChannel))
  {
    dma_channel_disable(DMA_PERIPHERAL(l_pstDev), l_enTxChannel);
    usart_dma_transmit_config(UART_HANDLE(l_pstDev), USART_DENT_DISABLE);
    usart_interrupt_disable(UART_HANDLE(l_pstDev), USART_INT_TC);
    modbus_uart_dma_flags_clear(DMA_PERIPHERAL(l_pstDev), l_enTxChannel);
    usart_flag_clear(UART_HANDLE(l_pstDev), USART_FLAG_TC);

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
  else if (dma_interrupt_flag_get(DMA_PERIPHERAL(l_pstDev), l_enTxChannel, DMA_INT_FLAG_FTF) != RESET)
  {
    dma_flag_clear(DMA_PERIPHERAL(l_pstDev), l_enTxChannel, DMA_FLAG_FTF);
    dma_channel_disable(DMA_PERIPHERAL(l_pstDev), l_enTxChannel);
    usart_dma_transmit_config(UART_HANDLE(l_pstDev), USART_DENT_DISABLE);

    /* DMA completion only means the last byte reached USART_DATA.  Wait for
     * USART TC before releasing DE and waking the sender. */
    if (l_pstDev->m_u8TxBusy)
    {
      usart_interrupt_enable(UART_HANDLE(l_pstDev), USART_INT_TC);
    }
  }

  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

static void modbus_uart_rx_dma_irq_handler(pstUartDevice l_pstDev)
{
  BaseType_t l_xHigherPriorityTaskWoken = pdFALSE;
  unsigned char l_u8DmaError;
  dma_channel_enum l_enRxChannel;

  if ((l_pstDev == NULL) || !l_pstDev->m_u8Initialized)
  {
    return;
  }
  l_enRxChannel = RX_DMA_HANDLE(l_pstDev);
  l_u8DmaError = modbus_uart_dma_error_pending(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);

  if (l_u8DmaError != 0U)
  {
    modbus_uart_rx_error_from_isr(l_pstDev);
  }
  else if (dma_interrupt_flag_get(DMA_PERIPHERAL(l_pstDev), l_enRxChannel, DMA_INT_FLAG_FTF) != RESET)
  {
    modbus_uart_rx_dma_stop(l_pstDev);
    l_pstDev->m_u16RecvLen = l_pstDev->m_u16RecvBuffSize;
    l_pstDev->m_u32NextTxAllowedTick = xTaskGetTickCountFromISR() + l_pstDev->m_u32FullFrameGapTicks;
    l_pstDev->m_u8TxGapLocked = 1;
    if (l_pstDev->m_pvRecvSem != NULL)
    {
      (void)xSemaphoreGiveFromISR((pstModbusSem_t)l_pstDev->m_pvRecvSem, &l_xHigherPriorityTaskWoken);
    }
  }
  else
  {
    /* Defensive acknowledgement for a stale/unexpected channel flag. */
    modbus_uart_dma_flags_clear(DMA_PERIPHERAL(l_pstDev), l_enRxChannel);
  }

  portYIELD_FROM_ISR(l_xHigherPriorityTaskWoken);
}

void USART1_IRQHandler(void)
{
  modbus_uart_irq_handler(sArM_pstUartDevs[MODBUS_UART1]);
}

void USART2_IRQHandler(void)
{
  modbus_uart_irq_handler(sArM_pstUartDevs[MODBUS_UART2]);
}

void DMA0_Channel6_IRQHandler(void)
{
  modbus_uart_tx_dma_irq_handler(sArM_pstUartDevs[MODBUS_UART1]);
}

void DMA0_Channel3_IRQHandler(void)
{
  modbus_uart_tx_dma_irq_handler(sArM_pstUartDevs[MODBUS_UART2]);
}

void DMA0_Channel5_IRQHandler(void)
{
  modbus_uart_rx_dma_irq_handler(sArM_pstUartDevs[MODBUS_UART1]);
}

void DMA0_Channel1_IRQHandler(void)
{
  modbus_uart_rx_dma_irq_handler(sArM_pstUartDevs[MODBUS_UART2]);
}
