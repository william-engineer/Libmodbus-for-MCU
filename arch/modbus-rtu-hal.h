/*******************************************************************
** Project Name :    Libmodbus for MCU                            **
********************************************************************
** File Name    :    modbus-rtu-hal.h                             **
** Version      :    V1.1.0                                       **
********************************************************************/
/*
 * Libmodbus for MCU - Version 1.1.0
 * Based on libmodbus 3.1.11
 *
 * Copyright © William Chen <william_engineer@outlook.com>
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#ifndef __MODBUS_RTU_HAL_H__
#define __MODBUS_RTU_HAL_H__

#include "modbus-port.h"

/**
 * @brief Modbus UART device ID.
 */
typedef enum
{
  MODBUS_UART1 = 0,
  MODBUS_UART2,
  MODBUS_UART3,
  MODBUS_UART_COUNT
} enModbusUartId_t;

/**
 * @brief Modbus device direction enumeration.
 */
typedef enum enModbusDir
{
  MODBUS_DIR_RX = 0,
  MODBUS_DIR_TX = 1,
} enModbusDir_t;

/**
 * @brief Common Modbus UART device state.
 *
 * Hardware-specific state, such as UART and DMA handles, belongs to the object
 * referenced by m_pvPortContext. Its concrete type is private to the selected
 * board port.
 */
typedef struct _UartDevice
{
  const char *m_pi8Name;      /**< UART device name used by modbus_new_rtu() */
  enModbusUartId_t m_enDevId; /**< Modbus UART device ID */

  unsigned char *m_pu8RecvBuff;           /**< UART receive buffer */
  unsigned short m_u16RecvBuffSize;       /**< UART receive buffer size */
  volatile unsigned short m_u16RecvLen;   /**< received Modbus frame length */
  volatile unsigned short m_u16ReadLen;   /**< received data already consumed */
  volatile unsigned char m_u8RecvDone;    /**< receive frame completion flag */
  volatile unsigned char m_u8Initialized; /**< UART, DMA and synchronization are ready */

  unsigned int m_u32Baud;                     /**< configured baud rate */
  unsigned char m_u8BitsPerChar;              /**< start + data + parity + stop bits */
  volatile unsigned char m_u8TxBusy;          /**< transmission is in progress */
  volatile unsigned char m_u8TxError;         /**< current transmission failed */
  volatile unsigned int m_u32TxCompleteCount; /**< completed RTU frames */
  volatile unsigned int m_u32TxTimeoutCount;  /**< recovered TX timeouts */
  volatile unsigned int m_u32TxDmaErrorCount; /**< recovered TX DMA errors */

  unsigned int m_u32RxToTxRemainTicks;          /**< remaining frame-gap ticks after UART IDLE */
  unsigned int m_u32FullFrameGapTicks;          /**< full RTU t3.5 interval in OS ticks */
  volatile unsigned int m_u32NextTxAllowedTick; /**< earliest tick for the next frame */
  volatile unsigned char m_u8TxGapLocked;       /**< next TX waits until the allowed tick */

  void *m_pvRecvSem;     /**< semaphore for receive completion */
  void *m_pvSendSem;     /**< semaphore signalled after the final stop bit */
  void *m_pvPortContext; /**< board-port-private hardware context */

  int (*m_pFnInit)(struct _UartDevice *l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit);
  int (*m_pFnSend)(struct _UartDevice *l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len, unsigned int l_u32Timeout);
  int (*m_pFnRecv)(struct _UartDevice *l_pstDev, unsigned char *l_pu8Datas, unsigned int l_u32Len, unsigned int l_u32Timeout);
  int (*m_pFnFlush)(struct _UartDevice *l_pstDev);
  void (*m_pFnDirSwitch)(struct _UartDevice *l_pstDev, enModbusDir_t l_enDir);
} stUartDevice, *pstUartDevice;

pstUartDevice find_uart_device(const char *l_pi8Name);

#endif /* __MODBUS_RTU_HAL_H__ */
