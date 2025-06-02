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
#ifndef __MODBUS_RTU_HAL_H__
#define __MODBUS_RTU_HAL_H__
#include "modbus-port.h"

/**
 * @brief UART device ID enumeration
 */
enum enDevId
{
	UART1 = 0,
	UART2,
	UART3,
	MAX_UART_DEV_ID
};

/**
 * @brief Modbus device direction enumeration
 */
typedef enum enModbusDir
{
  MODBUS_DIR_RX = 0,
  MODBUS_DIR_TX = 1,
} enModbusDir_t;

/**
 * @brief UART device structure
 */
typedef struct _UartDevice
{
	/* UART device-specific attributes and configurations */
	char *m_pi8Name; /**< uart device name */
	enum enDevId m_enDevId; /**< uart device id */
	stUartHalHandle_t m_stHalHandle; /**< uart handle */
	stUartTxDmaHandle_t m_stTxDmaHandle; /**< uart tx dma handle */
	stUartRxDmaHandle_t m_stRxDmaHandle; /**< uart rx dma handle */
	unsigned char *m_pu8RecvBuff; /**< uart data receive buff pointer */
	unsigned short m_u16RecvBuffSize; /**< uart data receive buff size */
	unsigned short m_u16RecvLen; /**< length of Modbus frame received via UART */
	unsigned short m_u16ReadLen; /**< length of Modbus data that has been processed */
	unsigned char m_u8RecvDone; /**< uart data receive done flag */
	void* m_pvRecvSem; /**< semaphore for receive completion */
	void* m_pvSendSem; /**< semaphore for send completion */

	/* uart device-specific driver interface */
  int (*m_pFnInit)(struct _UartDevice *l_pstDev, int l_i32Baud, char l_i8Parity, int l_i32DataBit, int l_i32StopBit);
  int (*m_pFnSend)( struct _UartDevice *l_pstDev, unsigned char * l_pu8Datas, unsigned int l_u32Len, unsigned int l_u32Timeout);
	int (*m_pFnRecv)( struct _UartDevice *l_pstDev, unsigned char * l_pu8Datas, unsigned int l_u32Len,unsigned int l_u32Timeout);
	int (*m_pFnFlush)(struct _UartDevice *l_pstDev);
	void (*m_pFnDirSwitch)(struct _UartDevice *l_pstDev, enModbusDir_t l_enDir);
}stUartDevice, *pstUartDevice;

pstUartDevice find_uart_device(char *l_pi8Name);

#endif /* __MODBUS_RTU_HAL_H__ */
