/*******************************************************************
** Project Name :   Libmodbus for MCU                             **
********************************************************************
** Product Name :                                                 **
**                                                                **
** File Name    :    modbus-port.h                                **
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

#ifndef __MODBUS_PORT_H__
#define __MODBUS_PORT_H__

#ifdef __MCU__
#include "modbus-version.h"
#include "string.h"

/* Platform-specific UART and DMA handle type mapping */
#ifdef STM32F429xx
    #include "stm32f4xx_hal.h"
    typedef UART_HandleTypeDef   stUartHalHandle_t;
    typedef DMA_HandleTypeDef    stUartTxDmaHandle_t;
    typedef DMA_HandleTypeDef    stUartRxDmaHandle_t;
#elif defined(YOUR_MCU)
    #include "xxx.h"
    typedef USART_Type        stUartHalHandle_t;
    typedef DMA_Channel_Type  stUartTxDmaHandle_t;
    typedef DMA_Channel_Type  stUartRxDmaHandle_t;
#else
    #error "Unsupported platform. Please define UART/DMA handle types for your MCU."
#endif

/* Redirect fprintf calls to printf to avoid file stream operations */
#define modbus_printf(...) printf(__VA_ARGS__)

/* Define auxiliary macros */
//#define ssize_t unsigned int
#define assert(expr) ((void)0)

/* Detect conflicting or missing OS macro definitions */
#if (defined(USING_OS_TYPE_FREERTOS) + defined(USING_OS_TYPE_RTTHREAD) + defined(USING_OS_TYPE_CMSIS_RTOS2)) > 1
    #error "Multiple OS types defined! Only one of USING_OS_TYPE_FREERTOS / RTTHREAD / CMSIS_RTOS2 should be defined."
#elif !(defined(USING_OS_TYPE_FREERTOS) || defined(USING_OS_TYPE_RTTHREAD) || defined(USING_OS_TYPE_CMSIS_RTOS2))
    #error "No OS type defined. You must define one of USING_OS_TYPE_FREERTOS / RTTHREAD / CMSIS_RTOS2"
#else
    #ifdef USING_OS_TYPE_FREERTOS
        #include "FreeRTOS.h"
        #include "task.h"
        #include "semphr.h"
        #include "queue.h"
        typedef QueueHandle_t pstModbusSem_t, pstModbusQueue_t;
        #define MODBUS_MAX_DELAY portMAX_DELAY
        #define os_delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))
    #elif defined(USING_OS_TYPE_RTTHREAD)
        #include <rtthread.h>
        #define os_delay_ms(ms) rt_thread_mdelay(ms)
    #elif defined(USING_OS_TYPE_CMSIS_RTOS2)
        #include "cmsis_os2.h"
        #define os_delay_ms(ms) osDelay(ms)
    #endif
#endif

/* Check only one network stack is selected */
#if (defined(USING_NET_STACK_LWIP) + defined(USING_NET_STACK_UCIP) + defined(USING_NET_STACK_BSD)) > 1
    #error "Multiple network stacks defined! Only one of USING_NET_STACK_LWIP / uC/TCP-IP / BSD should be defined."
#elif !(defined(USING_NET_STACK_LWIP) || defined(USING_NET_STACK_UCIP) || defined(USING_NET_STACK_BSD))
    #error "No network stack defined. You must define one of USING_NET_STACK_LWIP / uC/TCP-IP / BSD"
#else
    #ifdef USING_NET_STACK_LWIP
        #include "lwip/errno.h"
        #include <lwip/sockets.h>
        #include "lwipopts.h"
        /* Check necessary LWIP configuration options */
        #if !defined(LWIP_SOCKET)
            #error "LWIP_SOCKET is not defined. Please set it to 1 in lwipopts.h"
        #elif LWIP_SOCKET != 1
            #error "LWIP_SOCKET must be set to 1 to enable BSD-style socket API."
        #endif

        #if !defined(LWIP_COMPAT_SOCKETS)
            #error "LWIP_COMPAT_SOCKETS is not defined. Please set it to 0 in lwipopts.h"
        #elif LWIP_COMPAT_SOCKETS != 0
            #error "LWIP_COMPAT_SOCKETS must be set to 0 to enable lwIP socket API mode."
        #endif

        #if !defined(LWIP_SOCKET_SELECT)
            #error "LWIP_SOCKET_SELECT is not defined. Please set it to 1 in lwipopts.h"
        #elif LWIP_SOCKET_SELECT != 1
            #error "LWIP_SOCKET_SELECT must be set to 1 to use select() with sockets."
        #endif

        #if !defined(LWIP_POSIX_SOCKETS_IO_NAMES)
            #error "LWIP_POSIX_SOCKETS_IO_NAMES is not defined. Please set it to 1 in lwipopts.h"
        #elif LWIP_POSIX_SOCKETS_IO_NAMES != 1
            #error "LWIP_POSIX_SOCKETS_IO_NAMES must be set to 1 to use read()/write()/close() naming."
        #endif

        #if !defined(SO_REUSE)
            #error "SO_REUSE is not defined. Please set it to 1 in lwipopts.h"
        #elif SO_REUSE != 1
            #error "SO_REUSE must be set to 1 to allow port reuse on socket reconnects."
        #endif

        #if !defined(LWIP_NETCONN)
            #error "LWIP_NETCONN is not defined. Please set it to 1 in lwipopts.h"
        #elif LWIP_NETCONN != 1
            #error "LWIP_NETCONN must be set to 1 for proper socket API functionality."
        #endif

        /* Map lwIP socket functions to Modbus network interface for compatibility */
        #define modbus_net_send lwip_send
        #define modbus_net_recv lwip_recv
        #define modbus_net_close lwip_close
        #define modbus_net_socket lwip_socket
        #define modbus_net_bind lwip_bind
        #define modbus_net_listen lwip_listen
        #define modbus_net_accept lwip_accept
        #define modbus_net_setsockopt lwip_setsockopt
        #define modbus_net_getsockopt lwip_getsockopt
        #define modbus_net_ioctl lwip_ioctl
        #define modbus_net_connect lwip_connect
        #define modbus_net_select lwip_select
        #define modbus_net_shutdown lwip_shutdown
        #define modbus_net_inet_pton lwip_inet_pton
        #define modbus_net_inet_ntop lwip_inet_ntop
    #elif defined(USING_NET_STACK_UCIP)
        /* Future implementation: Add uC/TCP-IP stack support */
        #error "uC/TCP-IP support is not yet implemented."
    #elif defined(USING_NET_STACK_BSD)
        /* Future implementation: Add BSD socket support for non-RTOS targets */
        #error "BSD socket support is not yet implemented."
    #endif
#endif

#endif // __MCU__
#endif //__MODBUS_PORT_H__
