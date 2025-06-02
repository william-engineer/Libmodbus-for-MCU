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

#include "lwip_comm.h"
#include "lwipopts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modbus.h"
#include "lwip/errno.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <lwip/sockets.h>


/**
 * FreeRTOS task configuration
 */

#define START_TASK_PRIO         1     /* task priority */
#define START_STK_SIZE          256         /* task stack size */
TaskHandle_t StartTask_Handler;             /* task handle */
void start_task(void *pvParameters);        /* task function */


#define MODBUS_RTU_MASTER_TASK_PRIO     9          /* task priority */
#define MODBUS_RTU_MASTER_TASK_SIZE      512        /* task stack size */
TaskHandle_t g_pstModbusRtuMasterHandler;             /* task handle */
void modbus_rtu_master_task(void *pvParameters);    /* task function */

#define MODBUS_RTU_SLAVE_TASK_PRIO     11          /* task priority */
#define MODBUS_RTU_SLAVE_TASK_SIZE      512        /* task stack size */
TaskHandle_t g_pstModbusRtuSlaveHandler;             /* task handle */
void modbus_rtu_slave_task(void *pvParameters);    /* task function */

#define MODBUS_TCP_MASTER_TASK_PRIO     7          /* task priority */
#define MODBUS_TCP_MASTER_TASK_SIZE      512        /* task stack size */
TaskHandle_t g_pstModbusTcpMasterHandler;             /* task handle */
void modbus_tcp_master_task(void *pvParameters);    /* task function */

#define MODBUS_TCP_SLAVE_TASK_PRIO     8          /* task priority */
#define MODBUS_TCP_SLAVE_TASK_SIZE      512        /* task stack size */
TaskHandle_t g_pstModbusTcpSlaveHandler;             /* task handle */
void modbus_tcp_slave_task(void *pvParameters);    /* task function */

/**
 * @brief create start task
 * @param  none
 * @retval none
 */
void modbus_test(void)
{
  /* create start task */
  xTaskCreate((TaskFunction_t )start_task,
              (const char *   )"start_task",
              (uint16_t       )START_STK_SIZE,
              (void *         )NULL,
              (UBaseType_t    )START_TASK_PRIO,
              (TaskHandle_t * )&StartTask_Handler);
  /* start task scheduler */
  vTaskStartScheduler(); 
}

/**
 * @brief  create modbus test task
 * @param  pvParameters: task parameters
 * @retval none
 */
void start_task(void *pvParameters)
{
  pvParameters = pvParameters;

  /* initialize lwIP stack */
  while (lwip_comm_init() != 0)
  {
    delay_ms(500);
  }

  /* wait for DHCP completion */
  while (g_lwipdev.dhcpstatus != 2 && g_lwipdev.dhcpstatus != 0xff)
  {
    vTaskDelay(500);
  }

  /* enter critical section */
  taskENTER_CRITICAL(); 

  xTaskCreate((TaskFunction_t)modbus_rtu_master_task,
              (const char *)"modbus_rtu_master_task",
              (uint16_t)MODBUS_RTU_MASTER_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)MODBUS_RTU_MASTER_TASK_PRIO,
              (TaskHandle_t *)&g_pstModbusRtuMasterHandler);

  xTaskCreate((TaskFunction_t)modbus_rtu_slave_task,
              (const char *)"modbus_rtu_slave_task",
              (uint16_t)MODBUS_RTU_SLAVE_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)MODBUS_RTU_SLAVE_TASK_PRIO,
              (TaskHandle_t *)&g_pstModbusRtuSlaveHandler);

  xTaskCreate((TaskFunction_t)modbus_tcp_master_task,
              (const char *)"modbus_tcp_master_task",
              (uint16_t)MODBUS_TCP_MASTER_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)MODBUS_TCP_MASTER_TASK_PRIO,
              (TaskHandle_t *)&g_pstModbusTcpMasterHandler);

  xTaskCreate((TaskFunction_t)modbus_tcp_slave_task,
              (const char *)"modbus_tcp_slave_task",
              (uint16_t)MODBUS_TCP_SLAVE_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)MODBUS_TCP_SLAVE_TASK_PRIO,
              (TaskHandle_t *)&g_pstModbusTcpSlaveHandler);

  /* delete start task */
  vTaskDelete(StartTask_Handler);

  /* exit critical section */
  taskEXIT_CRITICAL();
}

/**
 * @brief  modbus rtu master task
 * @param  pvParameters: task parameters
 * @retval none
 */
void modbus_rtu_master_task(void *pvParameters)
{
  pvParameters = pvParameters;
  uint16_t holding_regs[10] = {0};
  uint16_t input_regs[10] = {0};
  uint8_t coils[10] = {0};
  uint8_t discrete_inputs[10] = {0};
  int rc;
  modbus_t *ctx;

  /* create new RTU context */
  ctx = modbus_new_rtu("uart3", 115200, 'N', 8, 1);

  /* connect to RTU device */
  if (modbus_connect(ctx) == -1)
  {
    printf("Connection failed: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
  }

  /* set slave ID */
  modbus_set_slave(ctx, 2);

  while (1)
  {
    /* Read Holding Registers (0x03) */
    rc = modbus_read_registers(ctx, 0, 10, holding_regs);
    if (rc == -1)
    {
      printf("RTU Read Holding Registers failed: %s\n", modbus_strerror(errno));
    }
    else
    {
      /* Increment and write back */
      for (int i = 0; i < 10; i++)
        holding_regs[i] += 1;
      modbus_write_registers(ctx, 0, 10, holding_regs);
    }

    /* Read Input Registers (0x04) */
    rc = modbus_read_input_registers(ctx, 0, 10, input_regs);
    if (rc == -1)
    {
      printf("RTU Read Input Registers failed: %s\n", modbus_strerror(errno));
    }
    else
    {
      printf("RTU Input Registers: ");
      for (int i = 0; i < 10; i++)
      {
        printf("%d ", input_regs[i]);
      }
      printf("\n");
    }

    /* Read Coils (0x01) */
    rc = modbus_read_bits(ctx, 0, 10, coils);
    if (rc == -1)
    {
      printf("RTU Read Coils failed: %s\n", modbus_strerror(errno));
    }
    else
    {
      /* Invert state and write back */
      for (int i = 0; i < 10; i++)
        coils[i] = !coils[i];
      modbus_write_bits(ctx, 0, 10, coils);
    }

    /* Read Discrete Inputs (0x02) */
    rc = modbus_read_input_bits(ctx, 0, 10, discrete_inputs);
    if (rc == -1)
    {
      printf("RTU Read Discrete Inputs failed: %s\n", modbus_strerror(errno));
    }
    else
    {
      printf("RTU Discrete Inputs: ");
      for (int i = 0; i < 10; i++)
      {
        printf("%d ", discrete_inputs[i]);
      }
      printf("\n");
    }
    /* Delay for a while */
    vTaskDelay(500);
  }
}

/**
 * @brief  modbus rtu slave task
 * @param  pvParameters: task parameters
 * @retval none
 */
void modbus_rtu_slave_task(void *pvParameters)
{
  modbus_t *ctx = NULL;
  modbus_mapping_t *mb_mapping = NULL;
  int rc;
  uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH]; // RTU frame buffer
  uint16_t index = 0;

  /* Initialize RTU context */
  ctx = modbus_new_rtu("uart2", 115200, 'N', 8, 1); 

  /* set slave ID */
  modbus_set_slave(ctx, 1);

  /* set response timeout */
  modbus_set_response_timeout(ctx, 1, 0); // 响应超时：1秒

  /* open serial connection */
  if (modbus_connect(ctx) == -1)
  {
    printf("modbus_rtu_slave_task Failed to connect: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
  }

  /* Create register address mapping: allocate 100 coils, 100 discrete inputs,
   * 100 holding registers, and 100 input registers */
  mb_mapping = modbus_mapping_new(100, 100, 100, 100);
  if (mb_mapping == NULL)
  {
    printf("modbus_rtu_slave_task Failed to allocate mapping\n");
    modbus_close(ctx);
    modbus_free(ctx);
  }

  /* Initialize register values (example) */
  mb_mapping->tab_registers[0] = 0x1234;       // Holding register address 40001 initial value
  mb_mapping->tab_input_registers[0] = 0x5678; // Input register address 30001 initial value
  mb_mapping->tab_bits[0] = 0x01;              // Coil address 00001 initial value (1 means ON)
  mb_mapping->tab_input_bits[0] = 0x00;        // Discrete input address 10001 initial value (0 means OFF)

  /* Main loop: listen for and process requests */
  while (1)
  {
    /* Receive Modbus request */
    rc = modbus_receive(ctx, query);
    if (rc > 0)
    {
      /* Process request and send response */
      modbus_reply(ctx, query, rc, mb_mapping);
    }
    else if (rc == -1)
    {
      /* Error handling */
      printf("modbus_rtu_slave_task Error: %s\n", modbus_strerror(errno));
    }

    /* dynamically update register values (simulate data changes) */
    mb_mapping->tab_registers[0] = index;           // Holding register address 40001
    mb_mapping->tab_input_registers[0] = index + 1; // Input register address 30001
    mb_mapping->tab_bits[1] = (index % 2) ? 1 : 0;  // Coil address 00002
    index++;
  }

  /* Clean up resources (usually won't reach here) */
  modbus_mapping_free(mb_mapping);
  modbus_close(ctx);
  modbus_free(ctx);
  return;
}

/**
 * @brief  modbus tcp master task
 * @param  pvParameters: task parameters
 * @retval none
 */
void modbus_tcp_master_task(void *pvParameters)
{
  modbus_t *ctx;
  uint16_t holding_regs[10] = {0};
  uint16_t input_regs[10] = {0};
  uint8_t coils[10] = {0};
  uint8_t discrete_inputs[10] = {0};
  int rc, retry_count = 0;
  enum
  {
    INIT,
    CONNECTING,
    CONNECTED,
    RECONNECT_WAIT,
    ERROR
  } state = INIT;


  while (1)
  {
    switch (state)
    {
      case INIT:
        printf("[INIT] Initializing Modbus TCP...\n");

        /* Create new Modbus TCP context */
        ctx = modbus_new_tcp("172.18.57.11", 6805);
        if (ctx == NULL)
        {
          printf("Unable to allocate libmodbus context\n");
          state = ERROR;
          break;
        }
        state = CONNECTING;
        break;

      case CONNECTING:
        printf("[CONNECTING] Trying to connect to %s:%d\n", "172.18.57.11", 6805);

        /* Set connection timeout and connect server */
        modbus_set_response_timeout(ctx, 2, 0);
        if (modbus_connect(ctx) == -1)
        {
          printf("TCP Client Connection failed: %s\n", modbus_strerror(errno));
          modbus_free(ctx);
          state = RECONNECT_WAIT;
        }
        else
        {
          printf("[CONNECTED] Connection established.\n");
          retry_count = 0;
          modbus_set_response_timeout(ctx, 0, 500000);
          state = CONNECTED;
        }
        break;

      case CONNECTED:
        /* Read Holding Registers (0x03) */
        rc = modbus_read_registers(ctx, 0, 10, holding_regs);
        if (rc == -1)
        {
          printf("TCP Read Holding Registers failed: %s\n", modbus_strerror(errno));
          retry_count++;
        }
        else
        {
          /* Increment and write back */
          for (int i = 0; i < 10; i++)
            holding_regs[i] += 1;
          modbus_write_registers(ctx, 0, 10, holding_regs);
          retry_count = 0;
        }

        /* Read Input Registers (0x04) */
        rc = modbus_read_input_registers(ctx, 0, 10, input_regs);
        if (rc == -1)
        {
          printf("TCP Read Input Registers failed: %s\n", modbus_strerror(errno));
          retry_count++;
        }
        else
        {
          printf("TCP Input Registers: ");
          for (int i = 0; i < 10; i++)
          {
            printf("%d ", input_regs[i]);
          }
          printf("\n");
          retry_count = 0;
        }

        /* Read Coils (0x01) */
        rc = modbus_read_bits(ctx, 0, 10, coils);
        if (rc == -1)
        {
          printf("TCP Read Coils failed: %s\n", modbus_strerror(errno));
          retry_count++;
        }
        else
        {
          /* Invert state and write back */
          for (int i = 0; i < 10; i++)
            coils[i] = !coils[i];
          modbus_write_bits(ctx, 0, 10, coils);
          retry_count = 0;
        }

        /* Read Discrete Inputs (0x02) */
        rc = modbus_read_input_bits(ctx, 0, 10, discrete_inputs);
        if (rc == -1)
        {
          printf("TCP Read Discrete Inputs failed: %s\n", modbus_strerror(errno));
          retry_count++;
        }
        else
        {
          printf("TCP Discrete Inputs: ");
          for (int i = 0; i < 10; i++)
          {
            printf("%d ", discrete_inputs[i]);
          }
          printf("\n");
          retry_count = 0;
        }

        // Error check: If max retry count exceeded, reconnect
        if (retry_count >= 4)
        {
          printf("TCP Max retry limit reached. Reconnecting...\n");
          modbus_close(ctx);
          modbus_free(ctx);
          state = RECONNECT_WAIT;
        }
        else
        {
          // Normal loop
          vTaskDelay(500);
        }
        break;

      case RECONNECT_WAIT:
        printf("[RECONNECT_WAIT] Waiting %d seconds before retrying...\n", 1);
        vTaskDelay(1000);
        state = INIT;
        break;

      case ERROR:
        printf("[ERROR] Critical failure. Exiting...\n");
        if (ctx)
        {
          modbus_free(ctx);
        }
        return;
        break;
    }
  }
}

/**
 * @brief  modbus tcp slave task
 * @param  pvParameters: task parameters
 * @retval none
 */
void modbus_tcp_slave_task(void *pvParameters)
{
  pvParameters = pvParameters;
  unsigned short index = 0;
  unsigned char query[MODBUS_TCP_MAX_ADU_LENGTH];
  int socket_fd;
  modbus_t *ctx;
  modbus_mapping_t *mb_mapping;

  /* Initialize Modbus TCP context */
  ctx = modbus_new_tcp("172.18.57.30", 6801);

  /* Create Modbus mapping */
  mb_mapping = modbus_mapping_new(20, 30, 50, 50);
  if (mb_mapping == NULL)
  {
    printf("modbus_tcp_slave_task Failed to allocate the mapping: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
  }

  /* Start listening for incoming connections */
  socket_fd = modbus_tcp_listen(ctx, 1);
  if (socket_fd == -1)
  {
    printf("Unable to listen TCP connection\n");
    modbus_free(ctx);
  }
  else
  {
    printf("listen ok.....\n");
  }
  printf("waiting accept.....\n");
  /* Accept incoming connection */
  if (modbus_tcp_accept(ctx, &socket_fd) == -1)
  {
    printf("Unable to accept TCP connection\n");
    modbus_free(ctx);
  }
  else
  {
    printf("accept ok.....\n");
    modbus_set_indication_timeout(ctx, 3, 0);
  }

  while (1)
  {
    /* Receive Modbus request */
    int rc = modbus_receive(ctx, query);
    if (rc > 0)
    {
      /* Process request and send response */
      modbus_reply(ctx, query, rc, mb_mapping);
    }
    else
    {
      printf("Connection closed.....................\n");
      modbus_close(ctx);
      modbus_tcp_accept(ctx, &socket_fd);
    }
    /* Dynamically update register values (simulate data changes) */
    mb_mapping->tab_registers[0] = index;
    mb_mapping->tab_input_registers[0] = index + 100;
    mb_mapping->tab_bits[1] = 0xf0;
    mb_mapping->tab_input_bits[1] = 0x0f;

    index++;
  }
}
