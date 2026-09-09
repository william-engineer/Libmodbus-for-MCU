/**
 ****************************************************************************************************
 * @file        freertos_demo.c
 * @author      正点原子团队(ALIENTEK), William Chen
 * @version     V1.1
 * @brief       STM32F429 FreeRTOS + lwIP + Libmodbus-for-MCU example
 ****************************************************************************************************
 */

#include "freertos_demo.h"

#include <stdio.h>

#include "./BSP/LED/led.h"
#include "./SYSTEM/delay/delay.h"
#include "FreeRTOS.h"
#include "lwip_comm.h"
#include "modbus-version.h"
#include "modbus_test.h"
#include "task.h"

#define START_TASK_PRIO                 1
#define START_TASK_STACK_SIZE           256
#define MODBUS_RTU_MASTER_TASK_PRIO     9
#define MODBUS_RTU_MASTER_TASK_SIZE     512
#define MODBUS_RTU_SLAVE_TASK_PRIO      11
#define MODBUS_RTU_SLAVE_TASK_SIZE      512
#define MODBUS_TCP_MASTER_TASK_PRIO     7
#define MODBUS_TCP_MASTER_TASK_SIZE     512
#define MODBUS_TCP_SLAVE_TASK_PRIO      8
#define MODBUS_TCP_SLAVE_TASK_SIZE      512

static TaskHandle_t s_pstStartTaskHandle;
static TaskHandle_t s_pstModbusRtuMasterHandle;
static TaskHandle_t s_pstModbusRtuSlaveHandle;
static TaskHandle_t s_pstModbusTcpMasterHandle;
static TaskHandle_t s_pstModbusTcpSlaveHandle;

static void start_task(void *l_pvParameters);

void freertos_demo(void)
{
  (void)xTaskCreate(start_task,
                    "start_task",
                    START_TASK_STACK_SIZE,
                    NULL,
                    START_TASK_PRIO,
                    &s_pstStartTaskHandle);

  vTaskStartScheduler();
}

static void start_task(void *l_pvParameters)
{
  (void)l_pvParameters;

  while (lwip_comm_init() != 0)
  {
    delay_ms(500);
    LED1_TOGGLE();
  }

  while ((g_lwipdev.dhcpstatus != 2) && (g_lwipdev.dhcpstatus != 0xFF))
  {
    vTaskDelay(pdMS_TO_TICKS(500U));
  }

  printf("Libmodbus-for-MCU version: %s\n", LIBMODBUS_FOR_MCU_VERSION_STRING);

  (void)xTaskCreate(modbus_rtu_master_test,
                    "modbus_rtu_master_test",
                    MODBUS_RTU_MASTER_TASK_SIZE,
                    NULL,
                    MODBUS_RTU_MASTER_TASK_PRIO,
                    &s_pstModbusRtuMasterHandle);

  (void)xTaskCreate(modbus_rtu_slave_test,
                    "modbus_rtu_slave_test",
                    MODBUS_RTU_SLAVE_TASK_SIZE,
                    NULL,
                    MODBUS_RTU_SLAVE_TASK_PRIO,
                    &s_pstModbusRtuSlaveHandle);

  (void)xTaskCreate(modbus_tcp_master_test,
                    "modbus_tcp_master_test",
                    MODBUS_TCP_MASTER_TASK_SIZE,
                    NULL,
                    MODBUS_TCP_MASTER_TASK_PRIO,
                    &s_pstModbusTcpMasterHandle);

  (void)xTaskCreate(modbus_tcp_slave_test,
                    "modbus_tcp_slave_test",
                    MODBUS_TCP_SLAVE_TASK_SIZE,
                    NULL,
                    MODBUS_TCP_SLAVE_TASK_PRIO,
                    &s_pstModbusTcpSlaveHandle);

  vTaskDelete(NULL);
}
