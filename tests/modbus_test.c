/*
 * Board-level Modbus integration tests.
 *
 * The example project owns scheduler startup and task creation.  This file
 * intentionally contains only the four Modbus test task functions so it can
 * be reused by different MCU examples.
 */
#include "modbus_test.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/errno.h"
#include "modbus.h"

void modbus_rtu_master_test(void *l_pvParameters)
{
  uint16_t l_au16HoldingRegisters[10] = {0};
  uint16_t l_au16InputRegisters[10] = {0};
  uint8_t l_au8Coils[10] = {0};
  uint8_t l_au8DiscreteInputs[10] = {0};
  modbus_t *l_pstCtx;
  int l_i32Rc;

  (void)l_pvParameters;

  l_pstCtx = modbus_new_rtu(MODBUS_TEST_RTU_MASTER_DEVICE, MODBUS_TEST_RTU_BAUD, 'N', 8, 1);
  if (l_pstCtx == NULL)
  {
    printf("RTU master context allocation failed: %s\n", modbus_strerror(errno));
    vTaskDelete(NULL);
    return;
  }

  modbus_set_slave(l_pstCtx, MODBUS_TEST_RTU_MASTER_SLAVE_ID);
  if (modbus_connect(l_pstCtx) == -1)
  {
    printf("RTU master connection failed: %s\n", modbus_strerror(errno));
    modbus_free(l_pstCtx);
    vTaskDelete(NULL);
    return;
  }

  for (;;)
  {
    l_i32Rc = modbus_read_registers(l_pstCtx, 0, 10, l_au16HoldingRegisters);
    if (l_i32Rc == -1)
    {
      printf("RTU read holding registers failed: %s\n", modbus_strerror(errno));
    }
    else
    {
      for (int l_i32Index = 0; l_i32Index < 10; l_i32Index++)
      {
        l_au16HoldingRegisters[l_i32Index]++;
      }
      if (modbus_write_registers(l_pstCtx, 0, 10, l_au16HoldingRegisters) == -1)
      {
        printf("RTU write holding registers failed: %s\n", modbus_strerror(errno));
      }
    }

    l_i32Rc = modbus_read_input_registers(l_pstCtx, 0, 10, l_au16InputRegisters);
    if (l_i32Rc == -1)
    {
      printf("RTU read input registers failed: %s\n", modbus_strerror(errno));
    }

    l_i32Rc = modbus_read_bits(l_pstCtx, 0, 10, l_au8Coils);
    if (l_i32Rc == -1)
    {
      printf("RTU read coils failed: %s\n", modbus_strerror(errno));
    }
    else
    {
      for (int l_i32Index = 0; l_i32Index < 10; l_i32Index++)
      {
        l_au8Coils[l_i32Index] = !l_au8Coils[l_i32Index];
      }
      if (modbus_write_bits(l_pstCtx, 0, 10, l_au8Coils) == -1)
      {
        printf("RTU write coils failed: %s\n", modbus_strerror(errno));
      }
    }

    l_i32Rc = modbus_read_input_bits(l_pstCtx, 0, 10, l_au8DiscreteInputs);
    if (l_i32Rc == -1)
    {
      printf("RTU read discrete inputs failed: %s\n", modbus_strerror(errno));
    }

    vTaskDelay(pdMS_TO_TICKS(500U));
  }
}

void modbus_rtu_slave_test(void *l_pvParameters)
{
  uint8_t l_au8Query[MODBUS_RTU_MAX_ADU_LENGTH];
  modbus_mapping_t *l_pstMapping;
  modbus_t *l_pstCtx;
  uint16_t l_u16Index = 0;
  int l_i32Rc;

  (void)l_pvParameters;

  l_pstCtx = modbus_new_rtu(MODBUS_TEST_RTU_SLAVE_DEVICE, MODBUS_TEST_RTU_BAUD, 'N', 8, 1);
  if (l_pstCtx == NULL)
  {
    printf("RTU slave context allocation failed: %s\n", modbus_strerror(errno));
    vTaskDelete(NULL);
    return;
  }

  modbus_set_slave(l_pstCtx, MODBUS_TEST_RTU_SLAVE_ID);
  modbus_set_response_timeout(l_pstCtx, 1, 0);
  if (modbus_connect(l_pstCtx) == -1)
  {
    printf("RTU slave connection failed: %s\n", modbus_strerror(errno));
    modbus_free(l_pstCtx);
    vTaskDelete(NULL);
    return;
  }

  l_pstMapping = modbus_mapping_new(100, 100, 100, 100);
  if (l_pstMapping == NULL)
  {
    printf("RTU slave mapping allocation failed: %s\n", modbus_strerror(errno));
    modbus_close(l_pstCtx);
    modbus_free(l_pstCtx);
    vTaskDelete(NULL);
    return;
  }

  l_pstMapping->tab_registers[0] = 0x1234;
  l_pstMapping->tab_input_registers[0] = 0x5678;
  l_pstMapping->tab_bits[0] = 1;
  l_pstMapping->tab_input_bits[0] = 0;

  for (;;)
  {
    l_i32Rc = modbus_receive(l_pstCtx, l_au8Query);
    if (l_i32Rc > 0)
    {
      if (modbus_reply(l_pstCtx, l_au8Query, l_i32Rc, l_pstMapping) == -1)
      {
        printf("RTU slave reply failed: %s\n", modbus_strerror(errno));
      }
    }
    else if (l_i32Rc == -1)
    {
      printf("RTU slave receive failed: %s\n", modbus_strerror(errno));
      vTaskDelay(pdMS_TO_TICKS(10U));
    }

    l_pstMapping->tab_registers[0] = l_u16Index;
    l_pstMapping->tab_input_registers[0] = (uint16_t)(l_u16Index + 1U);
    l_pstMapping->tab_bits[1] = (uint8_t)(l_u16Index & 1U);
    l_u16Index++;
  }
}

void modbus_tcp_master_test(void *l_pvParameters)
{
  uint16_t l_au16HoldingRegisters[10] = {0};
  uint16_t l_au16InputRegisters[10] = {0};
  uint8_t l_au8Coils[10] = {0};
  uint8_t l_au8DiscreteInputs[10] = {0};
  modbus_t *l_pstCtx = NULL;
  unsigned int l_u32RetryCount = 0;
  int l_i32Rc;
  enum
  {
    MODBUS_TCP_TEST_INIT,
    MODBUS_TCP_TEST_CONNECTING,
    MODBUS_TCP_TEST_CONNECTED,
    MODBUS_TCP_TEST_RECONNECT_WAIT
  } l_enState = MODBUS_TCP_TEST_INIT;

  (void)l_pvParameters;

  for (;;)
  {
    switch (l_enState)
    {
      case MODBUS_TCP_TEST_INIT:
        l_pstCtx = modbus_new_tcp(MODBUS_TEST_TCP_MASTER_IP, MODBUS_TEST_TCP_MASTER_PORT);
        if (l_pstCtx == NULL)
        {
          printf("TCP master context allocation failed: %s\n", modbus_strerror(errno));
          l_enState = MODBUS_TCP_TEST_RECONNECT_WAIT;
          break;
        }
        l_enState = MODBUS_TCP_TEST_CONNECTING;
        break;

      case MODBUS_TCP_TEST_CONNECTING:
        modbus_set_response_timeout(l_pstCtx, 2, 0);
        if (modbus_connect(l_pstCtx) == -1)
        {
          printf("TCP master connection failed: %s\n", modbus_strerror(errno));
          modbus_free(l_pstCtx);
          l_pstCtx = NULL;
          l_enState = MODBUS_TCP_TEST_RECONNECT_WAIT;
        }
        else
        {
          l_u32RetryCount = 0;
          modbus_set_response_timeout(l_pstCtx, 0, 500000);
          l_enState = MODBUS_TCP_TEST_CONNECTED;
        }
        break;

      case MODBUS_TCP_TEST_CONNECTED:
        l_i32Rc = modbus_read_registers(l_pstCtx, 0, 10, l_au16HoldingRegisters);
        if (l_i32Rc == -1)
        {
          printf("TCP read holding registers failed: %s\n", modbus_strerror(errno));
          l_u32RetryCount++;
        }
        else
        {
          for (int l_i32Index = 0; l_i32Index < 10; l_i32Index++)
          {
            l_au16HoldingRegisters[l_i32Index]++;
          }
          if (modbus_write_registers(l_pstCtx, 0, 10, l_au16HoldingRegisters) == -1)
          {
            l_u32RetryCount++;
          }
        }

        l_i32Rc = modbus_read_input_registers(l_pstCtx, 0, 10, l_au16InputRegisters);
        l_u32RetryCount += (l_i32Rc == -1) ? 1U : 0U;

        l_i32Rc = modbus_read_bits(l_pstCtx, 0, 10, l_au8Coils);
        if (l_i32Rc == -1)
        {
          l_u32RetryCount++;
        }
        else
        {
          for (int l_i32Index = 0; l_i32Index < 10; l_i32Index++)
          {
            l_au8Coils[l_i32Index] = !l_au8Coils[l_i32Index];
          }
          if (modbus_write_bits(l_pstCtx, 0, 10, l_au8Coils) == -1)
          {
            l_u32RetryCount++;
          }
        }

        l_i32Rc = modbus_read_input_bits(l_pstCtx, 0, 10, l_au8DiscreteInputs);
        l_u32RetryCount += (l_i32Rc == -1) ? 1U : 0U;

        if (l_u32RetryCount >= 4U)
        {
          printf("TCP master retry limit reached; reconnecting\n");
          modbus_close(l_pstCtx);
          modbus_free(l_pstCtx);
          l_pstCtx = NULL;
          l_enState = MODBUS_TCP_TEST_RECONNECT_WAIT;
        }
        else
        {
          vTaskDelay(pdMS_TO_TICKS(500U));
        }
        break;

      case MODBUS_TCP_TEST_RECONNECT_WAIT:
      default:
        vTaskDelay(pdMS_TO_TICKS(1000U));
        l_enState = MODBUS_TCP_TEST_INIT;
        break;
    }
  }
}

void modbus_tcp_slave_test(void *l_pvParameters)
{
  uint8_t l_au8Query[MODBUS_TCP_MAX_ADU_LENGTH];
  modbus_mapping_t *l_pstMapping;
  modbus_t *l_pstCtx;
  uint16_t l_u16Index = 0;
  int l_i32ServerSocket;

  (void)l_pvParameters;

  l_pstCtx = modbus_new_tcp(MODBUS_TEST_TCP_SLAVE_IP, MODBUS_TEST_TCP_SLAVE_PORT);
  if (l_pstCtx == NULL)
  {
    printf("TCP slave context allocation failed: %s\n", modbus_strerror(errno));
    vTaskDelete(NULL);
    return;
  }

  l_pstMapping = modbus_mapping_new(20, 30, 50, 50);
  if (l_pstMapping == NULL)
  {
    printf("TCP slave mapping allocation failed: %s\n", modbus_strerror(errno));
    modbus_free(l_pstCtx);
    vTaskDelete(NULL);
    return;
  }

  l_i32ServerSocket = modbus_tcp_listen(l_pstCtx, 1);
  if (l_i32ServerSocket == -1)
  {
    printf("TCP slave listen failed: %s\n", modbus_strerror(errno));
    modbus_mapping_free(l_pstMapping);
    modbus_free(l_pstCtx);
    vTaskDelete(NULL);
    return;
  }

  for (;;)
  {
    if (modbus_tcp_accept(l_pstCtx, &l_i32ServerSocket) == -1)
    {
      printf("TCP slave accept failed: %s\n", modbus_strerror(errno));
      vTaskDelay(pdMS_TO_TICKS(1000U));
      continue;
    }

    modbus_set_indication_timeout(l_pstCtx, 3, 0);
    for (;;)
    {
      int l_i32Rc = modbus_receive(l_pstCtx, l_au8Query);
      if (l_i32Rc <= 0)
      {
        break;
      }

      if (modbus_reply(l_pstCtx, l_au8Query, l_i32Rc, l_pstMapping) == -1)
      {
        break;
      }

      l_pstMapping->tab_registers[0] = l_u16Index;
      l_pstMapping->tab_input_registers[0] = (uint16_t)(l_u16Index + 100U);
      l_pstMapping->tab_bits[1] = 1;
      l_pstMapping->tab_input_bits[1] = 0;
      l_u16Index++;
    }

    modbus_close(l_pstCtx);
  }
}
