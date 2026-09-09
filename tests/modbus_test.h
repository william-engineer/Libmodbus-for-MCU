#ifndef MODBUS_TEST_H
#define MODBUS_TEST_H

/* These defaults match the STM32F429 example.  A board example may override
 * them with compiler definitions before including this header. */
#ifndef MODBUS_TEST_RTU_MASTER_DEVICE
    #define MODBUS_TEST_RTU_MASTER_DEVICE "uart3"
#endif

#ifndef MODBUS_TEST_RTU_SLAVE_DEVICE
    #define MODBUS_TEST_RTU_SLAVE_DEVICE "uart2"
#endif

#ifndef MODBUS_TEST_RTU_BAUD
    #define MODBUS_TEST_RTU_BAUD 115200
#endif

#ifndef MODBUS_TEST_RTU_MASTER_SLAVE_ID
    #define MODBUS_TEST_RTU_MASTER_SLAVE_ID 2
#endif

#ifndef MODBUS_TEST_RTU_SLAVE_ID
    #define MODBUS_TEST_RTU_SLAVE_ID 1
#endif

#ifndef MODBUS_TEST_TCP_MASTER_IP
    #define MODBUS_TEST_TCP_MASTER_IP "172.18.57.22"
#endif

#ifndef MODBUS_TEST_TCP_MASTER_PORT
    #define MODBUS_TEST_TCP_MASTER_PORT 6805
#endif

#ifndef MODBUS_TEST_TCP_SLAVE_IP
    #define MODBUS_TEST_TCP_SLAVE_IP "172.18.57.30"
#endif

#ifndef MODBUS_TEST_TCP_SLAVE_PORT
    #define MODBUS_TEST_TCP_SLAVE_PORT 6801
#endif

void modbus_rtu_master_test(void *l_pvParameters);
void modbus_rtu_slave_test(void *l_pvParameters);
void modbus_tcp_master_test(void *l_pvParameters);
void modbus_tcp_slave_test(void *l_pvParameters);

#endif /* MODBUS_TEST_H */
