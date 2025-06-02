<h1>Libmodbus for MCU</h1>
<p align="center">
  <img src="./docs/assets/libmodbus for mcu.png" alt="libmodbus for MCU">
</p>
**Libmodbus for MCU** 旨在将开源 Modbus 协议栈 libmodbus 适配到资源受限的 MCU 平台上，结合实时操作系统（FreeRTOS）和轻量级 TCP/IP 协议栈（lwIP），实现稳定可靠的 Modbus 通讯能力。
本项目经过在 STM32F429 平台的实际测试，验证了 Modbus TCP 和 Modbus RTU 分别在 Master 和 Slave 的功能，适用于需要在 MCU 上快速实现 Modbus 通讯的应用场景。

![Build Status](https://img.shields.io/badge/build-passing-brightgreen) ![Static Badge](https://img.shields.io/badge/STM32-passing-brightgreen?logo=stmicroelectronics) [![License: LGPL v2.1](https://img.shields.io/badge/license-LGPL%20v2.1-blue)](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html) [![Language: C](https://img.shields.io/badge/language-C-ff69b4)](https://en.wikipedia.org/wiki/C_(programming_language))

**中文简体** ｜ [**English**](./docs/README_EN.md) 

---

## 特性 Features

- 支持 Modbus RTU 多主站与从站
- 支持 Modbus TCP 多主站与从站
- 基于 lwIP 的轻量级 TCP/IP 协议栈适配
- 对 libmodbus 源码进行了适配，增加了硬件抽象接口（HAL）。
- 移植层独立，便于适配不同 MCU 平台

## 支持平台 Supported Platforms

- 正点原子 阿波罗 STM32F429 开发板（测试平台）
- 理论上支持所有带 FreeRTOS 和 lwIP 的 MCU


## 依赖 Dependencies

- FreeRTOS：V10.4.6
- lwIP：V2.1.3
- libmodbus：V3.1.11

## 使用方法 How to Use
### 平台搭建
1. 在目标板上完成实时操作系统（RTOS）移植，确保系统正常运行。
2. 移植 LwIP 协议栈，并在 `lwipopts.h` 中进行必要配置：

   - 设置 `LWIP_SOCKET`、`SO_REUSE`、`LWIP_NETCONN`、`LWIP_SOCKET_SELECT`、`LWIP_POSIX_SOCKETS_IO_NAMES` 为 `1`。
   - 设置 `LWIP_COMPAT_SOCKETS` 为 `0`。

   若上述选项未正确配置，将导致编译错误。
### modbus-port.h 移植
1. 在 `arch/modbus-port.h` 文件中添加目标MCU的头文件，并根据具体平台的外设定义，重定义 UART 和 DMA 的句柄类型（stUartHalHandle_t、stUartTxDmaHandle_t、stUartRxDmaHandle_t），以适配该平台的串口与 DMA 驱动。

2. 添加对应的操作系统头文件，并重定义信号量与消息队列类型（pstModbusSem_t、pstModbusQueue_t），同时定义最大延时时间常量（MODBUS_MAX_DELAY）以及延时函数宏（os_delay_ms(ms)），以实现操作系统层与 Modbus 协议栈之间的解耦。

3. 若使用其他支持 BSD socket 接口的 TCP/IP 协议栈（如 uC/TCP-IP、embOS/IP等），需要在 modbus_net_xxx 接口处进行适配。例如，将 modbus_net_send、modbus_net_recv 等宏定义映射为对应协议栈的 socket 函数。

   > 注意：若使用的 TCP/IP 协议栈不支持标准 BSD socket，则无法直接适配这些接口。

### modbus-rtu 硬件抽象层移植

libmodbus 最初设计用于 Linux, Mac OS X, FreeBSD, Embox, QNX 和 Windows等支持标准 POSIX I/O 接口的平台，其串口通信层完全依赖于主机操作系统提供的标准 POSIX I/O 接口和内核驱动模型，这些接口通常在 MCU 环境中并不存在。因此，在移植 libmodbus 到 MCU 平台时，需要重写串口相关的底层驱动接口，通常包括初始化、发送、接收、方向控制等功能。

**Libmodbus for MCU** 中设计了一个UART设备类，并采用设备表对设备进行管理。在创建 RTU 数据通道之前，需先定义 UART 设备，并完成相关成员变量与回调函数的注册。UART设备类的定义如下：

```c
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
```

在定义UART设备时，需要完成下列必要操作：

1. 指定设备名称，用于libmodbus的modbus_new_rtu函数
2. 绑定设备ID，用于设备表的管理
3. 定义一个接受缓存区（每一个设备应该有一个独立的接受缓存区），并绑定到设备，用于接受串口数据。
4. 注册init、send、recv、flush以及dir_switch功能的回调函数

在完成设备定义后，需要对 5 个回调函数进行平台适配。主要工作包括：在 init 回调中完成 UART 设备与 DMA 中断的初始化，并创建用于发送和接收的信号量；其余回调函数中，则需根据所使用的操作系统与硬件平台，替换相应的 **DMA 操作逻辑**与**信号量处理方式**。


## 测试 Testing

### 环境准备

- 使用 Modbus 调试工具：Modbus Poll、Modbus Slave。
- 配置测试主机IP为`172.18.57.11`，或者修改测试代码TCP Master任务中的连接IP地址。
- 具有FreeRTOS实时操作系统和LwIP协议栈的工程，同时保证网络连通性和串口连接正常。

### 测试步骤

1. **添加测试程序**
   main函数中添加`modbus_test`测试函数，创建测试任务。

2. **创建 Modbus Poll 配置**

   - 在Modbus Poll中分别创建串口和TCP连接，串口选择uart2对应的USB设备，TCP 的IP和端口为`172.18.57.30:6801`
   - 创建`读线圈`、`读离散输入`、`读保持寄存器`、`读输入寄存器`4个功能码的窗口，用于统计和观察通信情况

3. **创建 Modbus Slave 配置**

   - 在Modbus Slave中分别创建串口和TCP连接，与Poll类似创建`线圈状态`、`输入状态`、`保持寄存器状态`、`输入寄存器状态`4个地址区的窗口，并将数据做累加或翻转用于统计和观察通信情况

4. **观察串口打印信息**

   系统运行后，观察调试软件和打印信息，可以看到数据正常的变化

### 结果验证

- 调试工具的测试结果如下：

![Modbus调试工具](./docs/assets/modbus.gif)

- 打印信息如下：

![Modbus调试log](./docs/assets/modbus-log.gif)



## 许可证 License
本项目遵循 LGPL-2.1及之后的许可证，更多详细信息请查看 `LICENSE` 文件。

---

# 联系方式 Contact

- Author: William Chen
- Email: william_engineer@outlook.com