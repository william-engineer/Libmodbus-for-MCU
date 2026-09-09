# MCU 板级端口

根目录 `src/` 是唯一的 Modbus 协议栈实现，`arch/modbus-rtu-hal.h` 是所有 MCU
端口共用的唯一 UART 设备接口头文件。本目录仅保存与具体开发板绑定的 RTU
UART、DMA、GPIO 和中断实现，不再保存同名公共头文件副本。

## 选择端口

每个目标工程只能编译一个 `modbus-rtu-hal.c`：

- `stm32f429/`：STM32F429，USART2/USART3，STM32 HAL，PCF8574 方向控制。
- `gd32f527/`：GD32F527，USART1/USART2，GD32 Firmware Library，GPIO 方向控制。
- `at32f407/`：AT32F407，USART2，AT32 Firmware Library，GPIO 方向控制。

工程头文件搜索路径应包含根目录 `arch/`，板级源码通过
`#include "../../modbus-rtu-hal.h"` 使用公共接口。不要将具体端口目录加入搜索路径
来替代根目录 `arch/`，也不要同时编译多个端口，否则公共函数和中断入口会重复定义。

## 板级硬件上下文

公共 `stUartDevice` 只保存协议和运行状态。UART、TX DMA、RX DMA 句柄通过
`m_pvPortContext` 指向板级私有的 `stUartContext`：

```c
typedef struct
{
  VendorUartHandle m_stHalHandle;
  VendorDmaHandle m_stTxDmaHandle;
  VendorDmaHandle m_stRxDmaHandle;
  /* 可继续增加 DMA 控制器、GPIO、时钟和 IRQ 等板级资源。 */
} stUartContext;

#define UART_CONTEXT(dev) \
  ((stUartContext *)((dev)->m_pvPortContext))
#define UART_HANDLE(dev)   (UART_CONTEXT(dev)->m_stHalHandle)
#define TX_DMA_HANDLE(dev) (UART_CONTEXT(dev)->m_stTxDmaHandle)
#define RX_DMA_HANDLE(dev) (UART_CONTEXT(dev)->m_stRxDmaHandle)
```

三个基础成员和四个访问宏保持相同命名，但成员具体类型由厂商驱动决定。GD32 和
AT32 等平台需要的附加资源继续放在各自上下文中，不应加入公共 `stUartDevice`。

板级源文件应自行包含对应厂商设备头文件。公共 `arch/modbus-port.h` 只负责 RTOS
和网络协议栈适配，因此增加新 MCU 端口时不需要修改它。

## 新端口接入步骤

1. 新建 `arch/ports/<mcu>/modbus-rtu-hal.c`，包含厂商头文件和公共
   `../../modbus-rtu-hal.h`。
2. 定义板级 `stUartContext`、硬件实例和 `stUartDevice`，通过
   `m_pvPortContext` 建立关联。
3. 使用 `MODBUS_UART1`、`MODBUS_UART2`、`MODBUS_UART3` 作为设备表索引，并将
   相同枚举值赋给公共成员 `m_enDevId`。如需更多 UART，先扩展公共设备 ID 枚举。
4. 实现 `init`、`send`、`recv`、`flush`、`dir_switch` 和
   `find_uart_device()`，再接入该平台的 UART/DMA 中断。
5. 验证 UART IDLE 收帧、DMA 错误恢复、物理发送完成后切换接收，以及连续报文的
   RTU t3.5 帧间隔。

GPIO、UART 实例、DMA 通道和中断函数均与已验证开发板资源绑定。移植到同系列的
其他板卡时，仍需按照原理图和芯片资源重新配置。
