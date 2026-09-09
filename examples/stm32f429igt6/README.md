# STM32F429 Modbus RTU/TCP 示例

本示例面向正点原子阿波罗 STM32F429 开发板，使用 FreeRTOS 10.4.6、lwIP 2.1.3 和 STM32 HAL，同时验证 Modbus RTU/TCP 的 Master 与 Slave 功能。

## 源码关系

示例工程不保存 libmodbus 源码副本。Keil MDK 与 CMake 均直接引用仓库根目录中的文件：

```text
../../src/                                  Modbus 协议栈
../../arch/                                公共 MCU 适配接口
../../arch/ports/stm32f429/modbus-rtu-hal.c STM32F429 板级端口
../../tests/modbus_test.c                   四个集成测试任务
```

`User/freertos_demo.c` 只负责板级网络初始化、任务参数配置和任务创建；四个 RTU/TCP 测试任务统一实现在 `../../tests/modbus_test.c` 中。

## 构建方式

- Keil MDK：`Projects/MDK-ARM/atk_f429.uvprojx`
- CMake + Arm GNU Toolchain：`CMakeLists.txt`

当前工程中的 MDK 依赖文件完整。使用 Arm GNU Toolchain 构建前，还需要在示例依赖中补充 CMSIS 的 `cmsis_gcc.h` 以及 FreeRTOS 的 GCC Cortex-M4F portable 端口；

## 测试任务

示例默认创建以下四个任务：

- RTU Master：USART3，访问 Slave ID 2
- RTU Slave：USART2，Slave ID 1
- TCP Master：连接 `172.18.57.22:6805`
- TCP Slave：监听 `172.18.57.30:6801`

设备名、波特率、站号、IP 和端口的默认值位于 `../../tests/modbus_test.h`。

## 硬件资源

### Modbus RTU

| 角色 | UART | 引脚 | 默认站号 |
| --- | --- | --- | --- |
| Master | USART3 | PD8/PD9（TX/RX） | 访问 Slave ID 2 |
| Slave | USART2 | PD5/PD6（TX/RX） | Slave ID 1 |

RS485 方向控制使用 PCF8574 软件 I²C。由于方向切换包含毫秒级延时，当前实现在线程上下文完成切换；若改为直接 GPIO 控制，可参考 STM32F429 端口中 USART 发送完成中断旁的示例代码。

### 以太网 YT8521

| 信号 | 引脚 |
| --- | --- |
| ETH_MDIO | PA2 |
| ETH_MDC | PC1 |
| ETH_RMII_REF_CLK | PA1 |
| ETH_RMII_CRS_DV | PA7 |
| ETH_RMII_RXD0 | PC4 |
| ETH_RMII_RXD1 | PC5 |
| ETH_RMII_TX_EN | PB11 |
| ETH_RMII_TXD0 | PG13 |
| ETH_RMII_TXD1 | PG14 |
