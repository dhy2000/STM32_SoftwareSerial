# STM32 Software Serial

在 STM32 单片机上借助 TIM 时钟中断用软件模拟 UART 串口，从而可以用任意一对 GPIO 作为 RX/TX 进行 UART 通讯，达到类似 Arduino 标准库中的 `SoftwareSerial` 的效果。

本套代码采用 CubeIDE 开发，选用的单片机型号为 STM32F103C8T6 ，管脚配置见 ioc 文件。