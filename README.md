# mini_car

基于 STM32F103C8Tx 的循迹智能小车固件。项目面向带有 12 路光电管、双轮编码器和 BMI270 惯性传感器的两轮差速小车，使用级联控制实现循迹、弯道处理和安全停车。

> 当前项目依赖具体的电机驱动、电源、传感器安装方向和赛道布局。首次上电前请先架空车轮，确认电机方向、传感器读数和急停逻辑。

## 项目背景

本仓库代码在 2025 年秋季 NGJY 小车竞速赛校赛中取得第一名，是南工绝影三队的起点。项目由 `HanDcLaP-Lab` 与 `KlemdRick_hy` 协作开发。2026 年 9 月 9 日，在赛事结束后对仓库进行整理，希望能给正在调车的同学带来一些思路。

## 功能特性

- STM32F103C8T6（Cortex-M3，72 MHz）固件工程。
- 12 路光电管阵列，通过 4 位通道选择信号读取循迹信息。
- BMI270 通过 SPI1 接入，支持角速度和加速度数据读取，SPI 使用 DMA 传输。
- 左右轮增量编码器分别使用 TIM4 和 TIM3 的 Encoder Interface 模式。
- TIM1 输出左右电机 PWM，独立方向引脚控制电机正反转。
- TIM2 以 1 kHz 中断作为主控制周期。
- 转向环使用带非线性增益和陀螺仪阻尼的 Dual-PD 控制器。
- 左右轮速度环使用 PID 控制器，并对编码器和转向测量值进行一元卡尔曼滤波。
- 状态机覆盖直线、缓弯、急弯、边缘、恢复和丢线处理。
- 包含丢线、角速度异常和运行时间上限等安全停车逻辑。

## 控制流程

```text
12 路光电管 / MUX
        │
        ▼
偏差计算与状态机 ──► 转向 Dual-PD ◄── BMI270 角速度阻尼
        │                    │
        │                    ▼
        └──────────────► 左右轮速度目标
                             │
                     编码器速度测量与滤波
                             │
                     左右轮速度 PID
                             │
                     TIM1 PWM + 电机方向
```

主控制逻辑集中在 [`Core/Src/main.c`](Core/Src/main.c) 中；驱动和传感器适配层位于 `Core/code/`。

## 硬件接口

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| `L_PWM` | PA8 / TIM1_CH1 | 左电机 PWM |
| `R_PWM` | PA9 / TIM1_CH2 | 右电机 PWM |
| `R_DIR` | PA10 | 右电机方向 |
| `FAN_PWM` | PA11 / TIM1_CH4 | 风扇或扩展 PWM 输出 |
| `MUX_READ` | PA12 | 光电管 MUX 数据输入 |
| `BMI270_CS` | PA4 | BMI270 SPI 片选 |
| `SPI1_SCK` | PA5 | BMI270 SPI 时钟 |
| `SPI1_MISO` | PA6 | BMI270 SPI 主入 |
| `SPI1_MOSI` | PA7 | BMI270 SPI 主出 |
| `L_DIR` | PB15 | 左电机方向 |
| `MUX_0` | PA15 | MUX 通道选择位 0 |
| `MUX_1` | PB3 | MUX 通道选择位 1 |
| `R_ENCODER_B` | PB4 / TIM3_CH1 | 右轮编码器 B 相 |
| `R_ENCODER_A` | PB5 / TIM3_CH2 | 右轮编码器 A 相 |
| `L_ENCODER_B` | PB6 / TIM4_CH1 | 左轮编码器 B 相 |
| `L_ENCODER_A` | PB7 / TIM4_CH2 | 左轮编码器 A 相 |
| `MUX_2` | PB8 | MUX 通道选择位 2 |
| `MUX_3` | PB9 | MUX 通道选择位 3 |
| `USART3_TX` | PB10 | 调试串口发送 |
| `USART3_RX` | PB11 | 调试串口接收 |

以上引脚配置同时记录在 [`micro_smartcar.ioc`](micro_smartcar.ioc) 和 [`Core/Inc/main.h`](Core/Inc/main.h) 中。如需更换硬件连接，请同步修改 CubeMX 配置、引脚定义和对应驱动代码。

## 工程结构

```text
mini_car_race/
├── Core/
│   ├── Inc/                  # CubeMX 生成的头文件
│   ├── Src/                  # 启动文件、HAL 初始化和主控制逻辑
│   ├── Startup/              # STM32 启动汇编
│   └── code/                 # 项目驱动和传感器适配代码
├── Drivers/                  # STM32 HAL、CMSIS 及其第三方许可证
├── MDK-ARM/                  # Keil uVision 工程和输出目录
├── micro_smartcar.ioc        # STM32CubeMX 配置
├── STM32F103C8TX_FLASH.ld    # 链接脚本
├── LICENSE                   # 本项目自有代码的许可证
└── README.md
```

`Core/Inc/` 和 `Core/Src/` 中包含 CubeMX 生成内容。修改这些文件时，请将自定义代码放在 `USER CODE BEGIN` / `USER CODE END` 标记之间，避免重新生成工程时丢失修改。`Drivers/` 中的厂商代码应保留其原始版权和许可证文件。

## 构建与烧录

### 环境要求

- Windows
- Keil MDK-ARM / µVision
- ARM Compiler 6（工程当前使用 ARMCLANG 6.18）
- Keil STM32F1xx Device Family Pack 2.2.0 或兼容版本
- 目标芯片：STM32F103C8T6

本仓库没有 Makefile、命令行构建脚本或自动化测试套件；官方构建方式是使用 Keil 工程。

### 构建步骤

1. 打开 [`MDK-ARM/micro_smartcar.uvprojx`](MDK-ARM/micro_smartcar.uvprojx)。
2. 选择 `micro_smartcar` Target。
3. 执行 Build 或 Rebuild。
4. 构建生成的 HEX、AXF 和调试输出位于 `MDK-ARM/micro_smartcar/`。
5. 使用适配 STM32F103C8T6 的调试器或下载器烧录 HEX 文件。

需要清理 Keil 构建缓存时，可运行 [`MDK-ARM/清除缓存文件.bat`](MDK-ARM/清除缓存文件.bat)。

### STM32CubeMX 配置

`micro_smartcar.ioc` 用于记录外设、引脚、DMA、定时器和时钟配置。除非确实需要调整硬件配置，否则不建议直接重新生成整个工程；重新生成后应检查 `Core/Src/main.c`、中断处理和 `Core/code/` 驱动是否仍然完整。

## 串口调试

程序将 `printf` 重定向到 USART3：

- TX：PB10
- RX：PB11
- 波特率：115200
- 数据格式：8 数据位、无校验、1 停止位（8N1）

将串口转换器连接到正确的电平和地线后，可使用串口工具观察调试输出。控制周期、传感器采样、状态机和 PWM 输出相关调试信息在 [`Core/Src/main.c`](Core/Src/main.c) 中维护；部分输出默认关闭，按需启用后再进行调试。

## 参数调节

主要可调参数集中在 [`Core/Src/main.c`](Core/Src/main.c)：

- 左右轮速度 PID：`L`、`R` 控制器初始化值。
- 转向 Dual-PD：`ROT` 控制器初始化值。
- 速度上限、丢线计时、角速度保护和运行时间：文件中的 `#define` 参数。
- MUX 权重、状态机阈值、弯道速度因子和恢复时间：偏差计算和状态机相关代码。

建议一次只调整一组参数，并在架空车轮、低速和可立即断电的条件下验证。传感器方向、左右电机方向或编码器相位错误时，继续调 PID 通常无法解决问题，应先校正硬件和信号极性。

## 分支与版本

- `main`：仓库默认分支。
- `stable/v*`：按版本维护的稳定或实验分支。
- 具体行为以目标分支的提交和标签为准；不同版本可能使用不同的状态机阈值和控制参数。

## 贡献说明

提交代码前请：

1. 确认没有提交密钥、个人配置、串口日志或其他敏感信息。
2. 保留 STM32、Bosch 及其他第三方代码中的版权声明和许可证。
3. 将 CubeMX 生成文件的自定义修改放在用户代码区域。
4. 在真实小车上测试前，先验证电机方向、编码器方向、BMI270 数据和安全停车逻辑。

## 许可证

除另有说明的第三方文件外，本项目中由 `HanDcLaP-Lab` 与 `KlemdRick_hy` 共同创作、拥有并有权授权的代码采用 MIT License，详见 [`LICENSE`](LICENSE)。

本仓库还包含第三方组件，它们不受本项目根目录许可证替代：

- STM32 CMSIS：见 `Drivers/CMSIS/LICENSE.txt`。
- STM32 HAL：见 `Drivers/STM32F1xx_HAL_Driver/LICENSE.txt`。
- Bosch BMI270/BMI2 驱动：见 `Core/code/bmi*.c` 和 `Core/code/bmi*.h` 文件头中的 BSD-3-Clause 声明。

分发包含这些组件的产品或二进制文件时，请同时遵守对应的版权和许可证要求。
