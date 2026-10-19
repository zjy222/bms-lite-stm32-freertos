# 构建、下载与运行

## 固件

实测工具链：Keil µVision / ARM Compiler 5.06 update 3，STM32F1xx_DFP 2.4.1；器件 STM32F103C8T6，8 MHz 外部晶振，运行 72 MHz，64 KB Flash、20 KB RAM。

1. 安装 Keil MDK、ARM Compiler 5 及 STM32F1xx_DFP 2.4.1。RTE 中还声明 ARM.CMSIS 6.3.0 / CORE 6.2.0；按工程提示安装所需软件包。
2. 打开 firmware/bms-lite.uvprojx，选择 Target 1，检查设备与编译器版本。
3. Rebuild。工程使用相对源码路径，FreeRTOS 必要源码已包含；ST 外设库与设备头文件由 Device Pack 提供。不要把编译器/Pack 安装目录复制进仓库。
4. 使用 ST-Link 下载；确认目标板供电稳定，下载器设置根据自己的设备配置。首次复现建议先不接负载，核对温度与继电器指令，再断电接入负载。
5. 正常温度下负载通电；受控加温温度探头至 30°C 观察报警和断载，降温至 29°C 或以下观察恢复。不要通过加热电池进行演示。

本工程保留当前实际使用的 FreeRTOS 源码快照；task.h 标为 V11.1.0+ 开发分支，并非声称来自某个已确认的发布 tag。源码哈希见第三方清单。尚未验证 ARM Compiler 6、其他芯片或其他 Pack 版本。

## 主站

按根 README 或 host/README.md 创建虚拟环境、安装依赖。默认 115200/8E1，地址 1，36 个保持寄存器。选择 USB-RS485 所在串口，关闭占用该串口的其他软件。按 Ctrl+C 停止，CSV 每行刷新写盘。

## 软件测试

安装 Python 依赖，并保证 gcc 在 PATH 中，在仓库根目录执行：

```powershell
.\.venv\Scripts\python.exe tools/run_tests.py
```

测试从实际 main.c 提取 C 逻辑并用 GCC + 硬件桩编译执行，另用真实 MinimalModbus 库与模拟串口验证主站。它们不替代 MCU 全量构建、硬件测试或示波器时序检查。CI 配置只运行这些主机软件测试，不执行 Keil 或实物测试。
