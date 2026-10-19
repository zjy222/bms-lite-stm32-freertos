# 第三方内容与许可

原创应用代码、主站、测试和文档暂不指定开源许可证，发布前再决定；这不改变以下第三方内容的许可。

| 内容 | 来源 / 版本依据 | 许可与交付方式 |
| --- | --- | --- |
| FreeRTOS Kernel 必要源码及 RVDS ARM_CM3 / heap_4 | 本地实际工程快照；task.h 标注 V11.1.0+ / DEVELOPMENT BRANCH，未确认上游 commit | 保留各文件头及 [MIT 文本](licenses/FreeRTOS-MIT.txt)；[上游](https://github.com/FreeRTOS/FreeRTOS-Kernel) |
| ST 启动、系统及设备配置文件 | 当前 RTE 文件；使用 STM32F1xx_DFP 2.4.1 | 保留原文件头与 [ST 许可文本](licenses/ST-SLA0044.txt)，注意其针对 ST 器件的使用条件 |
| STM32F1xx Device Pack / 标准外设库 / CMSIS | Keil Pack 管理器安装，版本见构建说明 | 未复制完整 Pack，按供应商许可安装使用 |
| MinimalModbus 2.1.1 | [上游](https://github.com/pyhys/minimalmodbus) | 通过 pip 安装，不在包中分发其源码或 wheel，使用时遵守原许可 |
| pyserial 3.5 | [上游](https://github.com/pyserial/pyserial) | 同上 |

第三方复制文件的 SHA-256 清单见 [third-party-sha256.txt](licenses/third-party-sha256.txt)。该清单固定的是本项目实际使用快照，不等于推定其上游 tag/commit。FreeRTOSConfig.h 是本工程配置，不纳入第三方源码清单。
