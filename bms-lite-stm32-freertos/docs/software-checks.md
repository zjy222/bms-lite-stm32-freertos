# 打包后的软件检查

2026-10-19，在独立上传目录内执行以下检查，未更改原始实测固件逻辑。

| 检查 | 结果 |
| --- | --- |
| Keil AC5 / STM32F1xx_DFP 2.4.1 全量重建 | 0 错误、0 警告 |
| 固件大小（字节） | Code 21096，RO 1788，RW 240，ZI 12264 |
| 实际继电器策略与寄存器提取测试 | 20 个场景通过 |
| 实际 I²C 清理、转换、寄存器提取测试 | 36 个场景通过 |
| 实际 RTU 分帧、协议、发送提取测试 | 20 个场景通过 |
| Python 主站测试（真实 MinimalModbus + 模拟串口） | 17 项通过 |

合计 93 个软件测试场景/用例通过。主机测试使用 GCC C90、-Wall、-Wextra、-Werror，检查实际源码提取片段；不表示完整固件已用 GCC 构建，也不证明物理信号时序或线上通信可靠性。

firmware/main.c 与实物验证版本字节一致，SHA-256：

```text
e9d4407411ab93a609e4b50ca62d669ce4826b54de711caa8f5ad179188ceb92
```

测试入口是 tools/run_tests.py。GitHub Actions 配置已附带，但整理时未推送远程，因此尚无本仓库线上运行结果。

离线图可用 `python -m pip install matplotlib` 后执行 `python tools/plot_demo.py` 重建。它只依赖随包的实测 CSV，失败行和无效字段不作补值。
