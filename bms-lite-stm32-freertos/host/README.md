# Python 只读主站

建议 Python 3.12，最低 Python 3.10。依赖固定在 requirements.txt：MinimalModbus 2.1.1、pyserial 3.5。

在仓库根目录创建虚拟环境并安装：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r host/requirements.txt
.\.venv\Scripts\python.exe host/modbus_master.py --port COM6 --count 60
```

COM6 仅是示例。省略 --port 可列出并选择串口；--list-ports 只枚举设备；--debug 打印原始收发；--csv 指定新的日志文件，已有文件不会覆盖；--legacy 兼容旧版 22 寄存器映射。

使用 115200/8E1、地址 1，默认每秒读取 36 寄存器。仅 FC03 读取，不写设备。失败行写入 CSV，终端不沿用旧测量值；电流/功率有效性检查失败时显示空值。该程序不自动重写 INA226 校准寄存器。

运行前关闭占用同一串口的串口助手。
