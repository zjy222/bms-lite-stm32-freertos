@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
  echo 请先按 README 创建 .venv 并安装 host/requirements.txt。
  exit /b 1
)
.venv\Scripts\python.exe host\modbus_master.py %*
