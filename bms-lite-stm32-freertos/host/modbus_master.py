"""Read-only BMS-Lite master: M5 36-register map, optional M4 legacy mode."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
STACK_NAMES = ('sample', 'display', 'input', 'alarm', 'buzzer', 'uart', 'modbus')
RELAY_STATES = {0: '等待温度', 1: '温度正常', 2: '过温', 3: '温度传感器异常', 4: '采样超时'}
TEMP_STATES = {0: '等待转换', 1: '正常', 2: '未找到', 3: 'CRC错误'}
REGISTER_COUNT = 36
ELECTRICAL_VALID_MASK = 0x7F
ELECTRICAL_BITS = (
    '寄存器读取失败', '校准值不匹配', '启动配置或身份检查失败',
    '芯片运算溢出', 'ADC饱和或电压超量程', '尚无新转换结果', '读取期间转换更新',
)
CSV_FIELDS = [
    'time', 'result', 'error', 'voltage_v', 'temperature_c', 'temperature_raw',
    'alarm', 'temperature_status', 'status_hex', 'relay_on', 'relay_reason',
    'sequence', 'stop_timeouts',
    'heap_now_bytes', 'heap_min_bytes', 'keys', 'page', 'rx_errors',
    'electrical_flags_hex', 'electrical_state', 'current_a', 'power_w',
    'power_hw_w', 'shunt_voltage_mv', 'shunt_raw', 'current_raw',
    'power_raw', 'cal_read', 'mask_start_hex', 'mask_end_hex',
    'cal_expected', 'shunt_uohm', 'map_revision_hex',
    *('stack_' + name + '_units' for name in STACK_NAMES),
    *('reg_' + format(i, '04X') for i in range(REGISTER_COUNT)),
]


def decode_registers(registers):
    if len(registers) not in (22, REGISTER_COUNT):
        raise ValueError('需要22或36个寄存器，实际收到' + str(len(registers)))
    if any(type(value) is not int or not 0 <= value <= 65535 for value in registers):
        raise ValueError('寄存器必须是0～65535的整数')
    r = registers
    raw = r[1] if r[1] < 0x8000 else r[1] - 65536
    # Preserve raw values in CSV, but never present invalid/stale readings as live.
    fresh = (r[4] & 0x09) == 0x09
    voltage_valid = fresh and bool(r[4] & 0x02)
    temperature_valid = fresh and bool(r[4] & 0x04) and r[3] == 1
    result = {
        'voltage_v': r[0] / 1000 if voltage_valid else None,
        'temperature_c': raw / 16 if temperature_valid else None,
        'temperature_raw': raw,
        'alarm': r[2], 'temperature_status': r[3], 'status_hex': f'{r[4]:04X}',
        'sequence': r[5] | (r[6] << 16),
        'stop_timeouts': r[7] | (r[8] << 16),
        'heap_now_bytes': r[9], 'heap_min_bytes': r[10],
        'keys': r[18], 'page': r[19], 'rx_errors': r[20] | (r[21] << 16),
    }
    if r[4] & 0x8000:
        result['relay_on'] = int(bool(r[4] & 0x20))
        result['relay_reason'] = (r[4] >> 6) & 7
    result.update(zip(('stack_' + name + '_units' for name in STACK_NAMES), r[11:18]))
    result.update((f'reg_{i:04X}', value) for i, value in enumerate(r))
    if len(r) == REGISTER_COUNT:
        signed16 = lambda x: x if x < 0x8000 else x - 65536
        def signed32(low, high):
            value = low | (high << 16)
            return value if value < 0x80000000 else value - 0x100000000
        compatible = r[35] in (0x0501, 0x0502) and r[33] == 2048 and r[34] == 10000
        flags_valid = (r[22] & ELECTRICAL_VALID_MASK) == ELECTRICAL_VALID_MASK
        valid = fresh and compatible and flags_valid
        issues = [name for bit, name in enumerate(ELECTRICAL_BITS) if not r[22] & (1 << bit)]
        if not fresh:
            issues.append('快照过期或缺失')
        if not compatible:
            issues.append('寄存器版本/比例不匹配')
        current_raw = signed16(r[24])
        result.update(
            electrical_flags_hex=f'{r[22]:04X}',
            electrical_state='数据有效，负载实测标定待完成' if valid else '；'.join(issues),
            current_a=signed32(r[29], r[30]) / 1_000_000 if valid else None,
            power_w=signed32(r[31], r[32]) / 1_000_000 if valid else None,
            power_hw_w=r[25] * 0.00625 if valid and current_raw >= 0 else None,
            shunt_voltage_mv=signed16(r[23]) * 0.0025 if fresh and r[22] & 1 else None,
            shunt_raw=signed16(r[23]), current_raw=current_raw, power_raw=r[25],
            cal_read=r[26], mask_start_hex=f'{r[27]:04X}', mask_end_hex=f'{r[28]:04X}',
            cal_expected=r[33], shunt_uohm=r[34], map_revision_hex=f'{r[35]:04X}',
        )
    return result


def build_instrument(port, debug=False):
    import minimalmodbus
    import serial

    instrument = minimalmodbus.Instrument(port, 1, mode=minimalmodbus.MODE_RTU)
    try:
        instrument.serial.baudrate = 115200
        instrument.serial.bytesize = serial.EIGHTBITS
        instrument.serial.parity = serial.PARITY_EVEN
        instrument.serial.stopbits = serial.STOPBITS_ONE
        instrument.serial.timeout = 0.5
        instrument.serial.write_timeout = 0.5
        instrument.serial.xonxoff = False
        instrument.serial.rtscts = False
        instrument.serial.dsrdtr = False
        instrument.clear_buffers_before_each_transaction = True
        instrument.handle_local_echo = False
        instrument.close_port_after_each_call = False
        instrument.debug = debug
    except Exception:
        instrument.serial.close()
        raise
    return instrument


def choose_port():
    from serial.tools import list_ports

    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    if not ports:
        raise RuntimeError('未发现串口，请插入USB-RS485后重新运行。')
    print('检测到的串口（名称CH340也可能是USB-RS485）：')
    for index, port in enumerate(ports, 1):
        print(f'  {index}. {port.device}: {port.description}')
    while True:
        value = input('输入适配器的COM名称或上面的序号：').strip()
        if value.isdecimal() and 1 <= int(value) <= len(ports):
            return ports[int(value) - 1].device
        for port in ports:
            if value.casefold() == port.device.casefold():
                return port.device
        print('请选择上面列出的串口；不确定时可拔插适配器确认。')


def print_sample(sample, ok, failed):
    voltage = '--' if sample['voltage_v'] is None else f"{sample['voltage_v']:.3f} V"
    temp = '--' if sample['temperature_c'] is None else f"{sample['temperature_c']:.4f} ℃"
    alarm = {0: '未告警', 1: '告警中'}.get(sample['alarm'], f"未知({sample['alarm']})")
    state = TEMP_STATES.get(sample['temperature_status'], '未知状态')
    print(f"\n[{sample['time']}] 成功 {ok} / 失败 {failed}")
    print(f"  电压 {voltage} | 温度 {temp} | {alarm} | 传感器读取 {state}")
    if 'relay_on' in sample:
        command = '接通' if sample['relay_on'] else '断开'
        reason = RELAY_STATES.get(sample['relay_reason'], '未知原因')
        print(f'  继电器控制指令：{command} | {reason}（非触点反馈）')
    if 'electrical_state' in sample:
        current = '--' if sample['current_a'] is None else f"{sample['current_a']:.6f} A"
        power = '--' if sample['power_w'] is None else f"{sample['power_w']:.6f} W"
        print(f"  电流 {current} | 功率 {power} | 电测状态0x{sample['electrical_flags_hex']}")
        print('  ' + sample['electrical_state'])
    print(f"  序号 {sample['sequence']} | 按键 {sample['keys']} | 页面 {sample['page']} "
          f"| 状态 0x{sample['status_hex']}")
    print(f"  Heap 当前/最低 {sample['heap_now_bytes']}/{sample['heap_min_bytes']} 字节 "
          f"| I2C STOP超时 {sample['stop_timeouts']} | 接收错误 {sample['rx_errors']}")
    stacks = ' '.join(name + '=' + str(sample['stack_' + name + '_units']) for name in STACK_NAMES)
    print('  历史最低剩余栈(单位4字节): ' + stacks)
    if sample['voltage_v'] is None or sample['temperature_c'] is None:
        print('  -- 表示数据无效、等待或已过期；CSV保留原始寄存器供排查。')


def poll(instrument, csv_path, interval=1.0, count=0, register_count=REGISTER_COUNT):
    """One outstanding FC03 request at a time; log errors without cached readings."""
    import minimalmodbus
    import serial

    ok = failed = attempts = 0
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    # Never silently overwrite an earlier experiment.
    with csv_path.open('x', encoding='utf-8-sig', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
        writer.writeheader()
        output.flush()
        print('CSV记录：' + str(csv_path.resolve()))
        try:
            while count == 0 or attempts < count:
                started = time.monotonic()
                stamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
                attempts += 1
                try:
                    registers = instrument.read_registers(0, register_count, functioncode=3)
                    row = decode_registers(registers)
                except (minimalmodbus.ModbusException, serial.SerialException,
                        OSError, ValueError) as error:
                    failed += 1
                    row = {'time': stamp, 'result': 'ERROR', 'error': str(error)}
                    print(f'\n[{stamp}] 通信失败 {failed}（成功 {ok}）：{error}')
                    print('  本次不显示旧数据。检查串口占用、115200/8E1、供电及接线。')
                else:
                    ok += 1
                    row.update(time=stamp, result='OK', error='')
                    print_sample(row, ok, failed)
                writer.writerow(row)
                output.flush()
                if count and attempts >= count:
                    break
                # No catch-up burst after a slow transaction.
                time.sleep(max(0, interval - (time.monotonic() - started)))
        except KeyboardInterrupt:
            print('\n已停止轮询。')
    print(f'本次成功 {ok}，失败 {failed}。CSV已保存。')
    return 0 if failed == 0 else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description='BMS-Lite只读Modbus主站，115200/8E1/地址1/FC03，默认M5的36寄存器')
    parser.add_argument('--port', help='例如COM6；省略则列出串口并选择')
    parser.add_argument('--interval', type=float, default=1.0, help='轮询周期秒，最小0.1，默认1')
    parser.add_argument('--count', type=int, default=0, help='读取次数；默认0持续运行，Ctrl+C停止')
    parser.add_argument('--csv', type=Path, help='CSV新文件路径，已有文件不会覆盖')
    parser.add_argument('--debug', action='store_true', help='打印库的原始收发诊断')
    parser.add_argument('--list-ports', action='store_true', help='仅列串口，不打开设备')
    parser.add_argument('--legacy', action='store_true', help='旧M4固件只读22寄存器')
    args = parser.parse_args(argv)
    if not 0.1 <= args.interval <= 3600 or args.count < 0:
        parser.error('interval需在0.1～3600秒之间，count不能为负数')
    try:
        import minimalmodbus  # noqa: F401
        import serial  # noqa: F401
        from serial.tools import list_ports
    except ImportError:
        print('缺少依赖，请按host/README.md安装requirements.txt。', file=sys.stderr)
        return 1
    if args.list_ports:
        for port in list_ports.comports():
            print(f'{port.device}: {port.description}')
        return 0
    register_count = 22 if args.legacy else REGISTER_COUNT
    print(f'BMS-Lite Python主站 | 115200 / 8E1 | 地址1 | 每次读取{register_count}个寄存器')
    if not args.legacy:
        print('M5：R010测量，精度待验证；支持继电器温度保护演示。')
    print('请先关闭STC-ISP等串口助手的串口；Ctrl+C停止。只读取，不写设备。')
    instrument = None
    try:
        port = args.port or choose_port()
        csv_path = args.csv or ROOT / 'logs' / ('modbus_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
        # Fail before opening a device if the requested log already exists.
        if csv_path.exists():
            raise FileExistsError('日志文件已存在，请换一个文件名：' + str(csv_path))
        instrument = build_instrument(port, args.debug)
        return poll(instrument, csv_path, args.interval, args.count, register_count)
    except KeyboardInterrupt:
        print('\n已取消。')
        return 0
    except Exception as error:
        print('无法启动：' + str(error), file=sys.stderr)
        print('串口被占用时，请先关闭其他串口软件；确认选择的是USB-RS485。', file=sys.stderr)
        return 1
    finally:
        if instrument is not None:
            instrument.serial.close()


if __name__ == '__main__':
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8', errors='replace')
    raise SystemExit(main())
