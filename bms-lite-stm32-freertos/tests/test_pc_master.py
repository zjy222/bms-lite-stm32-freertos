"""Host tests use the real MinimalModbus library and a fake serial endpoint."""
import contextlib
import csv
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import minimalmodbus
import serial

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bms_master', ROOT / 'host' / 'modbus_master.py')
master = importlib.util.module_from_spec(spec)
spec.loader.exec_module(master)

# Actual validated firmware response: CRC 39 79, 49 bytes.
RESPONSE = bytes.fromhex(
    '01 03 2C 0C F8 01 95 00 00 00 01 00 1F 00 8C 00 00 00 00 00 00 '
    '03 70 03 70 00 B8 00 B6 00 62 00 72 00 62 00 60 00 B8 00 00 '
    '00 01 00 00 00 00 39 79'
)
REGISTERS = [int.from_bytes(RESPONSE[i:i+2], 'big') for i in range(3, 47, 2)]


class FakeSerial:
    port = 'TEST_ONLY_NO_HARDWARE'
    is_open = True
    baudrate = 115200
    timeout = 0.5

    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []
        self.current = b''

    def open(self): self.is_open = True
    def close(self): self.is_open = False
    def reset_input_buffer(self): self.current = b''
    def reset_output_buffer(self): pass

    def write(self, data):
        self.requests.append(data)
        self.current = self.responses.pop(0)
        return len(data)

    def read(self, count):
        result, self.current = self.current[:count], self.current[count:]
        return result


class MasterTests(unittest.TestCase):
    def test_actual_frame_and_settings(self):
        port = FakeSerial([RESPONSE])
        instrument = master.build_instrument(port)
        self.assertEqual((port.baudrate, port.bytesize, port.parity, port.stopbits), (115200, 8, 'E', 1))
        self.assertFalse(port.rtscts)
        self.assertFalse(port.xonxoff)
        self.assertEqual(instrument.read_registers(0, 22, 3), REGISTERS)
        self.assertEqual(port.requests, [bytes.fromhex('01 03 00 00 00 16 C4 04')])

    def test_decode_actual_snapshot(self):
        sample = master.decode_registers(REGISTERS)
        self.assertEqual(sample['voltage_v'], 3.320)
        self.assertEqual(sample['temperature_c'], 25.3125)
        self.assertEqual(sample['heap_now_bytes'], 880)
        self.assertEqual(sample['sequence'], 140)
        self.assertEqual(sample['stack_modbus_units'], 184)
        self.assertEqual(sample['reg_0015'], 0)

    def test_signed_temperature_and_counter_halves(self):
        r = REGISTERS.copy()
        r[1] = 65536 - 160
        r[5:9] = [0xFFFF, 1, 0x1234, 2]
        r[20:22] = [0x5678, 3]
        sample = master.decode_registers(r)
        self.assertEqual(sample['temperature_c'], -10)
        self.assertEqual(sample['sequence'], 131071)
        self.assertEqual(sample['stop_timeouts'], 0x21234)
        self.assertEqual(sample['rx_errors'], 0x35678)

    def test_invalid_or_stale_data_hidden(self):
        for flag in (0, 1, 0x17):
            r = REGISTERS.copy()
            r[4] = flag
            sample = master.decode_registers(r)
            self.assertIsNone(sample['voltage_v'])
            self.assertIsNone(sample['temperature_c'])
        r = REGISTERS.copy()
        r[3] = 3
        sample = master.decode_registers(r)
        self.assertIsNone(sample['temperature_c'])
        self.assertEqual(sample['temperature_raw'], 405)

    def test_invalid_register_input(self):
        for invalid in ([], [0]*21, [0]*23, [0]*21 + [-1], [0]*21 + [65536], [0]*21 + [True]):
            with self.assertRaises(ValueError): master.decode_registers(invalid)

    def test_crc_timeout_and_recovery_csv(self):
        corrupted = RESPONSE[:-1] + bytes([RESPONSE[-1] ^ 1])
        port = FakeSerial([RESPONSE, corrupted, b'', RESPONSE])
        instrument = master.build_instrument(port)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'capture.csv'
            with contextlib.redirect_stdout(io.StringIO()), patch.object(master.time, 'sleep'):
                code = master.poll(instrument, path, interval=1, count=4, register_count=22)
            self.assertEqual(code, 2)
            with path.open(encoding='utf-8-sig', newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([r['result'] for r in rows], ['OK', 'ERROR', 'ERROR', 'OK'])
            self.assertEqual(rows[0]['reg_0001'], '405')
            self.assertEqual(rows[1]['temperature_c'], '')
            self.assertEqual(rows[2]['voltage_v'], '')
            self.assertTrue(rows[1]['error'])
            self.assertTrue(rows[2]['error'])
            self.assertEqual(len(port.requests), 4)

    def test_existing_csv_not_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'capture.csv'
            path.write_text('keep', encoding='utf-8')
            with self.assertRaises(FileExistsError): master.poll(None, path, count=1)
            self.assertEqual(path.read_text(), 'keep')

    def test_main_closes_port(self):
        port = FakeSerial([RESPONSE])
        instrument = master.build_instrument(port)
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(master, 'build_instrument', return_value=instrument), contextlib.redirect_stdout(io.StringIO()):
                result = master.main(['--port', 'FAKE', '--legacy', '--count', '1', '--csv', str(Path(folder) / 'ok.csv')])
        self.assertEqual(result, 0)
        self.assertFalse(port.is_open)

    def test_m5_real_library_full_read(self):
        response = bytes.fromhex(
            '01 03 48 2E E0 01 95 00 00 00 01 00 1F 00 8C 00 00 00 00 00 00 '
            '03 70 03 70 00 B8 00 B6 00 62 00 72 00 62 00 60 00 B8 00 00 '
            '00 01 00 00 00 00 00 7F 06 40 06 40 03 00 08 00 00 08 00 00 '
            '1A 80 00 06 3E 00 00 49 08 00 27 10 05 01 27 2D')
        port = FakeSerial([response])
        instrument = master.build_instrument(port)
        regs = instrument.read_registers(0, 36, 3)
        self.assertEqual(port.requests, [bytes.fromhex('01 03 00 00 00 24 45 D1')])
        sample = master.decode_registers(regs)
        self.assertEqual(sample['current_a'], 0.4)
        self.assertEqual(sample['power_w'], 4.8)
        self.assertAlmostEqual(sample['power_hw_w'], 4.8)
        self.assertEqual(sample['shunt_voltage_mv'], 4.0)
        self.assertEqual(sample['cal_read'], 2048)

    def electrical_registers(self):
        return REGISTERS + [127, 1600, 1600, 768, 2048, 8, 0,
                            400000 & 65535, 400000 >> 16,
                            4800000 & 65535, 4800000 >> 16, 2048, 10000, 0x0501]

    def test_m5_negative_current_and_power(self):
        regs = self.electrical_registers()
        regs[23] = regs[24] = 65536 - 1600
        for start, value in ((29, -400000), (31, -4800000)):
            regs[start] = value & 65535
            regs[start + 1] = (value >> 16) & 65535
        sample = master.decode_registers(regs)
        self.assertEqual(sample['current_a'], -0.4)
        self.assertEqual(sample['power_w'], -4.8)
        self.assertIsNone(sample['power_hw_w'])
        self.assertEqual(sample['shunt_voltage_mv'], -4)

    def test_m5_invalid_flags_hidden(self):
        for bit in range(7):
            regs = self.electrical_registers()
            regs[22] &= ~(1 << bit)
            sample = master.decode_registers(regs)
            self.assertIsNone(sample['current_a'])
            self.assertIsNone(sample['power_w'])
            self.assertTrue(sample['electrical_state'])

    def test_m5_stale_and_unknown_scale_hidden(self):
        for field, value in ((4, 0x17), (33, 0), (34, 5000), (35, 0x9999)):
            regs = self.electrical_registers()
            regs[field] = value
            sample = master.decode_registers(regs)
            self.assertIsNone(sample['current_a'])
            self.assertIsNone(sample['power_w'])

    def test_m5_valid_zero_is_not_a_read_error(self):
        regs = self.electrical_registers()
        for index in (23, 24, 25, 29, 30, 31, 32):
            regs[index] = 0
        sample = master.decode_registers(regs)
        self.assertEqual(sample['current_a'], 0)
        self.assertEqual(sample['power_w'], 0)

    def test_m5_poll_csv(self):
        class Instrument:
            def read_registers(inner, start, count, functioncode):
                self.assertEqual((start, count, functioncode), (0, 36, 3))
                return self.electrical_registers()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'm5.csv'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(master.poll(Instrument(), path, count=1), 0)
            with path.open(encoding='utf-8-sig', newline='') as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row['current_a'], '0.4')
            self.assertEqual(row['reg_0023'], str(0x0501))

    def test_relay_revision_keeps_current_scale_and_decodes_command(self):
        regs = self.electrical_registers()
        regs[35] = 0x0502
        regs[4] |= 0x8000 | 0x20 | (1 << 6)
        sample = master.decode_registers(regs)
        self.assertEqual(sample['current_a'], 0.4)
        self.assertEqual(sample['relay_on'], 1)
        self.assertEqual(sample['relay_reason'], 1)
        with contextlib.redirect_stdout(io.StringIO()) as output:
            master.print_sample(dict(sample, time='TEST'), 1, 0)
        self.assertIn('继电器控制指令：接通', output.getvalue())
        self.assertIn('非触点反馈', output.getvalue())

    def test_relay_stale_snapshot_still_reports_off_command(self):
        regs = self.electrical_registers()
        regs[35] = 0x0502
        regs[4] = 0x8000 | 0x17 | (4 << 6)
        sample = master.decode_registers(regs)
        self.assertIsNone(sample['voltage_v'])
        self.assertIsNone(sample['current_a'])
        self.assertEqual((sample['relay_on'], sample['relay_reason']), (0, 4))

    def test_legacy_does_not_invent_relay_feedback(self):
        self.assertNotIn('relay_on', master.decode_registers(REGISTERS))



if __name__ == '__main__':
    unittest.main()
