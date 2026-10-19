"""Run host tests against the packaged firmware; requires GCC and host dependencies."""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
for script in ('test_relay_source.py', 'test_m5_source.py', 'test_rtu_source.py'):
    subprocess.run([sys.executable, str(root / 'tests' / script)], cwd=root, check=True)
subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_pc_master.py', '-v'], cwd=root, check=True)
