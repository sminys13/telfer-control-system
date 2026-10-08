"""Run production C++ logic against fake serial, EEPROM and motor I/O; no COM opens."""
from pathlib import Path
import argparse
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--zig', required=True, help='Path to Zig C++ compiler')
parser.add_argument('--output', required=True, help='Directory for binaries and logs')
args = parser.parse_args()
repo = Path(__file__).resolve().parents[2]
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
common = [str(Path(args.zig).resolve()), 'c++', '-std=c++17', '-Wall', '-Wextra',
          '-Werror', '-Wno-unused-variable', '-nostdlib++', '-fno-exceptions',
          '-fno-rtti', '-Itest/diagnostic', '-Iinclude']
web = ['-DV6_WEB_CONTROL_ENABLED=1', '-DV6_VFD_RS485_ENABLED=1',
       '-DV6_VFD_WRITE_COMMANDS_ENABLED=1', '-DV6_HE200_FIELD_SERVICE=1',
       '-DV6_HE200_NATIVE_PROTOCOL=1', '-DV6_AUTO_PHYSICAL_ENABLED=1',
       '-DV6_SAFETY_BENCH_MODE=0', '-DV6_DWIN_MOTION_ENABLED=0']
cases = [
    ('policy', ['test/diagnostic/test_web_policy.cpp'], []),
    ('commands', ['test/diagnostic/test_web_commands.cpp', 'src/program_v6.cpp'], web),
    ('auto', ['test/diagnostic/test_auto_simulation.cpp', 'src/auto_runner_v6.cpp', 'src/program_v6.cpp'], []),
    ('diagnostic', ['test/diagnostic/test_diagnostic.cpp', 'src/modbus.cpp', 'src/he200_audit_v6.cpp'], []),
]
failed = False
for name, sources, defines in cases:
    binary = out / (name + ('.exe' if sys.platform == 'win32' else ''))
    command = common + defines + sources + ['-o', str(binary)]
    build = subprocess.run(command, cwd=repo, capture_output=True, text=True)
    (out / (name + '-build.log')).write_text(build.stdout + build.stderr, encoding='utf-8')
    if build.returncode:
        print(name, 'BUILD FAIL', build.stderr)
        failed = True
        continue
    run = subprocess.run([str(binary)], cwd=repo, capture_output=True, text=True)
    (out / (name + '-tests.log')).write_text(run.stdout + run.stderr, encoding='utf-8')
    print(name, 'exit=' + str(run.returncode), run.stdout.strip(), run.stderr.strip())
    failed |= run.returncode != 0
raise SystemExit(failed)
