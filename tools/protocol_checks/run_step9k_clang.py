"""Clang path analysis with native I/O stubs, including actual web command code."""
from pathlib import Path
import argparse
import shlex
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--zig', required=True)
parser.add_argument('--output', required=True)
args = parser.parse_args()
repo = Path(__file__).resolve().parents[2]
zig = str(Path(args.zig).resolve())
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
web = ['-DV6_WEB_CONTROL_ENABLED=1', '-DV6_VFD_RS485_ENABLED=1',
       '-DV6_VFD_WRITE_COMMANDS_ENABLED=1', '-DV6_HE200_FIELD_SERVICE=1',
       '-DV6_HE200_NATIVE_PROTOCOL=1', '-DV6_AUTO_PHYSICAL_ENABLED=1',
       '-DV6_SAFETY_BENCH_MODE=0', '-DV6_DWIN_MOTION_ENABLED=0']
cases = [('test/diagnostic/test_web_commands.cpp', web)] + [
    (source, []) for source in ['src/auto_runner_v6.cpp', 'src/program_v6.cpp',
                               'src/modbus.cpp', 'src/he200_audit_v6.cpp']]
logs = []
failed = False
for source, defines in cases:
    trace = subprocess.run([zig, 'c++', '-###', '-std=c++17', '-Itest/diagnostic',
                            '-Iinclude', *defines, '-c', source],
                           cwd=repo, text=True, capture_output=True)
    line = next(s.strip() for s in trace.stderr.splitlines() if '"-cc1"' in s)
    command = shlex.split(line)
    command[command.index('-emit-obj')] = '-analyze'
    index = command.index('-o')
    del command[index:index+2]
    command += ['-analyzer-output=text', '-analyzer-checker=core,deadcode']
    result = subprocess.run(command, cwd=repo, text=True, capture_output=True)
    log = result.stdout + result.stderr
    logs.append(f'{source}: exit={result.returncode}\n{log}')
    print(source, 'exit=', result.returncode)
    print('\n'.join(s for s in log.splitlines() if 'warning:' in s or 'error:' in s))
    failed |= bool(result.returncode or 'warning:' in log or 'error:' in log)
(out / 'clang-analysis.log').write_text('\n'.join(logs), encoding='utf-8')
raise SystemExit(failed)
