# scripts/check_size.py
# Печатает размер прошивки и RAM после линковки, и валит сборку только если RAM > 100%.
# Работает на CI и локально.

Import("env")
import re
import subprocess

MCU = env.BoardConfig().get("build.mcu", "atmega2560")

def _run_size(elf_path: str) -> str:
    # avr-size гарантированно есть в toolchain PlatformIO для atmelavr
    cmd = ["avr-size", "-C", f"--mcu={MCU}", elf_path]
    return subprocess.check_output(cmd, universal_newlines=True, stderr=subprocess.STDOUT)

def _parse_data_percent(text: str):
    # Ищем строку вида: "Data:       6758 bytes (82.5% Full)"
    m = re.search(r"Data:\s+(\d+)\s+bytes\s+\(([\d.]+)%\s+Full\)", text)
    if not m:
        return None
    used_bytes = int(m.group(1))
    used_percent = float(m.group(2))
    return used_bytes, used_percent

def after_build(source, target, env):
    elf = str(target[0])
    try:
        out = _run_size(elf)
        print("\n" + out.strip() + "\n")

        parsed = _parse_data_percent(out)
        if parsed:
            used_bytes, used_percent = parsed
            if used_percent > 100.0:
                print(f"ERROR: RAM overflow! Data = {used_bytes} bytes ({used_percent}%).")
                env.Exit(1)
            elif used_percent > 95.0:
                print(f"WARNING: RAM is high: Data = {used_bytes} bytes ({used_percent}%).")
    except Exception as e:
        # Не ломаем сборку из-за парсинга — просто предупреждаем.
        print(f"WARNING: check_size.py failed: {e}")

# Хук после линковки ELF
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", after_build)