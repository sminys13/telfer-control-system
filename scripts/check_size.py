# scripts/check_size.py
# Safe no-op helper. PlatformIO allows "extra_scripts" to point here.
# If you want strict size checks, you can extend this script later.

Import("env")

def _after_build(source, target, env):
    # PlatformIO already prints size info if you run `pio run -v`.
    # Keep this hook minimal to avoid breaking builds on different setups.
    pass

env.AddPostAction("buildprog", _after_build)
