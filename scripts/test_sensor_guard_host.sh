#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/sensor_guard_v6_test"
g++ -std=c++11 -Wall -Wextra -Werror \
  -I"$ROOT/include" \
  "$ROOT/src/sensor_guard_v6.cpp" \
  "$ROOT/test/test_sensor_guard_v6.cpp" \
  -o "$OUT"
"$OUT"
