#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
test_binary="$(mktemp /tmp/arduino-ventilation-tests.XXXXXX)"
trap 'rm -f "$test_binary"' EXIT

g++ -std=c++11 -Wall -Wextra -Werror \
  -I"$repo_root/tests/fakes" \
  -I"$repo_root/ArduinoModbusVentilation" \
  "$repo_root/tests/PersistentTargetsTest.cpp" \
  "$repo_root/ArduinoModbusVentilation/PersistentTargets.cpp" \
  -o "$test_binary"

"$test_binary"
