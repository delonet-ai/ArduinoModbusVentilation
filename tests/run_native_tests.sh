#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
storage_test_binary="$(mktemp /tmp/arduino-ventilation-storage-tests.XXXXXX)"
airflow_test_binary="$(mktemp /tmp/arduino-ventilation-airflow-tests.XXXXXX)"
trap 'rm -f "$storage_test_binary" "$airflow_test_binary"' EXIT

g++ -std=c++11 -Wall -Wextra -Werror \
  -I"$repo_root/tests/fakes" \
  -I"$repo_root/ArduinoModbusVentilation" \
  "$repo_root/tests/PersistentTargetsTest.cpp" \
  "$repo_root/ArduinoModbusVentilation/PersistentTargets.cpp" \
  -o "$storage_test_binary"

"$storage_test_binary"

g++ -std=c++11 -Wall -Wextra -Werror \
  -I"$repo_root/tests/fakes" \
  -I"$repo_root/ArduinoModbusVentilation" \
  "$repo_root/tests/AirflowGuardTest.cpp" \
  "$repo_root/ArduinoModbusVentilation/AirflowGuard.cpp" \
  -o "$airflow_test_binary"

"$airflow_test_binary"
