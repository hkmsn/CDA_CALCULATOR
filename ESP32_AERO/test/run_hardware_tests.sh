#!/usr/bin/env zsh

set -eu

PROJECT_DIR="${0:A:h:h}"
TEST_PY="$PROJECT_DIR/test/hardware_resilience_test.py"
PORT="${1:-/dev/cu.usbmodem11101}"

python3 "$TEST_PY" sensor-matrix --port "$PORT" \
  --airspeed present --bmp present --temperature present --imu present
python3 "$TEST_PY" queue-saturation --port "$PORT" --duration 60

print "Automated baseline tests passed."
print "Run removal/fault scenarios individually; see test/README."
