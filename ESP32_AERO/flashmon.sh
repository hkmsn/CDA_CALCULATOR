#!/usr/bin/env zsh

set -e

PORT="${1:-/dev/cu.usbmodem11101}"
BAUD="${2:-115200}"

pio run --target upload --upload-port "$PORT"
sleep 2
exec pio device monitor --port "$PORT" --baud "$BAUD"
