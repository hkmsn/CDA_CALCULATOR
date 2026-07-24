#!/usr/bin/env python3
"""Hardware-in-the-loop resilience tests for ESP32_AERO.

These tests use the firmware's normal serial telemetry, discovery messages, and
``I`` status command. Tests which require removing hardware pause and tell the
operator exactly when to perform the action.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Callable

try:
    import serial
except ImportError:  # Keep parser unit tests importable without pyserial.
    serial = None


DEFAULT_PORT = "/dev/cu.usbmodem11101"
TELEMETRY_FIELD_COUNT = 12
HEALTH_RE = re.compile(
    r"Health:\s*writeFailures=(\d+)\s+failovers=(\d+)\s+"
    r"logDrops=(\d+)\s+queueDrops=(\d+)\s+i2cRecoveries=(\d+)\s+"
    r"imuRecoveries=(\d+)\s+truncations=(\d+)"
)
TELEMETRY_RE = re.compile(
    r"(?:\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{2}|"
    r"\d{2}:\d{2}:\d{2}:\d{2})\|([^*\r\n]+)\*"
)

HARDWARE_PROFILES = {
    "capsule-all": {
        "airspeed": "present", "bmp": "present",
        "temperature": "present", "imu": "present",
    },
    "capsule-bare": {
        "airspeed": "absent", "bmp": "absent",
        "temperature": "absent", "imu": "present",
    },
    "stamp-generic": {
        "airspeed": "either", "bmp": "either",
        "temperature": "either", "imu": "absent",
    },
    "devkit-bare": {
        "airspeed": "absent", "bmp": "absent",
        "temperature": "absent", "imu": "absent",
    },
}


@dataclass
class Capture:
    text: str = ""
    telemetry: list[list[str]] = field(default_factory=list)
    health: list[dict[str, int]] = field(default_factory=list)


def parse_capture(text: str) -> Capture:
    result = Capture(text=text)
    for match in TELEMETRY_RE.finditer(text):
        fields = [value.strip() for value in match.group(1).split("|")]
        if len(fields) == TELEMETRY_FIELD_COUNT:
            result.telemetry.append(fields)
    names = (
        "writeFailures", "failovers", "logDrops", "queueDrops",
        "i2cRecoveries", "imuRecoveries", "truncations",
    )
    for match in HEALTH_RE.finditer(text):
        result.health.append(dict(zip(names, map(int, match.groups()))))
    return result


class Monitor:
    def __init__(self, port: str, baud: int):
        if serial is None:
            raise RuntimeError("pyserial is required: python3 -m pip install pyserial")
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.1)
        self.serial.dtr = False
        self.serial.rts = False

    def close(self) -> None:
        self.serial.close()

    def send(self, command: str) -> None:
        self.serial.write(command.encode("ascii"))
        self.serial.flush()

    def capture(self, seconds: float, command: str | None = None) -> Capture:
        if command:
            self.send(command)
        deadline = time.monotonic() + seconds
        chunks: list[bytes] = []
        while time.monotonic() < deadline:
            data = self.serial.read(self.serial.in_waiting or 1)
            if data:
                chunks.append(data)
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()
        return parse_capture(b"".join(chunks).decode("utf-8", errors="replace"))

    def status(self, seconds: float = 3.0) -> Capture:
        self.serial.reset_input_buffer()
        return self.capture(seconds, "I")


def prompt(message: str, non_interactive: bool) -> None:
    if non_interactive:
        print(f"ACTION REQUIRED: {message}")
        return
    input(f"\nACTION: {message}\nPress Enter when ready... ")


def latest_health(capture: Capture) -> dict[str, int]:
    if not capture.health:
        raise AssertionError("No health status received after sending I")
    return capture.health[-1]


def require_telemetry(capture: Capture, minimum: int = 1) -> None:
    if len(capture.telemetry) < minimum:
        raise AssertionError(
            f"Expected at least {minimum} complete telemetry records; "
            f"received {len(capture.telemetry)}"
        )


def field_present(capture: Capture, index: int) -> bool:
    return any(row[index] != "NA" for row in capture.telemetry)


def check_expectation(name: str, actual: bool, expected: str) -> None:
    if expected == "either":
        return
    wanted = expected == "present"
    if actual != wanted:
        raise AssertionError(f"{name}: expected {expected}, observed " +
                             ("present" if actual else "absent"))


def sensor_matrix(monitor: Monitor, args: argparse.Namespace) -> None:
    print("Capturing telemetry for sensor-combination validation...")
    capture = monitor.capture(args.duration)
    require_telemetry(capture, 2)
    # Telemetry indexes: airspeed, density, temperature, pressure, altitude,
    # altitude change, ground speed, climb, forward acceleration, accel XYZ.
    expected = dict(HARDWARE_PROFILES.get(args.profile, {}))
    for name in ("airspeed", "bmp", "temperature", "imu"):
        explicit = getattr(args, name)
        if explicit != "either" or name not in expected:
            expected[name] = explicit
    check_expectation("airspeed", field_present(capture, 0), expected["airspeed"])
    check_expectation("barometer", field_present(capture, 3), expected["bmp"])
    check_expectation("temperature", field_present(capture, 2), expected["temperature"])
    check_expectation("IMU", field_present(capture, 9), expected["imu"])


def i2c_interruption(monitor: Monitor, args: argparse.Namespace) -> None:
    before = latest_health(monitor.status())
    prompt("Disconnect or hold low the EXTERNAL I2C sensor bus.", args.non_interactive)
    fault = monitor.capture(args.fault_duration)
    prompt("Reconnect/release the external I2C bus.", args.non_interactive)
    recovered = monitor.capture(args.recovery_duration)
    after = latest_health(monitor.status())
    require_telemetry(recovered, 2)
    if after["i2cRecoveries"] <= before["i2cRecoveries"]:
        raise AssertionError("No I2C recovery was recorded during the interruption")
    if "Booting ESP32-S3" in fault.text + recovered.text:
        raise AssertionError("Firmware rebooted during I2C interruption")


def sd_removal(monitor: Monitor, args: argparse.Namespace) -> None:
    baseline = monitor.status()
    if "Active Storage: SD Card" not in baseline.text:
        raise AssertionError("Test must start with a mounted SD card")
    prompt("Remove the microSD card while the device remains powered.", args.non_interactive)
    monitor.capture(args.flush_wait)
    after = monitor.status()
    health = latest_health(after)
    if "Active Storage: Internal Flash" not in after.text:
        raise AssertionError("Firmware did not fail over from SD to LittleFS")
    if health["failovers"] < 1:
        raise AssertionError("SD removal did not increment the failover counter")
    require_telemetry(monitor.capture(3), 1)


def sd_full(monitor: Monitor, args: argparse.Namespace) -> None:
    baseline = monitor.status()
    if "Active Storage: SD Card" not in baseline.text:
        raise AssertionError("Insert a mounted, nearly-full SD card before this test")
    prompt("Confirm the SD card has less than one log-buffer of free space.",
           args.non_interactive)
    monitor.capture(args.flush_wait)
    after = monitor.status()
    health = latest_health(after)
    if health["writeFailures"] < 1:
        raise AssertionError("A full-card write failure was not observed")
    if "Active Storage: Internal Flash" not in after.text:
        raise AssertionError("Full SD card did not cause LittleFS failover")


def corrupt_filesystem(monitor: Monitor, args: argparse.Namespace) -> None:
    prompt(
        "Boot with no SD card and a deliberately unmountable LittleFS partition, "
        "then reconnect this test if USB disconnects.", args.non_interactive,
    )
    capture = monitor.capture(args.duration, "I")
    if "Booting ESP32-S3" in capture.text and capture.text.count("Booting ESP32-S3") > 1:
        raise AssertionError("Repeated reboot detected with corrupt filesystem")
    require_telemetry(capture, 1)
    # Current firmware should explicitly report no storage. Keeping this strict
    # makes the known misleading 'Internal Flash' status visible as a test failure.
    if "Active Storage: None" not in capture.text:
        raise AssertionError("Firmware did not explicitly report unavailable storage")


def queue_saturation(monitor: Monitor, args: argparse.Namespace) -> None:
    before = latest_health(monitor.status())
    print(f"Running {args.duration:.0f}s queue/serial/BLE soak...")
    capture = monitor.capture(args.duration)
    require_telemetry(capture, max(2, int(args.duration / 4)))
    after = latest_health(monitor.status())
    delta = after["queueDrops"] - before["queueDrops"]
    if delta > args.max_queue_drops:
        raise AssertionError(
            f"Queue dropped {delta} samples; allowed maximum is {args.max_queue_drops}"
        )
    if "Booting ESP32-S3" in capture.text:
        raise AssertionError("Firmware rebooted during queue soak")


SCENARIOS: dict[str, Callable[[Monitor, argparse.Namespace], None]] = {
    "sensor-matrix": sensor_matrix,
    "i2c-interruption": i2c_interruption,
    "sd-removal": sd_removal,
    "sd-full": sd_full,
    "corrupt-filesystem": corrupt_filesystem,
    "queue-saturation": queue_saturation,
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=SCENARIOS)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--fault-duration", type=float, default=8)
    parser.add_argument("--recovery-duration", type=float, default=15)
    parser.add_argument("--flush-wait", type=float, default=35)
    parser.add_argument("--max-queue-drops", type=int, default=0)
    parser.add_argument("--non-interactive", action="store_true")
    parser.add_argument("--profile", choices=HARDWARE_PROFILES)
    for name in ("airspeed", "bmp", "temperature", "imu"):
        parser.add_argument(
            f"--{name}", choices=("present", "absent", "either"), default="either"
        )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    monitor: Monitor | None = None
    try:
        monitor = Monitor(args.port, args.baud)
        print(f"Connected to {args.port} at {args.baud} baud")
        SCENARIOS[args.scenario](monitor, args)
    except (AssertionError, RuntimeError, serial.SerialException if serial else OSError) as exc:
        print(f"\nFAIL [{args.scenario}]: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nABORTED", file=sys.stderr)
        return 130
    finally:
        if monitor is not None:
            monitor.close()
    print(f"\nPASS [{args.scenario}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
