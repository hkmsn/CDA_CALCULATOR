Import("env")

from SCons.Script import Exit

import os
import signal
import subprocess
import time

from serial.tools import list_ports


ESPRESSIF_USB_VID = 0x303A


def _resolve_upload_port(env):
    candidates = [
        device.device
        for device in list_ports.comports()
        if device.vid == ESPRESSIF_USB_VID
        and device.device.startswith("/dev/cu.")
    ]
    if not candidates:
        print(
            "UPLOAD STOPPED: No powered Espressif USB device found "
            "(expected VID 303A)."
        )
        print("The unrelated nRF52 serial port will not be used.")
        Exit(1)
    if len(candidates) != 1:
        print("UPLOAD STOPPED: Multiple Espressif USB devices found.")
        print("Set upload_port explicitly to select the intended board.")
        Exit(1)

    replacement = candidates[0]
    print(f"Using detected Espressif upload port {replacement}")
    env.Replace(UPLOAD_PORT=replacement)
    return replacement


def _port_holders(port):
    result = subprocess.run(
        ["lsof", "-t", port],
        capture_output=True,
        text=True,
        check=False,
    )
    return {int(pid) for pid in result.stdout.split() if pid.isdigit()}


def _command(pid):
    result = subprocess.run(
        ["ps", "-p", str(pid), "-o", "command="],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip()


def release_upload_port(source, target, env):
    if env.GetProjectOption("upload_protocol", "") != "esptool":
        return

    port = _resolve_upload_port(env)

    project_dir = env.subst("$PROJECT_DIR")
    holders = []
    for pid in _port_holders(port):
        command = _command(pid)
        is_monitor = (
            "monitor" in command
            and ("platformio" in command or "/pio device monitor" in command)
        )
        is_project_esptool = "esptool.py" in command and project_dir in command
        if is_monitor or is_project_esptool:
            holders.append(pid)
            print(f"Releasing {port} from PID {pid}: {command}")
            os.kill(pid, signal.SIGTERM)

    if not holders:
        return

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        remaining = set(holders) & _port_holders(port)
        if not remaining:
            time.sleep(1.0)
            return
        time.sleep(0.1)

    for pid in set(holders) & _port_holders(port):
        print(f"Forcing stale process {pid} to release {port}")
        os.kill(pid, signal.SIGKILL)

env.AddPreAction("upload", release_upload_port)
