"""
device_smoke_test.py

An automated validation tool to verify build integrity and runtime stability across 
all supported Garmin devices defined in the project manifest.

Description:
    This script automates the 'smoke testing' process by:
    1. Resolving the Connect IQ SDK path from VS Code settings.
    2. Validating the SDK installation integrity (checking api.db format).
    3. Identifying all target devices from manifest.xml.
    4. Compiling the app for each device using the Java-based monkeybrains compiler.
    5. Launching the Connect IQ Simulator and side-loading the app via monkeydo.
    6. Monitoring the simulator for a set period to detect early runtime crashes.

    It provides a summary report at the end, highlighting incompatible SDKs, 
    missing device definitions, build errors, or runtime failures.

Usage:
    python3 test/device_smoke_test.py

Prerequisites:
    - Connect IQ SDK installed and path set in .vscode/settings.json.
    - Java Runtime Environment (JRE) installed and available in PATH.
    - Developer Key (developer_key.der) in the project root.

Configuration:
    - DEEP_CLEAN: If True, deletes bin and gen folders before every device test.
    - SIM_WAIT_TIME: Adjust this value to change how long the script waits 
      to detect a crash (default: 8 seconds).
    - ROOT_DIR: Automatically calculated relative to this script's location.
"""
import os
import subprocess
import time
import xml.etree.ElementTree as ET
import sys
import signal
from pathlib import Path
import re
import shutil
import json

# --- Configuration ---
# Use absolute paths relative to the script location for reliability
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)

def find_developer_key():
    """Locates the developer key, handling common naming variations."""
    # Garmin toolchain and Java compiler strictly prefer .der
    # We look for .der first to avoid using an extensionless version that might fail in Java
    patterns = ["developer_key.der", "developer_key"]
    for name in patterns:
        path = os.path.join(ROOT_DIR, name)
        if os.path.exists(path):
            # Ensure we return the absolute path to avoid Java CWD confusion
            return os.path.abspath(path)
    # Fallback to the expected standard name
    return os.path.join(ROOT_DIR, "developer_key.der")

DEV_KEY = find_developer_key()
MANIFEST_PATH = os.path.join(ROOT_DIR, "manifest.xml")
JUNGLE_PATH = os.path.join(ROOT_DIR, "monkey.jungle")
OUTPUT_PRG = os.path.join(ROOT_DIR, "bin", "smoke_test.prg")
SIM_WAIT_TIME = 8  # Seconds to let the app run and check for crashes
DEEP_CLEAN = True  # Recommended for System 6 devices (Edge 540/1040)

def get_sdk_bin_path():
    """
    Reads the SDK path from VS Code settings to ensure consistency between CLI and IDE.
    
    Handles JSON with comments and trailing commas common in VS Code settings.
    Returns:
        Tuple[str, str]: (Path to SDK bin directory, Description of the source).
    """
    settings_path = os.path.join(ROOT_DIR, ".vscode", "settings.json")
    if not os.path.exists(settings_path):
        return "", f"System Environment (PATH) - settings.json not found at {settings_path}"

    try:
        with open(settings_path, 'r') as f:
            content = f.read()
            # Strip comments (// and /* */)
            content = re.sub(r'//.*?\n|/\*.*?\*/', '', content, flags=re.S)
            # Strip trailing commas
            content = re.sub(r',\s*([\]}])', r'\1', content)
            
            data = json.loads(content)
            
            # 1. Try top-level first
            path = data.get("monkeyC.sdkBinPath") or data.get("monkeyc.sdkBinPath")
            
            # 2. Search in common VS Code nested structures (like your python-envs.pythonProjects)
            if not path and "python-envs.pythonProjects" in data:
                projects = data["python-envs.pythonProjects"]
                if isinstance(projects, list):
                    for project in projects:
                        if isinstance(project, dict):
                            path = project.get("monkeyC.sdkBinPath") or project.get("monkeyc.sdkBinPath")
                            if path: break
            
            # Validate settings path - if it doesn't exist, treat as not found to allow fallbacks
            if path and not os.path.exists(path):
                print(f"  [DEBUG] Path from settings.json does not exist: {path}")
                path = None

            # 3. macOS "Smart Discovery" - Look in the default Garmin SDK location
            if not path and sys.platform == "darwin":
                default_base = os.path.expanduser("~/Library/Application Support/Garmin/ConnectIQ/Sdks")
                if os.path.exists(default_base):
                    sdks = sorted([d for d in os.listdir(default_base) if os.path.isdir(os.path.join(default_base, d))], reverse=True)
                    if sdks:
                        path = os.path.join(default_base, sdks[0], "bin")
                        return path, f"Auto-Detected macOS SDK ({sdks[0]})"
                system_path = shutil.which("monkeyc")
                if system_path:
                    path = os.path.dirname(os.path.abspath(system_path))
                    return path, f"System PATH ({path})"

            if path:
                if os.path.exists(path):
                    # If the path points directly to a file (like 'monkeyc'), use its containing directory
                    if os.path.isfile(path):
                        path = os.path.dirname(path)
                    return path, f"VS Code Settings ({path})"
                else:
                    return "", f"!!! WARNING: Path in settings.json NOT FOUND on disk: {path}"
            return "", "FALLBACK to System Path (Reason: 'monkeyC.sdkBinPath' key missing in settings.json)"
    except Exception as e:
        return "", f"FALLBACK to System Path (Reason: JSON Error parsing settings.json: {e})"
    return "", "System Environment (PATH)"

# Resolve Tool Paths
SDK_BIN_PATH, SDK_SOURCE = get_sdk_bin_path()
MONKEYC = os.path.join(SDK_BIN_PATH, "monkeyc") if SDK_BIN_PATH else "monkeyc"
MONKEYDO = os.path.join(SDK_BIN_PATH, "monkeydo") if SDK_BIN_PATH else "monkeydo"
CONNECTIQ = os.path.join(SDK_BIN_PATH, "connectiq") if SDK_BIN_PATH else "connectiq"
MONKEYBRAINS_JAR = os.path.join(SDK_BIN_PATH, "monkeybrains.jar") if SDK_BIN_PATH else "monkeybrains.jar"

def get_devices():
    """
    Parses manifest.xml to find all supported product IDs.
    Returns:
        List[str]: A list of device IDs (e.g., ['edge1040', 'edge540']).
    """
    try:
        tree = ET.parse(MANIFEST_PATH)
        root = tree.getroot()
        ns = {'iq': 'http://www.garmin.com/xml/connectiq'}
        products = root.findall('.//iq:product', ns)
        return [p.get('id') for p in products if p.get('id')]
    except Exception as e:
        print(f"Error parsing manifest: {e}")
        return []

def kill_simulator():
    """
    Force closes the Connect IQ Simulator process based on the operating system.
    Used to ensure a clean slate before testing a new device.
    """
    if sys.platform == "darwin": # macOS
        subprocess.run(["pkill", "-f", "ConnectIQ"], stderr=subprocess.DEVNULL)
    elif sys.platform == "win32": # Windows
        subprocess.run(["taskkill", "/F", "/IM", "simulator.exe"], stderr=subprocess.DEVNULL)
    time.sleep(2)

def get_sdk_info():
    """
    Determines the current Connect IQ SDK version, API level, and installation health.
    
    Returns:
        Tuple[str, str, str]: (Display string, Absolute compiler path, Raw version string).
    """
    sdk_compiler_version = "Unknown"
    sdk_version = "Unknown"
    sdk_api_level = "Unknown"
    db_status = ""
    sdk_root = os.path.dirname(SDK_BIN_PATH) if SDK_BIN_PATH else ""

    # Ensure the SDK bin directory is in the PATH for this subprocess call too
    env_patch = {"PATH": f"{SDK_BIN_PATH}:{os.environ.get('PATH', '')}"} if SDK_BIN_PATH else {}

    try:
        # 1. Get compiler version from monkeyc --version
        ver_res = subprocess.run([MONKEYC, "--version"], capture_output=True, text=True, check=False, timeout=5, env=env_patch)
        if ver_res.returncode == 0:
            match = re.search(r"Connect IQ Compiler version: ([\d.]+)", ver_res.stdout)
            if match:
                sdk_compiler_version = match.group(1)
            else:
                print(f"DEBUG: Could not parse compiler version from: {ver_res.stdout.strip()}")
        else:
            print(f"DEBUG: monkeyc --version failed. Return code: {ver_res.returncode}")
            print(f"DEBUG: stdout: {ver_res.stdout.strip()}")
            print(f"DEBUG: stderr: {ver_res.stderr.strip()}")

        # 2. Read the SDK version. Newer SDK packages may omit sdk.version, but
        # bin/version.txt is part of the compiler distribution.
        if SDK_BIN_PATH:
            version_file = os.path.join(sdk_root, "sdk.version")
            fallback_version_file = os.path.join(SDK_BIN_PATH, "version.txt")
            for candidate in (version_file, fallback_version_file):
                if os.path.exists(candidate):
                    with open(candidate, 'r') as f:
                        sdk_version = f.read().strip()
                    break
            else:
                sdk_version = sdk_compiler_version

        # 3. Check api.db status. Despite its name, api.db is a small text
        # symbol map, not a SQLite database. Its first line is the maximum API
        # level supported by the SDK (for example, "6.0.0").
        api_db_path = os.path.join(SDK_BIN_PATH, "api.db")
        if os.path.exists(api_db_path):
            size_kb = os.path.getsize(api_db_path) / 1024
            try:
                with open(api_db_path, 'r', encoding='utf-8') as f:
                    first_line = f.readline().strip()
                if re.fullmatch(r"\d+\.\d+\.\d+", first_line):
                    sdk_api_level = first_line
                    db_status = f" | api.db: {size_kb:.0f}KB [OK]"
                else:
                    db_status = f" | api.db: {size_kb:.0f}KB [!] INVALID FORMAT"
            except (OSError, UnicodeError) as e:
                db_status = f" | [!] api.db UNREADABLE ({e})"
        else:
            db_status = " | [!] api.db MISSING"

        if "[!]" in db_status:
            db_status += " -> REINSTALL REQUIRED"

        # Device definitions are installed globally by Garmin's SDK Manager on
        # macOS, rather than under each individual SDK.
        device_dirs = [
            os.path.join(sdk_root, "share", "devices"),
            os.path.expanduser("~/Library/Application Support/Garmin/ConnectIQ/Devices"),
        ]
        if not any(os.path.isdir(path) for path in device_dirs):
            db_status += " | [!] device definitions MISSING"

        display_version = f"Compiler: {sdk_compiler_version} (SDK: {sdk_version}, API Level: {sdk_api_level}){db_status}"
        return display_version, os.path.abspath(MONKEYC), sdk_compiler_version # Return compiler version as raw info

    except Exception as e:
        print(f"DEBUG: Exception in get_sdk_info: {e}")
        return f"Error: {str(e)}", str(MONKEYC), "Unknown"

def run_command(cmd, description, env_extra=None):
    """
    Runs a shell command and captures its output.
    Args:
        cmd (List[str]): The command and arguments.
        description (str): Human-readable name of the task.
        env_extra (dict): Extra environment variables (e.g., PATH) to merge.
    Returns:
        Tuple[bool, str]: (Success status, Error output if failed).
    """
    print(f"  > {description}...")
    
    # Merge environment variables to ensure monkeyc can find its sibling tools
    current_env = os.environ.copy()
    if env_extra:
        current_env.update(env_extra)
    
    result = subprocess.run(cmd, capture_output=True, text=True, env=current_env, cwd=ROOT_DIR)
    if result.returncode != 0:
        return False, result.stderr.strip()
    return True, ""

def smoke_test_device(device_id, sdk_display_ver, sdk_raw_ver):
    """
    Performs a build and runtime check for a specific device.
    Args:
        device_id (str): The Garmin device ID.
        sdk_display_ver (str): Formatted SDK info.
        sdk_raw_ver (str): Raw compiler version.
    Returns:
        str: Result status (e.g., SUCCESS, BUILD_ERROR, RUNTIME_CRASH).
    """
    # 0. Early check for modern devices on old SDKs
    modern_devices = ["edge540", "edge840", "edge1040", "fr265", "fr965", "fenix7"]
    if device_id in modern_devices and sdk_raw_ver.startswith("5."):
        print(f"\n--- Testing Device: {device_id} ---")
        print(f"  [SKIPPED] Potential API Mismatch.")
        print(f"  HINT: {device_id} usually requires SDK 6.0.0+. Current SDK is {sdk_raw_ver}.")
        return "SDK_OUTDATED"

    # 1. Verify device metadata exists on this machine
    # Garmin devices are usually in ~/Library/Application Support/Garmin/ConnectIQ/Devices/
    device_config_path = os.path.expanduser(f"~/Library/Application Support/Garmin/ConnectIQ/Devices/{device_id}")
    if not os.path.exists(device_config_path):
        print(f"\n--- Testing Device: {device_id} ---")
        print(f"  [ERROR] Device definition not found at: {device_config_path}")
        print(f"  HINT: Download this device using the Connect IQ SDK Manager.")
        return "DEVICE_MISSING"

    print(f"\n--- Testing Device: {device_id} ---")
    
    # 1.5 Deep Clean to prevent "Undefined symbol :Layouts" errors
    if DEEP_CLEAN:
        for folder in ["bin", "gen", "source/gen"]:
            folder_path = os.path.join(ROOT_DIR, folder)
            if os.path.exists(folder_path):
                print(f"  > Cleaning {folder}...")
                shutil.rmtree(folder_path, ignore_errors=True)

    # 1.7 Verify monkeybrains.jar exists
    if not os.path.exists(MONKEYBRAINS_JAR):
        print(f"\n--- Testing Device: {device_id} ---")
        print(f"  [ERROR] Cannot find monkeybrains.jar at: {MONKEYBRAINS_JAR}")
        print(f"  HINT: Check your SDK installation or .vscode/settings.json path.")
        return "SDK_ERROR"

    # 2. Compile
    build_cmd = [
        "java",
        "-Xms1g", 
        "-Xmx2g", # Increase max heap space for System 6 devices
        "-Dfile.encoding=UTF-8",
        "-Dapple.awt.UIElement=true",
        "-jar", MONKEYBRAINS_JAR,
        "-o", OUTPUT_PRG,
        "-f", JUNGLE_PATH,
        "-d", f"{device_id}_sim",
        "-y", DEV_KEY,
        "-w"
    ]
    
    # Ensure the SDK bin directory is in the PATH so monkeyc can find api.db
    env_patch = {"PATH": f"{SDK_BIN_PATH}:{os.environ.get('PATH', '')}"} if SDK_BIN_PATH else {}
    
    success, error_output = run_command(build_cmd, "Compiling", env_extra=env_patch)
    if not success:
        if "requires API Level" in error_output:
            print(f"  [SKIPPED] {device_id} is incompatible with this SDK version.")
            print(f"  Reason: {error_output.strip()}")
            
            # Extract the reported API Level from the compiler's error message
            match = re.search(r"supports up to API Level '([\d.]+)'", error_output)
            compiler_reported_api_level = match.group(1) if match else "Unknown"

            print(f"  HINT: The compiler (version {sdk_raw_ver} from {os.path.basename(MONKEYC)}) is reporting API Level support up to '{compiler_reported_api_level}', but the device requires API Level '6.0.0'.")
            print(f"        This indicates a shallow or corrupted SDK installation. Reinstall steps:")
            print(f"        1. Open the Connect IQ SDK Manager.")
            print(f"        2. Delete and re-download the {sdk_raw_ver} SDK.")
            print(f"        3. Verify that 'api.db' is readable and begins with an API version.")
            print(f"        5. Check macOS 'Full Disk Access' for the SDK Manager in System Settings.")
            return "SDK_INCOMPATIBLE"
        
        if "A critical error has occurred" in error_output:
            print(f"  [CRITICAL COMPILER CRASH] The Monkey C compiler crashed for {device_id}.")
            print(f"  --- FULL COMPILER LOG ---\n{error_output}\n-------------------------")
        else:
            print(f"  [ERROR] Compiling failed: {error_output[:500]}") 
        return "BUILD_ERROR"

    # 3. Ensure Simulator is running
    # Note: connectiq usually needs to be started manually or via background process
    # We attempt to launch it if not visible
    subprocess.Popen([CONNECTIQ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=ROOT_DIR)
    time.sleep(2)

    # 4. Run in Simulator
    try:
        run_proc = subprocess.Popen(
            [MONKEYDO, OUTPUT_PRG, device_id],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=ROOT_DIR
        )
        
        # Wait to see if it stays alive or crashes
        time.sleep(SIM_WAIT_TIME)
        
        if run_proc.poll() is not None:
            # Process exited early, likely a crash
            _, stderr = run_proc.communicate()
            print(f"  [CRASH] Simulator exited early for {device_id}")
            print(f"  Logs: {stderr}")
            return "RUNTIME_CRASH"
        
        print(f"  [SUCCESS] {device_id} loaded successfully.")
        run_proc.terminate()
        return "SUCCESS"

    except Exception as e:
        print(f"  [EXCEPTION] {e}")
        return "EXCEPTION"
    finally:
        # Clean up the PRG after each run to prevent the VS Code Language Server
        # from attempting to index a file that is rapidly changing.
        if os.path.exists(OUTPUT_PRG):
            try:
                os.remove(OUTPUT_PRG)
            except Exception:
                pass

def main():
    """
    Main execution loop. Validates prerequisites, iterates through devices, 
    and prints the final summary report.
    """
    # Verify critical files exist before starting the suite
    for path, name in [(DEV_KEY, "Developer Key"), (MANIFEST_PATH, "Manifest"), (JUNGLE_PATH, "Jungle file")]:
        if not os.path.exists(path):
            abs_path = os.path.abspath(path)
            print(f"\n[!] MISSING CRITICAL FILE: {name}")
            print(f"    Expected at: {abs_path}")
            parent_dir = os.path.dirname(abs_path)
            if os.path.exists(parent_dir):
                contents = os.listdir(parent_dir)
                print(f"    Directory contents of {parent_dir}:")
                for item in contents:
                    print(f"      - {item}")
            return

    devices = get_devices()
    if not devices:
        print("No devices found to test.")
        return

    sdk_display, sdk_path, sdk_raw_info = get_sdk_info()
    
    # Abort if SDK tools are unusable
    if not os.path.exists(MONKEYBRAINS_JAR) or "Error" in sdk_display:
        print(f"\n[FATAL ERROR] SDK tools not found or corrupted.")
        print(f"Expected monkeybrains.jar at: {MONKEYBRAINS_JAR}")
        print(f"HINT: Ensure 'monkeyC.sdkBinPath' is correctly set in .vscode/settings.json")
        return

    print("="*30)
    print(f"SMOKE TEST STARTING")
    print(f"SDK Version: {sdk_display}")
    print(f"SDK Path:    {sdk_path}")
    print(f"MONKEYC:     {MONKEYC}")
    print(f"MONKEYDO:    {MONKEYDO}")
    print(f"SDK Source:  {SDK_SOURCE}")
    print("="*30)

    results = {}
    os.makedirs("bin", exist_ok=True)

    try:
        for device in devices:
            # We kill and restart to ensure a clean slate for resource loading
            kill_simulator()
            status = smoke_test_device(device, sdk_display, sdk_raw_info)
            results[device] = status
    except KeyboardInterrupt:
        print("\n\n[!] Smoke test interrupted by user. Cleaning up...")
    finally:
        # Ensure the simulator is closed on exit
        kill_simulator()

    print("\n" + "="*30)
    print("SMOKE TEST SUMMARY")
    print("="*30)
    for dev, res in results.items():
        print(f"{dev:<20}: {res}")

if __name__ == "__main__":
    main()
