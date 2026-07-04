"""
fitparse_run.py

A specialized utility to inspect and debug Garmin FIT files, with support for 
Connect IQ Developer Fields and human-readable, aligned output.

Description:
    This tool reads FIT activity files and prints recorded data messages.
    It links raw developer data back to Field Names and App UUIDs, 
    calculates local time, and maps common 'unknown' Garmin fields.

Usage:
    python3 test/fitparse_run.py [input_file] [options]

Examples:
    Basic usage:
        python3 test/fitparse_run.py
    
    Specify file and timezone (e.g., EST):
        python3 test/fitparse_run.py path/to/activity.fit --tz -5
    
    Advanced debugging (show IDs and UUIDs):
        python3 test/fitparse_run.py --show-ids --show-uuids

Arguments:
    input           The path to the FIT file (default: session.fit).
    --tz            Timezone offset in hours (default: 7).
    --show-ids      Display field definition numbers (IDs).
    --show-uuids    Display developer application UUIDs and expand column width.

Time (Local):
    Automatically calculated as UTC time + offset. Used to correlate events with 
    the actual time of the activity.
e.g.:

CV                              : 54.959617614746094
accumulated_power               : 965735
cadence                         : 66
distance                        : 68431.14
enhanced_altitude               : -116.80000000000001
enhanced_respiration_rate       : 20.97
enhanced_speed                  : 6.485
fractional_cadence              : 0.0
heart_rate                      : 106
left_pedal_smoothness           : 9.0
left_right_balance              : 196
left_torque_effectiveness       : 34.0
position_lat                    : 94285408
position_long                   : 1173391073
power                           : 0
right_pedal_smoothness          : 15.0
right_torque_effectiveness      : 56.5
temperature                     : 32
Time (Local): 2026-04-14 08:47:20 (UTC: 2026-04-14 01:47:20)
fractional_cadence              : 1
enhanced_speed                  : None
torque_effectiveness            : 79
pedal_smoothness                : 79
performance_condition           : 106

"""
import os
import fitparse
import argparse
from pathlib import Path
import uuid
from datetime import timedelta
# Set FIT_FILE_PATH to look for session.fit in the same directory as this script
FIT_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "session.fit")

print(FIT_FILE_PATH)

# Manual mapping for standard Garmin fields that fitparse might not recognize yet
KNOWN_GARMIN_FIELDS = {
    107: "fractional_cadence",
    134: "enhanced_speed",
    137: "torque_effectiveness",
    138: "pedal_smoothness", 
    # Add airspeed if it's a known standard field ID not automatically recognized
    # Example: 139: "airspeed" 
    144: "performance_condition"
}

def examine_fit_records(file_path: str, tz_offset: int = 7, show_ids: bool = False, show_uuids: bool = False):
    if not Path(file_path).exists():
        print(f"Error: File {file_path} not found.")
        return

    try:
        fitfile = fitparse.FitFile(file_path)
        fitfile.parse()
    except Exception as e:
        print(f"Error parsing {file_path}: {e}")
        return

    # 1. Map developer data indices to UUIDs (from developer_data_id messages)
    dev_apps = {}
    for msg in fitfile.get_messages('developer_data_id'):
        idx = msg.get_value('developer_data_index')
        app_id = msg.get_value('application_id')
        if idx is not None and app_id:
            try:
                # Convert byte arrays/lists to a standard UUID string
                if isinstance(app_id, list): app_id = bytes(app_id)
                if isinstance(app_id, (bytes, bytearray)) and len(app_id) == 16:
                    app_id = str(uuid.UUID(bytes=app_id))
                dev_apps[idx] = app_id
            except Exception:
                dev_apps[idx] = str(app_id)

    # 2. Map field definition numbers to names (from field_description messages)
    dev_field_names = {}
    for msg in fitfile.get_messages('field_description'):
        idx = msg.get_value('developer_data_index')
        f_num = msg.get_value('field_definition_number')
        name = msg.get_value('field_name')
        if idx is not None and f_num is not None:
            dev_field_names[(idx, f_num)] = name

    padding = 65 if show_uuids else 30

    record_count = 0
    for record in fitfile.get_messages("record"):
        record_count += 1
        print("-" * 20)
        for data in record:
            if data.name == 'timestamp' and data.value:
                local_ts = data.value + timedelta(hours=tz_offset)
                print(f"Time (Local): {local_ts} (UTC: {data.value})")
            else:
                if hasattr(data, 'is_developer') and data.is_developer:
                    d_idx = getattr(data, 'developer_data_index', None)
                    f_num = getattr(data, 'def_num', '??')
                    app_id = dev_apps.get(d_idx, "Unknown UUID")
                    name = dev_field_names.get((d_idx, f_num), data.name or "DevField")
                    id_str = f" (ID: {f_num})" if show_ids else ""
                    uuid_str = f" (UUID: {app_id})" if show_uuids else ""
                    label = f"DEV: {name}{id_str}{uuid_str}"
                    print(f"{label:<{padding}}  : {data.value}")
                else:
                    field_id = getattr(data, 'def_num', '??')
                    name = data.name
                    # If the name is unknown, check our manual map
                    if name.startswith("unknown") and field_id in KNOWN_GARMIN_FIELDS:
                        name = KNOWN_GARMIN_FIELDS[field_id]
                    id_str = f" (ID: {field_id})" if show_ids else ""
                    label = f"{str(name)}{id_str}"
                    print(f"{label:<{padding}}  : {data.value}")

    print(f"\nTotal record messages parsed: {record_count}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inspect raw FIT file records.")
    parser.add_argument("input", nargs="?", default=FIT_FILE_PATH, help="Path to the .FIT file")
    parser.add_argument("--tz", type=int, default=7, help="Timezone offset in hours (default: 7)")
    parser.add_argument("--show-ids", action="store_true", help="Display field definition numbers (IDs)")
    parser.add_argument("--show-uuids", action="store_true", help="Display developer application UUIDs")
    
    args = parser.parse_args()
    examine_fit_records(args.input, tz_offset=args.tz, show_ids=args.show_ids, show_uuids=args.show_uuids)
