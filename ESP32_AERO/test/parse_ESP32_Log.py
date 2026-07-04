"""
parse_Esp32_Log.py

A utility to inspect and debug ESP32 Aero logs (pipe-delimited text files).

Description:
    This tool reads the aero_log.txt file and prints recorded data messages 
    record by record with aligned attribute labels.

Usage:
    python3 test/parse_Esp32_Log.py
"""
import pandas as pd
import os
import sys

def parse_aero_log(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        return

    try:
        # Read pipe-delimited file. index_col=False handles trailing delimiters.
        # na_values='NA' ensures GroundSpeed 'NA' values are handled as nulls.
        df = pd.read_csv(file_path, sep='|', engine='python', index_col=False, na_values='NA')

        # Remove columns that are entirely empty (common with trailing delimiters)
        df = df.dropna(axis=1, how='all')

        # Calculate padding for aligned output
        padding = max(len(col.strip()) for col in df.columns) + 1 if not df.empty else 20

        for _, row in df.iterrows():
            print("********")
            for column in df.columns:
                attr = column.strip()
                val = str(row[column]).strip()
                print(f"{attr:<{padding}}: {val}")
        
        if not df.empty:
            print("********")

    except Exception as e:
        print(f"An error occurred while reading the file: {e}")

if __name__ == "__main__":
    # Use provided argument or default to aero_log.txt
    if len(sys.argv) > 1:
        log_path = sys.argv[1]
    else:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        log_path = os.path.join(base_dir, 'aero_log.txt')
    
    print(f"Parsing: {log_path}")
    parse_aero_log(log_path)
