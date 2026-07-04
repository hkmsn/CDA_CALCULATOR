import os
try:
    import fitparse
except ImportError:
    print("Error: 'fitparse' library not found. Please install it using 'pip install fitparse'")
    exit(1)

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import timedelta

# Set FIT_FILE_PATH to look for session.fit in the same directory as this script
FIT_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "session.fit")

def parse_fit_file(file_path):
    """Parses a FIT file and extracts timestamp and CV data."""
    try:

        fitfile = fitparse.FitFile(file_path)
    except FileNotFoundError:
        print(f"Error: FIT file not found at {file_path}")
        return [], []

    timestamps = []
    cda_values = []

    print("Scanning FIT file records...")
    for record in fitfile.get_messages("record"):
        timestamp = None
        cda = None
        for data in record:
            if data.name == 'timestamp' and data.value is not None:
                timestamp = data.value
            elif data.name in ['CdA', 'cda'] and data.value is not None:
                cda = data.value
            elif cda is None and ('developer_field' in str(data.name) or 'unknown' in str(data.name)):
                # Fallback for developer fields if names aren't resolved
                cda = data.value

        # Filter out invalid CdA values (<= 0) often caused by coasting or simulator defaults
        if timestamp is not None and cda is not None and cda > 0:
            timestamps.append(timestamp)
            cda_values.append(cda)

    return timestamps, cda_values

def create_dataframe(timestamps, cda_values):
    """Creates a Pandas DataFrame from timestamps and CdA values."""
    df = pd.DataFrame({'timestamp': timestamps, 'cda': cda_values})
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    df.set_index('timestamp', inplace=True)
    return df

def aggregate_data(df, interval='15Min'):
    """Aggregates data into 15-minute blocks."""
    # Calculate the mean CdA for each block and return as a DataFrame
    return df.resample(interval)['cda'].mean().dropna().to_frame()

def plot_cv_data(df, filename='cda_plot.png'):
    """Plots CdA data over time and saves the plot to a file."""
    if df.empty:
        print("No data available to plot.")
        return

    plt.figure(figsize=(12, 6))
    # Plot raw data points with reduced transparency
    plt.plot(df.index, df['cda'], marker='.', linestyle='-', markersize=2, linewidth=0.5, alpha=0.3, label='Raw CdA')
    
    # Add a smoothing curve (1-minute rolling average)
    smoothed_cda = df['cda'].rolling(window='1Min').mean()
    plt.plot(df.index, smoothed_cda, color='red', linewidth=1.5, label='1-Min Moving Average')

    plt.title('CdA Over Time (Raw vs Smoothed)')
    plt.xlabel('Session Time (Start and End)')
    plt.ylabel('CdA (m^2)')
    plt.ylim(0, 1.0)

    # Set X-axis to show only start and end times
    ax = plt.gca()
    ax.set_xticks([df.index[0], df.index[-1]])
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))

    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.xticks(rotation=45)
    plt.legend()
    plt.tight_layout()

    plt.savefig(filename)  # Save the plot to a file
    plt.close()  # Close the plot to free memory
    print(f"CdA plot saved to {filename}")


def main():
    timestamps, cda_values = parse_fit_file(FIT_FILE_PATH)

    if not timestamps or not cda_values:
        print("No timestamp or CdA data found in the FIT file.")
        return

    df = create_dataframe(timestamps, cda_values)
    plot_cv_data(df)

if __name__ == "__main__":
    main()