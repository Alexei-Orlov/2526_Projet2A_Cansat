#!/usr/bin/env python3
"""
Telemetry Data Analysis Script
Plots all telemetry fields vs timestamp with flight phase visualization
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
import sys
import os

# Flight phase flag definitions (matching STM32 main.c)
FLAG_GO_FOR_LAUNCH = 0x01  # Bit 0
FLAG_ASCENSION = 0x02      # Bit 1
FLAG_DROP = 0x04           # Bit 2
FLAG_RECOVERY = 0x08       # Bit 3

def decode_flight_phase(flags_raw):
    """
    Decode flags_raw byte into flight phase name
    
    Args:
        flags_raw (int or float): Raw flags byte from telemetry
    Returns:
        str: Flight phase name
    """
    # Handle NaN or empty values
    if pd.isna(flags_raw):
        return 'NO DATA'
    
    # Convert to int (in case it's a float)
    try:
        flags_raw = int(flags_raw)
    except (ValueError, TypeError):
        return 'INVALID'
    
    if flags_raw == 0:
        return 'CALIBRATING'
    elif flags_raw == FLAG_GO_FOR_LAUNCH:
        return 'GO FOR LAUNCH'
    elif flags_raw == (FLAG_GO_FOR_LAUNCH | FLAG_ASCENSION):
        return 'ASCENSION'
    elif flags_raw == (FLAG_GO_FOR_LAUNCH | FLAG_ASCENSION | FLAG_DROP):
        return 'DROP'
    elif flags_raw == (FLAG_GO_FOR_LAUNCH | FLAG_ASCENSION | FLAG_DROP | FLAG_RECOVERY):
        return 'RECOVERY'
    else:
        return f'UNKNOWN (0x{flags_raw:02X})'

def get_phase_color(phase):
    """Get color for flight phase"""
    colors = {
        'CALIBRATING': '#95a5a6',      # Gray
        'GO FOR LAUNCH': '#3498db',    # Blue
        'ASCENSION': '#2ecc71',        # Green
        'DROP': '#e74c3c',             # Red
        'RECOVERY': '#f39c12'          # Orange
    }
    return colors.get(phase, '#ecf0f1')

def get_phase_transitions(df):
    """
    Get timestamp ranges for each flight phase
    
    Args:
        df (DataFrame): Telemetry data
    Returns:
        list: List of (phase_name, start_time, end_time) tuples
    """
    if 'flags_raw' not in df.columns:
        return []
    
    # Remove rows with NaN flags
    df_valid = df.dropna(subset=['flags_raw']).copy()
    
    if len(df_valid) == 0:
        return []
    
    # Decode all phases
    df_valid['phase'] = df_valid['flags_raw'].apply(decode_flight_phase)
    
    # Find phase transitions
    phase_changes = df_valid['phase'].ne(df_valid['phase'].shift())
    phase_starts = df_valid[phase_changes].index
    
    transitions = []
    for i, idx in enumerate(phase_starts):
        phase = df_valid.loc[idx, 'phase']
        start_time = df_valid.loc[idx, 'timestamp']
        
        # Find end time (next phase start or end of data)
        if i < len(phase_starts) - 1:
            end_time = df_valid.loc[phase_starts[i + 1], 'timestamp']
        else:
            end_time = df_valid['timestamp'].max()
        
        transitions.append((phase, start_time, end_time))
    
    return transitions

def add_phase_backgrounds(ax, phase_transitions, y_limits):
    """
    Add colored background rectangles for each flight phase
    
    Args:
        ax: Matplotlib axis
        phase_transitions: List of (phase, start, end) tuples
        y_limits: (ymin, ymax) for rectangle height
    """
    for phase, start, end in phase_transitions:
        color = get_phase_color(phase)
        rect = Rectangle((start, y_limits[0]), end - start, y_limits[1] - y_limits[0],
                         facecolor=color, alpha=0.15, zorder=0)
        ax.add_patch(rect)

def add_calibration_area(ax, df, y_limits):
    """
    Add gray calibration area before first data point
    
    Args:
        ax: Matplotlib axis
        df: DataFrame with telemetry data
        y_limits: (ymin, ymax) for rectangle height
    """
    if len(df) == 0:
        return
    
    first_timestamp = df['timestamp'].min()
    
    # Add calibration area from 0 to first_timestamp
    if first_timestamp > 0:
        rect = Rectangle((0, y_limits[0]), first_timestamp, y_limits[1] - y_limits[0],
                         facecolor='#95a5a6', alpha=0.2, zorder=0)
        ax.add_patch(rect)
        
        # Add "CALIBRATION" label
        mid_time = first_timestamp / 2
        mid_y = (y_limits[0] + y_limits[1]) / 2
        ax.text(mid_time, mid_y, 'CALIBRATION', 
                horizontalalignment='center', verticalalignment='center',
                fontsize=10, fontweight='bold', alpha=0.5, rotation=90)

def plot_telemetry_data(csv_filename, output_dir=None):
    """
    Create comprehensive telemetry plots from CSV file
    
    Args:
        csv_filename (str): Path to CSV file
        output_dir (str): Directory to save plots (None = display only)
    """
    # Load data
    print(f"Loading data from {csv_filename}...")
    df = pd.read_csv(csv_filename)
    
    print(f"Loaded {len(df)} data points")
    print(f"Time range: {df['timestamp'].min():.2f}s - {df['timestamp'].max():.2f}s")
    print(f"Duration: {df['relative_time'].max():.2f}s")
    
    # Get phase transitions
    phase_transitions = get_phase_transitions(df)
    print(f"\nFlight phases detected:")
    for phase, start, end in phase_transitions:
        print(f"  {phase:15s}: {start:7.2f}s - {end:7.2f}s ({end-start:5.2f}s)")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 20))
    fig.suptitle(f'Telemetry Analysis - {os.path.basename(csv_filename)}', 
                 fontsize=16, fontweight='bold')
    
    # Define plots (field_name, ylabel, subplot_index)
    plots = [
        # IMU Data
        (['accel_x', 'accel_y', 'accel_z'], 'Acceleration (m/s^2)', 1),
        (['gyro_x', 'gyro_y', 'gyro_z'], 'Angular Velocity (deg/s)', 2),
        (['roll', 'pitch', 'yaw'], 'Orientation (deg)', 3),
        
        # Environmental
        (['temperature'], 'Temperature (C)', 4),
        (['altitude'], 'Altitude (m)', 5),
        (['vertical_speed'], 'Vertical Speed (m/s)', 6),
        
        # GPS
        (['latitude'], 'Latitude (deg)', 7),
        (['longitude'], 'Longitude (deg)', 8),
        (['satellites'], 'GPS Satellites (count)', 9),
        
        # Power & Signal
        (['battery_voltage'], 'Battery Voltage (V)', 10),
        (['rssi'], 'RSSI (dBm)', 11),
        (['snr'], 'SNR (dB)', 12),
    ]
    
    for fields, ylabel, subplot_idx in plots:
        ax = plt.subplot(6, 2, subplot_idx)
        
        # Plot data for each field
        for field in fields:
            if field in df.columns:
                # Remove NaN values
                plot_df = df[['timestamp', field]].dropna()
                if len(plot_df) > 0:
                    ax.plot(plot_df['timestamp'], plot_df[field], 
                           label=field, linewidth=1.5, marker='o', markersize=2)
        
        # Get y-limits for phase backgrounds
        y_limits = ax.get_ylim()
        
        # Add calibration area
        add_calibration_area(ax, df, y_limits)
        
        # Add phase backgrounds
        add_phase_backgrounds(ax, phase_transitions, y_limits)
        
        # Formatting
        ax.set_xlabel('Timestamp (s)', fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        
        # Set x-axis to start from 0
        ax.set_xlim(left=0)
    
    # Adjust layout
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    
    # Save or display
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        base_name = os.path.splitext(os.path.basename(csv_filename))[0]
        output_file = os.path.join(output_dir, f'{base_name}_analysis.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {output_file}")
    else:
        plt.show()

def generate_flight_report(csv_filename, output_file=None):
    """
    Generate a text report with flight statistics
    
    Args:
        csv_filename (str): Path to CSV file
        output_file (str): Path to save report (None = print to console)
    """
    df = pd.read_csv(csv_filename)
    
    # Decode phases (only for valid flags)
    if 'flags_raw' in df.columns:
        df_valid = df.dropna(subset=['flags_raw']).copy()
        if len(df_valid) > 0:
            df_valid['phase'] = df_valid['flags_raw'].apply(decode_flight_phase)
            # Merge back to original dataframe
            df['phase'] = None
            df.loc[df_valid.index, 'phase'] = df_valid['phase']
    
    report_lines = []
    report_lines.append("="*80)
    report_lines.append(f"FLIGHT TELEMETRY REPORT: {os.path.basename(csv_filename)}")
    report_lines.append("="*80)
    report_lines.append("")
    
    # Overall statistics
    report_lines.append("OVERALL STATISTICS")
    report_lines.append("-"*80)
    report_lines.append(f"Total Data Points:     {len(df)}")
    report_lines.append(f"Start Timestamp:       {df['timestamp'].min():.3f} s")
    report_lines.append(f"End Timestamp:         {df['timestamp'].max():.3f} s")
    report_lines.append(f"Flight Duration:       {df['relative_time'].max():.3f} s")
    report_lines.append(f"Data Rate:             {len(df) / df['relative_time'].max():.2f} Hz")
    report_lines.append("")
    
    # Altitude statistics
    report_lines.append("ALTITUDE STATISTICS")
    report_lines.append("-"*80)
    report_lines.append(f"Starting Altitude:     {df['altitude'].iloc[0]:.2f} m")
    report_lines.append(f"Maximum Altitude:      {df['altitude'].max():.2f} m")
    report_lines.append(f"Final Altitude:        {df['altitude'].iloc[-1]:.2f} m")
    report_lines.append(f"Altitude Gain:         {df['altitude'].max() - df['altitude'].iloc[0]:.2f} m")
    report_lines.append("")
    
    # Vertical speed statistics
    if 'vertical_speed' in df.columns:
        report_lines.append("VERTICAL SPEED STATISTICS")
        report_lines.append("-"*80)
        report_lines.append(f"Maximum Ascent Rate:   {df['vertical_speed'].max():.2f} m/s")
        report_lines.append(f"Maximum Descent Rate:  {df['vertical_speed'].min():.2f} m/s")
        report_lines.append("")
    
    # GPS statistics
    if 'satellites' in df.columns:
        report_lines.append("GPS STATISTICS")
        report_lines.append("-"*80)
        report_lines.append(f"Average Satellites:    {df['satellites'].mean():.1f}")
        report_lines.append(f"Min Satellites:        {df['satellites'].min()}")
        report_lines.append(f"Max Satellites:        {df['satellites'].max()}")
        
        # GPS fix quality
        gps_no_fix = (df['satellites'] == 0).sum()
        gps_poor = ((df['satellites'] >= 1) & (df['satellites'] <= 3)).sum()
        gps_good = ((df['satellites'] >= 4) & (df['satellites'] <= 5)).sum()
        gps_excellent = (df['satellites'] >= 6).sum()
        
        total_packets = len(df)
        report_lines.append(f"No Fix (0 sats):       {gps_no_fix:4d} ({100*gps_no_fix/total_packets:5.1f}%)")
        report_lines.append(f"Poor (1-3 sats):       {gps_poor:4d} ({100*gps_poor/total_packets:5.1f}%)")
        report_lines.append(f"Good (4-5 sats):       {gps_good:4d} ({100*gps_good/total_packets:5.1f}%)")
        report_lines.append(f"Excellent (6+ sats):   {gps_excellent:4d} ({100*gps_excellent/total_packets:5.1f}%)")
        report_lines.append("")
    
    # Battery statistics
    if 'battery_voltage' in df.columns:
        report_lines.append("BATTERY STATISTICS")
        report_lines.append("-"*80)
        report_lines.append(f"Starting Voltage:      {df['battery_voltage'].iloc[0]:.2f} V")
        report_lines.append(f"Ending Voltage:        {df['battery_voltage'].iloc[-1]:.2f} V")
        report_lines.append(f"Voltage Drop:          {df['battery_voltage'].iloc[0] - df['battery_voltage'].iloc[-1]:.2f} V")
        report_lines.append(f"Average Voltage:       {df['battery_voltage'].mean():.2f} V")
        report_lines.append("")
    
    # Signal quality statistics
    if 'rssi' in df.columns and 'snr' in df.columns:
        report_lines.append("SIGNAL QUALITY STATISTICS")
        report_lines.append("-"*80)
        report_lines.append(f"Average RSSI:          {df['rssi'].mean():.1f} dBm")
        report_lines.append(f"Min RSSI:              {df['rssi'].min():.1f} dBm")
        report_lines.append(f"Max RSSI:              {df['rssi'].max():.1f} dBm")
        report_lines.append(f"Average SNR:           {df['snr'].mean():.1f} dB")
        report_lines.append(f"Min SNR:               {df['snr'].min():.1f} dB")
        report_lines.append(f"Max SNR:               {df['snr'].max():.1f} dB")
        
        # Signal quality classification
        rssi_excellent = (df['rssi'] >= -60).sum()
        rssi_good = ((df['rssi'] >= -80) & (df['rssi'] < -60)).sum()
        rssi_fair = ((df['rssi'] >= -100) & (df['rssi'] < -80)).sum()
        rssi_poor = (df['rssi'] < -100).sum()
        
        report_lines.append(f"Excellent (>=-60 dBm):  {rssi_excellent:4d} ({100*rssi_excellent/total_packets:5.1f}%)")
        report_lines.append(f"Good (-60 to -80):     {rssi_good:4d} ({100*rssi_good/total_packets:5.1f}%)")
        report_lines.append(f"Fair (-80 to -100):    {rssi_fair:4d} ({100*rssi_fair/total_packets:5.1f}%)")
        report_lines.append(f"Poor (<-100 dBm):      {rssi_poor:4d} ({100*rssi_poor/total_packets:5.1f}%)")
        report_lines.append("")
    
    # Flight phase breakdown
    if 'phase' in df.columns:
        report_lines.append("FLIGHT PHASE BREAKDOWN")
        report_lines.append("-"*80)
        
        phase_transitions = get_phase_transitions(df)
        for phase, start, end in phase_transitions:
            duration = end - start
            packet_count = len(df[(df['timestamp'] >= start) & (df['timestamp'] < end)])
            report_lines.append(f"{phase:15s}: {start:7.2f}s - {end:7.2f}s  ({duration:6.2f}s, {packet_count:4d} packets)")
        report_lines.append("")
    
    # Temperature extremes
    if 'temperature' in df.columns:
        report_lines.append("TEMPERATURE STATISTICS")
        report_lines.append("-"*80)
        report_lines.append(f"Minimum Temperature:   {df['temperature'].min():.1f} C")
        report_lines.append(f"Maximum Temperature:   {df['temperature'].max():.1f} C")
        report_lines.append(f"Average Temperature:   {df['temperature'].mean():.1f} C")
        report_lines.append("")
    
    report_lines.append("="*80)
    
    # Output report
    report_text = "\n".join(report_lines)
    
    if output_file:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(report_text)
        print(f"Report saved to: {output_file}")
    else:
        print(report_text)
    
    return report_text

def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("Usage: python analyze_telemetry.py <csv_file> [output_directory]")
        print("\nExample:")
        print("  python analyze_telemetry.py telemetry_data_20260323_143052.csv")
        print("  python analyze_telemetry.py telemetry_data_20260323_143052.csv ./analysis_output")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    print("="*80)
    print("TELEMETRY DATA ANALYSIS")
    print("="*80)
    print()
    
    # Generate report
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        base_name = os.path.splitext(os.path.basename(csv_file))[0]
        report_file = os.path.join(output_dir, f'{base_name}_report.txt')
        generate_flight_report(csv_file, report_file)
    else:
        generate_flight_report(csv_file)
    
    print()
    
    # Generate plots
    plot_telemetry_data(csv_file, output_dir)
    
    print("\nAnalysis complete!")

if __name__ == '__main__':
    main()