#!/usr/bin/env python3
"""
Advanced Telemetry Analysis - Runs all analyses and saves to folder
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def run_all_analyses(csv_file, output_dir='advanced_analysis_folder'):
    """Run all advanced analyses and save report"""
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    results = []
    
    # 1. Rotation Analysis
    results.extend(analyze_rotation_rate(csv_file, output_dir))
    
    # 2. GPS Trajectory
    results.extend(analyze_gps_trajectory(csv_file, output_dir))
    
    # 3. Signal vs Altitude
    results.extend(analyze_signal_vs_altitude(csv_file, output_dir))
    
    # 4. Anomaly Detection
    results.extend(detect_anomalies(csv_file))
    
    # 5. Energy Analysis
    results.extend(calculate_energy(csv_file, output_dir))
    
    # 6. Google Earth Export
    results.extend(export_kml(csv_file, output_dir))
    
    # Save report
    report_file = os.path.join(output_dir, 'analysis_report.txt')
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(results))
    
    return results, report_file

def analyze_rotation_rate(csv_file, output_dir):
    df = pd.read_csv(csv_file)
    df['rotation_magnitude'] = np.sqrt(df['gyro_x']**2 + df['gyro_y']**2 + df['gyro_z']**2)
    
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    plt.plot(df['timestamp'], df['gyro_x'], label='Gyro X', alpha=0.7)
    plt.plot(df['timestamp'], df['gyro_y'], label='Gyro Y', alpha=0.7)
    plt.plot(df['timestamp'], df['gyro_z'], label='Gyro Z', alpha=0.7)
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Angular Velocity (deg/s)')
    plt.title('Gyroscope Data')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    plt.plot(df['timestamp'], df['rotation_magnitude'], color='red', linewidth=2)
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Total Rotation Rate (deg/s)')
    plt.title('Total Rotation Magnitude')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, 'rotation_analysis.png')
    plt.savefig(output_file, dpi=150)
    plt.close()
    
    return [
        f"Rotation analysis saved to {output_file}",
        f"Max rotation rate: {df['rotation_magnitude'].max():.1f} deg/s",
        f"Average rotation rate: {df['rotation_magnitude'].mean():.1f} deg/s"
    ]

def analyze_gps_trajectory(csv_file, output_dir):
    df = pd.read_csv(csv_file)
    gps_df = df[['timestamp', 'latitude', 'longitude', 'altitude']].dropna()
    
    if len(gps_df) == 0:
        return ["No GPS data available"]
    
    fig = plt.figure(figsize=(14, 6))
    
    ax1 = plt.subplot(1, 2, 1)
    scatter = ax1.scatter(gps_df['longitude'], gps_df['latitude'], c=gps_df['altitude'], cmap='viridis', s=50)
    ax1.plot(gps_df['longitude'], gps_df['latitude'], 'k--', alpha=0.3, linewidth=1)
    ax1.plot(gps_df['longitude'].iloc[0], gps_df['latitude'].iloc[0], 'go', markersize=15, label='Start')
    ax1.plot(gps_df['longitude'].iloc[-1], gps_df['latitude'].iloc[-1], 'rs', markersize=15, label='End')
    ax1.set_xlabel('Longitude (deg)')
    ax1.set_ylabel('Latitude (deg)')
    ax1.set_title('GPS Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax1, label='Altitude (m)')
    
    ax2 = plt.subplot(1, 2, 2)
    lat_start = gps_df['latitude'].iloc[0]
    lon_start = gps_df['longitude'].iloc[0]
    lat_diff = (gps_df['latitude'] - lat_start) * 111320
    lon_diff = (gps_df['longitude'] - lon_start) * 111320 * np.cos(np.radians(lat_start))
    horizontal_dist = np.sqrt(lat_diff**2 + lon_diff**2)
    
    ax2.plot(horizontal_dist, gps_df['altitude'], linewidth=2)
    ax2.set_xlabel('Horizontal Distance (m)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Distance')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, 'gps_trajectory.png')
    plt.savefig(output_file, dpi=150)
    plt.close()
    
    return [
        f"GPS trajectory saved to {output_file}",
        f"Total horizontal displacement: {horizontal_dist.iloc[-1]:.1f} m"
    ]

def analyze_signal_vs_altitude(csv_file, output_dir):
    df = pd.read_csv(csv_file)
    
    if 'rssi' not in df.columns or 'altitude' not in df.columns:
        return ["Missing RSSI or altitude data"]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    valid_data = df[['altitude', 'rssi']].dropna()
    ax1.scatter(valid_data['altitude'], valid_data['rssi'], alpha=0.5, s=20)
    ax1.set_xlabel('Altitude (m)')
    ax1.set_ylabel('RSSI (dBm)')
    ax1.set_title('Signal Strength vs Altitude')
    ax1.grid(True, alpha=0.3)
    
    if len(valid_data) > 1:
        z = np.polyfit(valid_data['altitude'], valid_data['rssi'], 1)
        p = np.poly1d(z)
        ax1.plot(valid_data['altitude'], p(valid_data['altitude']), "r--", alpha=0.8, label=f'Trend: {z[0]:.3f} dBm/m')
        ax1.legend()
    
    if 'snr' in df.columns:
        valid_snr = df[['altitude', 'snr']].dropna()
        ax2.scatter(valid_snr['altitude'], valid_snr['snr'], alpha=0.5, s=20, color='orange')
        ax2.set_xlabel('Altitude (m)')
        ax2.set_ylabel('SNR (dB)')
        ax2.set_title('SNR vs Altitude')
        ax2.grid(True, alpha=0.3)
        
        if len(valid_snr) > 1:
            z2 = np.polyfit(valid_snr['altitude'], valid_snr['snr'], 1)
            p2 = np.poly1d(z2)
            ax2.plot(valid_snr['altitude'], p2(valid_snr['altitude']), "r--", alpha=0.8, label=f'Trend: {z2[0]:.3f} dB/m')
            ax2.legend()
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, 'signal_vs_altitude.png')
    plt.savefig(output_file, dpi=150)
    plt.close()
    
    return [f"Signal analysis saved to {output_file}"]

def detect_anomalies(csv_file):
    df = pd.read_csv(csv_file)
    results = [
        "="*80,
        "ANOMALY DETECTION",
        "="*80
    ]
    
    df_sorted = df.sort_values('timestamp')
    time_diffs = df_sorted['timestamp'].diff()
    expected_interval = time_diffs.median()
    large_gaps = time_diffs[time_diffs > 2 * expected_interval]
    
    if len(large_gaps) > 0:
        results.append(f"\nWarning: Detected {len(large_gaps)} data gaps")
    else:
        results.append("\n✅ No significant data gaps detected")
    
    if 'latitude' in df.columns and 'longitude' in df.columns:
        lat_diff = df['latitude'].diff().abs()
        lon_diff = df['longitude'].diff().abs()
        gps_jumps = ((lat_diff > 0.001) | (lon_diff > 0.001)).sum()
        
        if gps_jumps > 0:
            results.append(f"\nWarning: Detected {gps_jumps} large GPS jumps")
        else:
            results.append("\n✅ No large GPS position jumps detected")
    
    if 'temperature' in df.columns:
        temp_diff = df['temperature'].diff().abs()
        temp_spikes = temp_diff[temp_diff > 5]
        
        if len(temp_spikes) > 0:
            results.append(f"\nWarning: Detected {len(temp_spikes)} temperature spikes")
        else:
            results.append("\n✅ No temperature spikes detected")
    
    if 'accel_x' in df.columns:
        accel_mag = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
        saturated = accel_mag > 15
        
        if saturated.sum() > 0:
            results.append(f"\nWarning: Detected {saturated.sum()} near-saturation events")
        else:
            results.append("\n✅ No accelerometer saturation detected")
    
    return results

def calculate_energy(csv_file, output_dir):
    df = pd.read_csv(csv_file)
    MASS = 0.5  # kg
    
    if 'altitude' not in df.columns or 'vertical_speed' not in df.columns:
        return ["Missing altitude or vertical_speed data"]
    
    g = 9.81
    altitude_start = df['altitude'].iloc[0] if not pd.isna(df['altitude'].iloc[0]) else 0
    df['potential_energy'] = MASS * g * (df['altitude'] - altitude_start)
    df['kinetic_energy'] = 0.5 * MASS * df['vertical_speed']**2
    df['total_energy'] = df['potential_energy'] + df['kinetic_energy']
    
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(df['timestamp'], df['altitude'], linewidth=2)
    plt.ylabel('Altitude (m)')
    plt.title(f'Energy Analysis (Mass = {MASS} kg)')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(3, 1, 2)
    plt.plot(df['timestamp'], df['potential_energy'], label='Potential', linewidth=2)
    plt.plot(df['timestamp'], df['kinetic_energy'], label='Kinetic', linewidth=2)
    plt.plot(df['timestamp'], df['total_energy'], label='Total', linewidth=2, linestyle='--')
    plt.ylabel('Energy (J)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(3, 1, 3)
    plt.plot(df['timestamp'], df['vertical_speed'], linewidth=2, color='red')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Vertical Speed (m/s)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, 'energy_analysis.png')
    plt.savefig(output_file, dpi=150)
    plt.close()
    
    return [
        f"Energy analysis saved to {output_file}",
        f"Max potential energy: {df['potential_energy'].max():.2f} J",
        f"Max kinetic energy: {df['kinetic_energy'].max():.2f} J"
    ]

def export_kml(csv_file, output_dir):
    df = pd.read_csv(csv_file)
    gps_df = df[['timestamp', 'latitude', 'longitude', 'altitude']].dropna()
    
    if len(gps_df) == 0:
        return ["No GPS data to export"]
    
    kml = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<kml xmlns="http://www.opengis.net/kml/2.2">',
           '  <Document>',
           '    <name>Flight Trajectory</name>',
           '    <Placemark>',
           '      <LineString>',
           '        <altitudeMode>absolute</altitudeMode>',
           '        <coordinates>']
    
    for _, row in gps_df.iterrows():
        kml.append(f'          {row["longitude"]},{row["latitude"]},{row["altitude"]}')
    
    kml.extend(['        </coordinates>',
                '      </LineString>',
                '    </Placemark>',
                '  </Document>',
                '</kml>'])
    
    output_file = os.path.join(output_dir, 'flight_trajectory.kml')
    with open(output_file, 'w') as f:
        f.write('\n'.join(kml))
    
    return [
        f"KML file saved to {output_file}",
        "Open this file in Google Earth to view 3D trajectory!"
    ]

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python advanced_analysis.py <csv_file> [output_folder]")
        print("\nExample:")
        print("  python advanced_analysis.py test.csv")
        print("  python advanced_analysis.py test.csv my_analysis")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'advanced_analysis_output'
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    print("="*80)
    print("ADVANCED TELEMETRY ANALYSIS")
    print("="*80)
    print(f"Input: {csv_file}")
    print(f"Output: {output_dir}/")
    print()
    
    results, report_file = run_all_analyses(csv_file, output_dir)
    
    for line in results:
        print(line)
    
    print(f"\nReport saved to: {report_file}")
    print("="*80)