#!/usr/bin/env python3
"""
Generate Synthetic LIDAR + Telemetry Data for Testing
======================================================

Creates realistic test data simulating a satellite drop:
- Launch site: Cergy-Pontoise, France (48.8566°N, 2.3522°E)
- Scenario: Launch at 400m, ascend to 500m, drop with tumbling
- LIDAR sampling: 50 Hz
- Telemetry sampling: 2 Hz

Output:
- LIDAR.CSV (high frequency)
- TELEM.CSV (low frequency)
"""

import numpy as np
import pandas as pd
from datetime import datetime

# Simulation parameters
DURATION_SEC = 60  # 60 second flight
LIDAR_RATE = 50    # 50 Hz (realistic for LightWare SF20)
TELEM_RATE = 2     # 2 Hz (limited by LoRa transmission)

# Starting position (Cergy-Pontoise, France)
START_LAT = 48.8566
START_LON = 2.3522
START_ALT = 400.0  # meters above sea level
GROUND_ALT = 350.0  # Ground level

# Flight phases timing (seconds)
PHASE_CALIB = 0
PHASE_GO_FOR_LAUNCH = 5
PHASE_ASCENSION = 15
PHASE_DROP = 35
PHASE_RECOVERY = 60

def generate_flight_trajectory():
    """
    Generate realistic flight trajectory
    
    Timeline:
    0-5s:   Calibration on ground
    5-15s:  Ascent (balloon/drone lift)
    15-35s: Free fall with tumbling
    35-60s: Recovery/landing
    """
    print("Generating flight trajectory...")
    
    # Time array for telemetry (2 Hz)
    t_telem = np.arange(0, DURATION_SEC, 1.0/TELEM_RATE)
    n_telem = len(t_telem)
    
    # Initialize arrays
    altitude = np.zeros(n_telem)
    vertical_speed = np.zeros(n_telem)
    latitude = np.zeros(n_telem)
    longitude = np.zeros(n_telem)
    roll = np.zeros(n_telem)
    pitch = np.zeros(n_telem)
    yaw = np.zeros(n_telem)
    flags = np.zeros(n_telem, dtype=int)
    
    for i, t in enumerate(t_telem):
        
        # === PHASE 1: CALIBRATION (0-5s) ===
        if t < PHASE_GO_FOR_LAUNCH:
            altitude[i] = START_ALT
            vertical_speed[i] = 0
            latitude[i] = START_LAT
            longitude[i] = START_LON
            roll[i] = 0 + np.random.randn() * 0.5
            pitch[i] = 0 + np.random.randn() * 0.5
            yaw[i] = 45 + np.random.randn() * 1
            flags[i] = 0  # CALIBRATING
            
        # === PHASE 2: GO FOR LAUNCH (5-15s) ===
        elif t < PHASE_ASCENSION:
            # On ground, waiting
            altitude[i] = START_ALT
            vertical_speed[i] = 0
            latitude[i] = START_LAT
            longitude[i] = START_LON + (t - PHASE_GO_FOR_LAUNCH) * 0.00001  # Slight drift
            roll[i] = 0 + np.random.randn() * 1
            pitch[i] = 0 + np.random.randn() * 1
            yaw[i] = 45 + (t - PHASE_GO_FOR_LAUNCH) * 2  # Slow rotation
            flags[i] = 1  # GO_FOR_LAUNCH (0x01)
            
        # === PHASE 3: ASCENSION (15-35s) ===
        elif t < PHASE_DROP:
            # Ascending at ~5 m/s
            phase_time = t - PHASE_ASCENSION
            altitude[i] = START_ALT + phase_time * 5.0
            vertical_speed[i] = 5.0 + np.random.randn() * 0.5
            
            # Horizontal drift during ascent
            latitude[i] = START_LAT + phase_time * 0.00005
            longitude[i] = START_LON + phase_time * 0.00008
            
            # Gentle rocking
            roll[i] = 5 * np.sin(phase_time * 0.5) + np.random.randn() * 2
            pitch[i] = 3 * np.cos(phase_time * 0.3) + np.random.randn() * 2
            yaw[i] = 45 + phase_time * 5 + np.random.randn() * 3
            flags[i] = 3  # GO_FOR_LAUNCH | ASCENSION (0x01 | 0x02 = 0x03)
            
        # === PHASE 4: DROP (35-60s) ===
        elif t < PHASE_RECOVERY:
            # Free fall with tumbling
            phase_time = t - PHASE_DROP
            
            # Physics: v = v0 + g*t, but with air resistance approaching terminal velocity
            g = 9.81
            terminal_v = -15.0  # Terminal velocity (negative = falling)
            v_t = terminal_v * (1 - np.exp(-g * phase_time / abs(terminal_v)))
            
            altitude[i] = 500.0 + v_t * phase_time + 0.5 * (-g) * phase_time**2 * np.exp(-phase_time/3)
            altitude[i] = max(altitude[i], GROUND_ALT + 10)  # Don't go below ground
            
            vertical_speed[i] = v_t + np.random.randn() * 1.0
            
            # Horizontal drift
            latitude[i] = START_LAT + 0.001 + phase_time * 0.00003
            longitude[i] = START_LON + 0.0016 + phase_time * 0.00002
            
            # Intense tumbling during drop
            roll[i] = 180 * np.sin(phase_time * 2.5) + np.random.randn() * 10
            pitch[i] = 90 * np.cos(phase_time * 2.0) + np.random.randn() * 10
            yaw[i] = phase_time * 180 + np.random.randn() * 15
            
            flags[i] = 7  # GO_FOR_LAUNCH | ASCENSION | DROP (0x01 | 0x02 | 0x04 = 0x07)
            
        # === PHASE 5: RECOVERY (after landing) ===
        else:
            altitude[i] = GROUND_ALT + 5  # On ground
            vertical_speed[i] = 0
            latitude[i] = START_LAT + 0.0015
            longitude[i] = START_LON + 0.002
            roll[i] = 0 + np.random.randn() * 2
            pitch[i] = 0 + np.random.randn() * 2
            yaw[i] = 90 + np.random.randn() * 5
            flags[i] = 15  # All flags (0x01 | 0x02 | 0x04 | 0x08 = 0x0F)
    
    return t_telem, altitude, vertical_speed, latitude, longitude, roll, pitch, yaw, flags

def generate_telemetry_csv():
    """Generate TELEM.CSV with full sensor data"""
    print("\nGenerating TELEM.CSV...")
    
    t, altitude, v_speed, lat, lon, roll, pitch, yaw, flags = generate_flight_trajectory()
    
    # Generate other sensor data
    n = len(t)
    
    # Accelerometer (m/s²) - includes gravity and motion
    accel_x = np.random.randn(n) * 0.5
    accel_y = np.random.randn(n) * 0.5
    accel_z = 9.81 + np.random.randn(n) * 0.3  # Gravity + noise
    
    # Add acceleration spikes during drop
    for i, time in enumerate(t):
        if PHASE_DROP <= time < PHASE_RECOVERY:
            # Tumbling creates centripetal acceleration
            accel_x[i] += np.random.randn() * 2
            accel_y[i] += np.random.randn() * 2
            accel_z[i] += np.random.randn() * 3
    
    # Gyroscope (deg/s) - rotation rates
    gyro_x = np.zeros(n)
    gyro_y = np.zeros(n)
    gyro_z = np.zeros(n)
    
    for i, time in enumerate(t):
        if PHASE_DROP <= time < PHASE_RECOVERY:
            # Tumbling - high rotation rates
            gyro_x[i] = 50 * np.sin(time * 2.5) + np.random.randn() * 10
            gyro_y[i] = 40 * np.cos(time * 2.0) + np.random.randn() * 10
            gyro_z[i] = 180 + np.random.randn() * 20
        else:
            # Stable - low rotation
            gyro_x[i] = np.random.randn() * 2
            gyro_y[i] = np.random.randn() * 2
            gyro_z[i] = np.random.randn() * 3
    
    # Temperature (°C) - increases slightly during flight
    temperature = 20 + t * 0.05 + np.random.randn(n) * 0.5
    
    # GPS satellites
    satellites = np.random.randint(6, 11, n)  # 6-10 satellites
    satellites[:10] = np.random.randint(0, 4, 10)  # Poor at start
    
    # Battery voltage (V) - slowly decreases
    battery = 8.0 - t * 0.005 + np.random.randn(n) * 0.02
    
    # Timestamp (ms)
    timestamp_ms = (t * 1000 + 3000).astype(int)  # Start at 3000ms (after calibration)
    
    # Create DataFrame
    telem_df = pd.DataFrame({
        'timestamp_ms': timestamp_ms,
        'accel_x': accel_x,
        'accel_y': accel_y,
        'accel_z': accel_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw,
        'temperature': temperature,
        'altitude': altitude,
        'latitude': lat,
        'longitude': lon,
        'satellites': satellites,
        'flags_raw': flags,
        'battery_voltage': battery
    })
    
    # Save to CSV
    telem_df.to_csv('TELEM.CSV', index=False, float_format='%.6f')
    print(f"  Saved {len(telem_df)} telemetry packets")
    print(f"  Sampling rate: {TELEM_RATE} Hz")
    print(f"  File: TELEM.CSV")
    
    return telem_df

def generate_lidar_csv(telem_df):
    """Generate LIDAR.CSV with high-frequency distance measurements"""
    print("\nGenerating LIDAR.CSV...")
    
    # High frequency time array (50 Hz)
    t_lidar = np.arange(0, DURATION_SEC, 1.0/LIDAR_RATE)
    n_lidar = len(t_lidar)
    
    # Interpolate telemetry to LIDAR timestamps
    from scipy.interpolate import interp1d
    
    t_telem = telem_df['timestamp_ms'].values / 1000.0  # Convert to seconds
    
    # Create interpolators
    interp_alt = interp1d(t_telem, telem_df['altitude'].values, kind='linear', fill_value='extrapolate')
    interp_lat = interp1d(t_telem, telem_df['latitude'].values, kind='linear', fill_value='extrapolate')
    interp_lon = interp1d(t_telem, telem_df['longitude'].values, kind='linear', fill_value='extrapolate')
    interp_roll = interp1d(t_telem, telem_df['roll'].values, kind='linear', fill_value='extrapolate')
    interp_pitch = interp1d(t_telem, telem_df['pitch'].values, kind='linear', fill_value='extrapolate')
    interp_yaw = interp1d(t_telem, telem_df['yaw'].values, kind='linear', fill_value='extrapolate')
    interp_flags = interp1d(t_telem, telem_df['flags_raw'].values, kind='nearest', fill_value='extrapolate')
    
    # Generate interpolated values
    altitude = interp_alt(t_lidar)
    latitude = interp_lat(t_lidar)
    longitude = interp_lon(t_lidar)
    roll = interp_roll(t_lidar)
    pitch = interp_pitch(t_lidar)
    yaw = interp_yaw(t_lidar)
    flags = interp_flags(t_lidar).astype(int)
    
    # Calculate LIDAR distance (altitude - ground)
    # Add realistic variations based on terrain and orientation
    distance_m = np.zeros(n_lidar)
    
    for i, t in enumerate(t_lidar):
        # Base distance (satellite altitude - ground altitude)
        base_distance = altitude[i] - GROUND_ALT
        
        # Add terrain variation (simulated hills/valleys)
        terrain_offset = 5 * np.sin(longitude[i] * 1000) + 3 * np.cos(latitude[i] * 1000)
        
        # Add effect of orientation (if tilted, distance changes)
        # If pitched forward/back or rolled left/right, LIDAR sees farther
        tilt_angle = np.sqrt(roll[i]**2 + pitch[i]**2)
        tilt_factor = 1.0 / np.cos(np.radians(min(tilt_angle, 45)))
        
        # Calculate distance
        distance_m[i] = (base_distance - terrain_offset) * tilt_factor
        
        # Add measurement noise
        distance_m[i] += np.random.randn() * 0.3
        
        # Ensure realistic bounds
        distance_m[i] = max(1.0, min(distance_m[i], 200.0))
    
    # Timestamp (ms)
    timestamp_ms = (t_lidar * 1000 + 3000).astype(int)
    
    # Create DataFrame
    lidar_df = pd.DataFrame({
        'timestamp_ms': timestamp_ms,
        'distance_m': distance_m,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw,
        'latitude': latitude,
        'longitude': longitude,
        'altitude': altitude,
        'flags': flags
    })
    
    # Save to CSV
    lidar_df.to_csv('LIDAR.CSV', index=False, float_format='%.6f')
    print(f"  Saved {len(lidar_df)} LIDAR readings")
    print(f"  Sampling rate: {LIDAR_RATE} Hz")
    print(f"  File: LIDAR.CSV")
    
    return lidar_df

def print_summary(telem_df, lidar_df):
    """Print summary statistics"""
    print("\n" + "="*80)
    print("SYNTHETIC DATA GENERATION COMPLETE")
    print("="*80)
    
    print("\nFLIGHT SCENARIO:")
    print(f"  Location: Cergy-Pontoise, France")
    print(f"  Start: {START_LAT:.4f}°N, {START_LON:.4f}°E")
    print(f"  Duration: {DURATION_SEC} seconds")
    print()
    print(f"  0-5s:    Calibration on ground ({START_ALT}m)")
    print(f"  5-15s:   Go for launch (waiting)")
    print(f"  15-35s:  Ascension to 500m")
    print(f"  35-60s:  Drop with tumbling")
    print(f"  60s+:    Recovery/landing")
    
    print("\nTELEMETRY (TELEM.CSV):")
    print(f"  Packets: {len(telem_df)}")
    print(f"  Rate: {TELEM_RATE} Hz")
    print(f"  Fields: timestamp_ms, accel_xyz, gyro_xyz, roll, pitch, yaw,")
    print(f"          temperature, altitude, lat, lon, satellites, flags, battery")
    print(f"  Altitude range: {telem_df['altitude'].min():.1f} - {telem_df['altitude'].max():.1f} m")
    print(f"  Max rotation: {telem_df['gyro_z'].max():.1f} deg/s")
    
    print("\nLIDAR (LIDAR.CSV):")
    print(f"  Readings: {len(lidar_df)}")
    print(f"  Rate: {LIDAR_RATE} Hz")
    print(f"  Fields: timestamp_ms, distance_m, roll, pitch, yaw, lat, lon, alt, flags")
    print(f"  Distance range: {lidar_df['distance_m'].min():.1f} - {lidar_df['distance_m'].max():.1f} m")
    
    print("\nFILE SIZES:")
    import os
    telem_size = os.path.getsize('TELEM.CSV') / 1024
    lidar_size = os.path.getsize('LIDAR.CSV') / 1024
    print(f"  TELEM.CSV: {telem_size:.1f} KB")
    print(f"  LIDAR.CSV: {lidar_size:.1f} KB")
    print(f"  Total: {telem_size + lidar_size:.1f} KB")
    
    print("\nNEXT STEPS:")
    print("  1. Run: python merge_point_cloud.py LIDAR.CSV TELEM.CSV")
    print("  2. View: point_cloud_visualization.png")
    print("  3. Import: point_cloud_XYZ.txt into CloudCompare")
    
    print("\n" + "="*80)

def main():
    """Main generation pipeline"""
    print("="*80)
    print("SYNTHETIC DATA GENERATOR")
    print("="*80)
    print("\nGenerating realistic flight data for testing...")
    print(f"Flight duration: {DURATION_SEC} seconds")
    print(f"LIDAR rate: {LIDAR_RATE} Hz")
    print(f"Telemetry rate: {TELEM_RATE} Hz")
    
    # Generate files
    telem_df = generate_telemetry_csv()
    lidar_df = generate_lidar_csv(telem_df)
    
    # Print summary
    print_summary(telem_df, lidar_df)

if __name__ == '__main__':
    main()
