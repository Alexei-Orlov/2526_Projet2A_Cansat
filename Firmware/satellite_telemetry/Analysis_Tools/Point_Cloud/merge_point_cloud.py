#!/usr/bin/env python3
"""
Point Cloud Generator from LIDAR + GPS + IMU Data
==================================================

This script merges two CSV files from your satellite:
  1. LIDAR.CSV - High-speed LIDAR distance measurements
  2. TELEM.CSV - Full telemetry (GPS, IMU, etc.)

OUTPUT:
  - 3D point cloud (latitude, longitude, altitude of ground points)
  - Exportable to CloudCompare, MeshLab, or GIS software
  - Visualization plots

USAGE:
  python merge_point_cloud.py LIDAR.CSV TELEM.CSV
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
import sys

def load_data(lidar_file, telem_file):
    """Load both CSV files"""
    print(f"Loading {lidar_file}...")
    lidar_df = pd.read_csv(lidar_file)
    print(f"  Loaded {len(lidar_df)} LIDAR readings")
    
    print(f"Loading {telem_file}...")
    telem_df = pd.read_csv(telem_file)
    print(f"  Loaded {len(telem_df)} telemetry packets")
    
    return lidar_df, telem_df

def interpolate_telemetry(lidar_df, telem_df):
    """
    Interpolate telemetry data to LIDAR timestamps
    
    WHY? LIDAR samples at 50 Hz, telemetry at 2 Hz
    We need GPS/IMU at every LIDAR timestamp
    
    HOW? Linear interpolation between telemetry samples
    """
    print("\nInterpolating telemetry to LIDAR timestamps...")
    
    # Create interpolation functions for each field we need
    fields_to_interpolate = [
        'roll', 'pitch', 'yaw',
        'latitude', 'longitude', 'altitude'
    ]
    
    # Remove NaN values from telemetry
    telem_clean = telem_df.dropna(subset=fields_to_interpolate)
    
    if len(telem_clean) < 2:
        print("ERROR: Not enough valid telemetry data for interpolation")
        return None
    
    # Create interpolation functions
    interpolators = {}
    for field in fields_to_interpolate:
        interpolators[field] = interp1d(
            telem_clean['timestamp_ms'],
            telem_clean[field],
            kind='linear',
            bounds_error=False,
            fill_value='extrapolate'
        )
    
    # Apply interpolation to LIDAR timestamps
    lidar_enhanced = lidar_df.copy()
    
    for field in fields_to_interpolate:
        # If field exists in LIDAR.CSV, it might be outdated, recalculate
        lidar_enhanced[f'{field}_interp'] = interpolators[field](lidar_df['timestamp_ms'])
    
    print(f"  Interpolated {len(fields_to_interpolate)} fields")
    print(f"  Result: {len(lidar_enhanced)} complete data points")
    
    return lidar_enhanced

def calculate_ground_points(lidar_enhanced):
    """
    Calculate 3D coordinates of ground points
    
    PHYSICS:
    ========
    Satellite is at position P = (lat, lon, alt)
    LIDAR measures distance d in direction v
    Ground point G = P + d * v
    
    Direction vector v is calculated from roll/pitch/yaw:
    - Roll: rotation around X (forward)
    - Pitch: rotation around Y (sideways)
    - Yaw: rotation around Z (up)
    
    For LIDAR pointing down (default):
    - 0° roll, 0° pitch, any yaw → points straight down
    - Positive pitch → LIDAR tilts forward
    - Positive roll → LIDAR tilts right
    
    COORDINATE SYSTEM:
    ==================
    We use ENU (East-North-Up):
    - X = East
    - Y = North
    - Z = Up
    """
    print("\nCalculating ground point positions...")
    
    points = []
    
    for idx, row in lidar_enhanced.iterrows():
        # Satellite position (WGS84)
        sat_lat = row['latitude_interp']
        sat_lon = row['longitude_interp']
        sat_alt = row['altitude_interp']
        
        # LIDAR measurement
        distance = row['distance_m']
        
        # Orientation (degrees)
        roll = np.radians(row['roll_interp'])
        pitch = np.radians(row['pitch_interp'])
        yaw = np.radians(row['yaw_interp'])
        
        # === CALCULATE LIDAR DIRECTION VECTOR ===
        # Assuming LIDAR points downward when roll=pitch=0
        # This creates a unit vector in body frame, then rotates to world frame
        
        # Start with downward vector in body frame
        body_vector = np.array([0, 0, -1])  # Points down
        
        # Rotation matrices (ZYX Euler angles - yaw, pitch, roll)
        # Yaw (rotation around Z-axis)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,            0,           1]
        ])
        
        # Pitch (rotation around Y-axis)
        Ry = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [ 0,             1, 0            ],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Roll (rotation around X-axis)
        Rx = np.array([
            [1, 0,            0           ],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]
        ])
        
        # Combined rotation: R = Rz * Ry * Rx
        R = Rz @ Ry @ Rx
        
        # Apply rotation to get world-frame direction
        world_vector = R @ body_vector
        
        # === CONVERT TO GEOGRAPHIC OFFSET ===
        # Small angle approximation (good for local areas)
        # 1 degree latitude ≈ 111,320 meters
        # 1 degree longitude ≈ 111,320 * cos(latitude) meters
        
        # Offset in meters (ENU frame)
        offset_east_m = world_vector[0] * distance   # X component
        offset_north_m = world_vector[1] * distance  # Y component
        offset_up_m = world_vector[2] * distance     # Z component (should be negative)
        
        # Convert meters to degrees
        meters_per_deg_lat = 111320  # Approximately constant
        meters_per_deg_lon = 111320 * np.cos(np.radians(sat_lat))
        
        offset_lat = offset_north_m / meters_per_deg_lat
        offset_lon = offset_east_m / meters_per_deg_lon
        
        # Calculate ground point position
        ground_lat = sat_lat + offset_lat
        ground_lon = sat_lon + offset_lon
        ground_alt = sat_alt + offset_up_m  # Should reduce altitude
        
        points.append({
            'timestamp_ms': row['timestamp_ms'],
            'sat_lat': sat_lat,
            'sat_lon': sat_lon,
            'sat_alt': sat_alt,
            'ground_lat': ground_lat,
            'ground_lon': ground_lon,
            'ground_alt': ground_alt,
            'distance_m': distance,
            'roll': np.degrees(roll),
            'pitch': np.degrees(pitch),
            'yaw': np.degrees(yaw)
        })
    
    points_df = pd.DataFrame(points)
    print(f"  Calculated {len(points_df)} ground points")
    
    return points_df

def export_point_cloud(points_df, output_file='point_cloud.csv'):
    """Export point cloud to CSV"""
    print(f"\nExporting point cloud to {output_file}...")
    
    # Save full data
    points_df.to_csv(output_file, index=False)
    print(f"  Saved {len(points_df)} points")
    
    # Also save simple XYZ format for CloudCompare
    xyz_file = output_file.replace('.csv', '_XYZ.txt')
    with open(xyz_file, 'w') as f:
        for _, row in points_df.iterrows():
            f.write(f"{row['ground_lon']:.8f} {row['ground_lat']:.8f} {row['ground_alt']:.2f}\n")
    print(f"  Saved XYZ format to {xyz_file}")
    print(f"  Import this into CloudCompare or MeshLab!")

def visualize_point_cloud(points_df):
    """Create visualization plots"""
    print("\nCreating visualizations...")
    
    fig = plt.figure(figsize=(18, 12))
    
    # === PLOT 1: 3D Point Cloud ===
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    
    # Plot satellite trajectory
    ax1.plot(points_df['sat_lon'], points_df['sat_lat'], points_df['sat_alt'],
             'r-', linewidth=2, label='Satellite Path', alpha=0.7)
    
    # Plot ground points
    scatter = ax1.scatter(points_df['ground_lon'], points_df['ground_lat'], points_df['ground_alt'],
                         c=points_df['distance_m'], cmap='viridis', s=20,
                         label='Ground Points')
    
    ax1.set_xlabel('Longitude')
    ax1.set_ylabel('Latitude')
    ax1.set_zlabel('Altitude (m)')
    ax1.set_title('3D Point Cloud')
    ax1.legend()
    plt.colorbar(scatter, ax=ax1, label='LIDAR Distance (m)')
    
    # === PLOT 2: Top View (2D Map) ===
    ax2 = fig.add_subplot(2, 3, 2)
    
    ax2.plot(points_df['sat_lon'], points_df['sat_lat'], 'r-', linewidth=2, label='Satellite', alpha=0.7)
    ax2.scatter(points_df['ground_lon'], points_df['ground_lat'],
               c=points_df['ground_alt'], cmap='terrain', s=10, label='Ground')
    ax2.set_xlabel('Longitude')
    ax2.set_ylabel('Latitude')
    ax2.set_title('Top View (Map)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    plt.colorbar(ax2.collections[0], ax=ax2, label='Ground Altitude (m)')
    
    # === PLOT 3: Side View ===
    ax3 = fig.add_subplot(2, 3, 3)
    
    # Calculate distance from start
    start_lat = points_df['sat_lat'].iloc[0]
    start_lon = points_df['sat_lon'].iloc[0]
    
    horizontal_dist = np.sqrt(
        ((points_df['sat_lat'] - start_lat) * 111320)**2 +
        ((points_df['sat_lon'] - start_lon) * 111320 * np.cos(np.radians(start_lat)))**2
    )
    
    ax3.plot(horizontal_dist, points_df['sat_alt'], 'r-', linewidth=2, label='Satellite')
    ax3.scatter(horizontal_dist, points_df['ground_alt'], c='brown', s=10, label='Ground', alpha=0.5)
    ax3.set_xlabel('Horizontal Distance (m)')
    ax3.set_ylabel('Altitude (m)')
    ax3.set_title('Side View Profile')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # === PLOT 4: LIDAR Distance Over Time ===
    ax4 = fig.add_subplot(2, 3, 4)
    
    time_sec = (points_df['timestamp_ms'] - points_df['timestamp_ms'].iloc[0]) / 1000.0
    ax4.plot(time_sec, points_df['distance_m'], linewidth=1)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('LIDAR Distance (m)')
    ax4.set_title('Distance to Ground vs Time')
    ax4.grid(True, alpha=0.3)
    
    # === PLOT 5: Orientation Over Time ===
    ax5 = fig.add_subplot(2, 3, 5)
    
    ax5.plot(time_sec, points_df['roll'], label='Roll', linewidth=1.5)
    ax5.plot(time_sec, points_df['pitch'], label='Pitch', linewidth=1.5)
    ax5.plot(time_sec, points_df['yaw'], label='Yaw', linewidth=1.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Angle (degrees)')
    ax5.set_title('Satellite Orientation')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # === PLOT 6: Statistics ===
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')
    
    stats_text = f"""
    POINT CLOUD STATISTICS
    {'='*40}
    
    Total Points:          {len(points_df)}
    
    LIDAR Distance:
      Min:                 {points_df['distance_m'].min():.2f} m
      Max:                 {points_df['distance_m'].max():.2f} m
      Mean:                {points_df['distance_m'].mean():.2f} m
    
    Satellite Altitude:
      Min:                 {points_df['sat_alt'].min():.2f} m
      Max:                 {points_df['sat_alt'].max():.2f} m
    
    Ground Altitude:
      Min:                 {points_df['ground_alt'].min():.2f} m
      Max:                 {points_df['ground_alt'].max():.2f} m
    
    Coverage Area:
      Lat range:           {(points_df['ground_lat'].max() - points_df['ground_lat'].min())*111320:.1f} m
      Lon range:           {(points_df['ground_lon'].max() - points_df['ground_lon'].min())*111320*np.cos(np.radians(points_df['ground_lat'].mean())):.1f} m
    
    Flight Duration:       {time_sec.iloc[-1]:.1f} seconds
    LIDAR Sample Rate:     {len(points_df) / time_sec.iloc[-1]:.1f} Hz
    """
    
    ax6.text(0.1, 0.9, stats_text, transform=ax6.transAxes,
            verticalalignment='top', fontfamily='monospace', fontsize=10)
    
    plt.tight_layout()
    plt.savefig('point_cloud_visualization.png', dpi=150, bbox_inches='tight')
    print("  Saved visualization to point_cloud_visualization.png")
    plt.show()

def main():
    """Main processing pipeline"""
    if len(sys.argv) < 3:
        print("Usage: python merge_point_cloud.py LIDAR.CSV TELEM.CSV")
        print("\nExample:")
        print("  python merge_point_cloud.py LIDAR.CSV TELEM.CSV")
        sys.exit(1)
    
    lidar_file = sys.argv[1]
    telem_file = sys.argv[2]
    
    print("="*80)
    print("POINT CLOUD GENERATOR")
    print("="*80)
    
    # Step 1: Load data
    lidar_df, telem_df = load_data(lidar_file, telem_file)
    
    # Step 2: Interpolate telemetry to LIDAR timestamps
    lidar_enhanced = interpolate_telemetry(lidar_df, telem_df)
    
    if lidar_enhanced is None:
        print("\nERROR: Interpolation failed")
        sys.exit(1)
    
    # Step 3: Calculate ground points
    points_df = calculate_ground_points(lidar_enhanced)
    
    # Step 4: Export
    export_point_cloud(points_df, 'point_cloud.csv')
    
    # Step 5: Visualize
    visualize_point_cloud(points_df)
    
    print("\n" + "="*80)
    print("PROCESSING COMPLETE!")
    print("="*80)
    print("\nOUTPUT FILES:")
    print("  - point_cloud.csv          (Full data)")
    print("  - point_cloud_XYZ.txt      (For CloudCompare/MeshLab)")
    print("  - point_cloud_visualization.png")
    print("\nNEXT STEPS:")
    print("  1. Open point_cloud_XYZ.txt in CloudCompare")
    print("  2. Apply colormap by altitude")
    print("  3. Generate surface mesh if needed")
    print("  4. Export to GIS formats (.las, .laz)")
    print("="*80)

if __name__ == '__main__':
    main()
