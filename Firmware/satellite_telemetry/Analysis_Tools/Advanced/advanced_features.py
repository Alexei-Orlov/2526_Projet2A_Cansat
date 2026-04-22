#!/usr/bin/env python3
"""
Advanced Flight Analysis Features
==================================

This script provides 5 advanced analysis tools:
1. Animated Flight Replay (video of trajectory)
2. Drag Coefficient Calculation (aerodynamics)
3. Parachute Deployment Analysis (detect deployment)
4. 3D Orientation Visualization (roll/pitch/yaw)
5. Power Consumption Analysis (battery drain)

Each feature teaches you the physics and math behind it!
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy import signal
import os
import sys

# Flight phase definitions
FLAG_GO_FOR_LAUNCH = 0x01
FLAG_ASCENSION = 0x02
FLAG_DROP = 0x04
FLAG_RECOVERY = 0x08

def decode_flight_phase(flags_raw):
    """Decode flags to flight phase"""
    if pd.isna(flags_raw):
        return 'UNKNOWN'
    try:
        flags_raw = int(flags_raw)
    except:
        return 'UNKNOWN'
    
    if flags_raw == 0:
        return 'CALIBRATING'
    elif flags_raw == FLAG_GO_FOR_LAUNCH:
        return 'GO_FOR_LAUNCH'
    elif flags_raw == (FLAG_GO_FOR_LAUNCH | FLAG_ASCENSION):
        return 'ASCENSION'
    elif flags_raw == (FLAG_GO_FOR_LAUNCH | FLAG_ASCENSION | FLAG_DROP):
        return 'DROP'
    elif flags_raw == (FLAG_GO_FOR_LAUNCH | FLAG_ASCENSION | FLAG_DROP | FLAG_RECOVERY):
        return 'RECOVERY'
    else:
        return 'UNKNOWN'

def create_animated_replay(df, output_dir='advanced_features'):
    """
    FEATURE 1: Animated Flight Replay
    ==================================
    
    WHAT IT DOES:
      Creates a video showing the flight trajectory evolving over time
      
    WHY IT'S USEFUL:
      - Visualize the entire flight in motion
      - See how GPS path develops
      - Share flight video with others
      - Spot anomalies by watching trajectory unfold
      
    HOW IT WORKS:
      Uses matplotlib animation to update plots frame-by-frame
      Each frame shows position up to that point in time
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("FEATURE 1: ANIMATED FLIGHT REPLAY")
    print("="*80)
    print("\nCREATING ANIMATED FLIGHT VIDEO...")
    print("\nThis creates a video showing your flight path developing over time.")
    print("Great for presentations or sharing your flight!")
    
    # Get GPS data
    gps_data = df[['timestamp', 'latitude', 'longitude', 'altitude', 'flags_raw']].dropna()
    
    if len(gps_data) < 2:
        print("\nNot enough GPS data for animation (need at least 2 points)")
        return
    
    # Decode phases
    gps_data['phase'] = gps_data['flags_raw'].apply(decode_flight_phase)
    
    # Calculate bounds for plotting
    lat_range = gps_data['latitude'].max() - gps_data['latitude'].min()
    lon_range = gps_data['longitude'].max() - gps_data['longitude'].min()
    
    # Add padding
    lat_center = gps_data['latitude'].mean()
    lon_center = gps_data['longitude'].mean()
    max_range = max(lat_range, lon_range) * 1.2
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    # Color map for phases
    phase_colors = {
        'CALIBRATING': 'gray',
        'GO_FOR_LAUNCH': 'blue',
        'ASCENSION': 'green',
        'DROP': 'red',
        'RECOVERY': 'orange',
        'UNKNOWN': 'black'
    }
    
    # Initialize plots
    line1, = ax1.plot([], [], 'b-', linewidth=2, alpha=0.6)
    scatter1 = ax1.scatter([], [], c=[], s=100, cmap='viridis', edgecolor='black', linewidth=1)
    current_pos1, = ax1.plot([], [], 'ro', markersize=15, label='Current Position')
    
    ax1.set_xlim(lon_center - max_range/2, lon_center + max_range/2)
    ax1.set_ylim(lat_center - max_range/2, lat_center + max_range/2)
    ax1.set_xlabel('Longitude (deg)', fontsize=12)
    ax1.set_ylabel('Latitude (deg)', fontsize=12)
    ax1.set_title('GPS Trajectory (Top View)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Start and end markers
    ax1.plot(gps_data['longitude'].iloc[0], gps_data['latitude'].iloc[0], 
             'go', markersize=20, label='Start', zorder=10)
    ax1.plot(gps_data['longitude'].iloc[-1], gps_data['latitude'].iloc[-1], 
             'rs', markersize=20, label='End', zorder=10)
    
    # Altitude plot
    line2, = ax2.plot([], [], 'b-', linewidth=2)
    current_pos2, = ax2.plot([], [], 'ro', markersize=10)
    
    ax2.set_xlim(gps_data['timestamp'].min(), gps_data['timestamp'].max())
    ax2.set_ylim(gps_data['altitude'].min() - 10, gps_data['altitude'].max() + 10)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Altitude (m)', fontsize=12)
    ax2.set_title('Altitude vs Time', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Info text
    info_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes, 
                        verticalalignment='top', fontsize=11,
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def init():
        """Initialize animation"""
        line1.set_data([], [])
        scatter1.set_offsets(np.empty((0, 2)))
        current_pos1.set_data([], [])
        line2.set_data([], [])
        current_pos2.set_data([], [])
        info_text.set_text('')
        return line1, scatter1, current_pos1, line2, current_pos2, info_text
    
    def animate(frame):
        """Update animation frame"""
        # Get data up to current frame
        current_data = gps_data.iloc[:frame+1]
        
        # Update trajectory line
        line1.set_data(current_data['longitude'], current_data['latitude'])
        
        # Update scatter with altitude coloring
        scatter1.set_offsets(np.c_[current_data['longitude'], current_data['latitude']])
        scatter1.set_array(current_data['altitude'].values)
        
        # Update current position marker
        current_lon = current_data['longitude'].iloc[-1]
        current_lat = current_data['latitude'].iloc[-1]
        current_pos1.set_data([current_lon], [current_lat])
        
        # Update altitude plot
        line2.set_data(current_data['timestamp'], current_data['altitude'])
        current_pos2.set_data([current_data['timestamp'].iloc[-1]], 
                              [current_data['altitude'].iloc[-1]])
        
        # Update info text
        current_time = current_data['timestamp'].iloc[-1]
        current_alt = current_data['altitude'].iloc[-1]
        current_phase = current_data['phase'].iloc[-1]
        
        info_text.set_text(
            f"Time: {current_time:.1f} s\n"
            f"Altitude: {current_alt:.1f} m\n"
            f"Phase: {current_phase}\n"
            f"Point {frame+1}/{len(gps_data)}"
        )
        
        return line1, scatter1, current_pos1, line2, current_pos2, info_text
    
    # Create animation
    print(f"\nGenerating {len(gps_data)} frames...")
    anim = FuncAnimation(fig, animate, init_func=init, 
                        frames=len(gps_data), interval=200, blit=True)
    
    # Save as GIF
    output_file = os.path.join(output_dir, 'flight_replay.gif')
    writer = PillowWriter(fps=5)
    
    print(f"Saving animation (this may take a minute)...")
    anim.save(output_file, writer=writer)
    plt.close()
    
    print(f"\n✓ Animation saved to: {output_file}")
    print(f"  Duration: {len(gps_data)/5:.1f} seconds at 5 fps")
    print(f"  Frames: {len(gps_data)}")
    print("\nYou can open this GIF in any browser or image viewer!")

def calculate_drag_coefficient(df, output_dir='advanced_features'):
    """
    FEATURE 2: Drag Coefficient Calculation
    ========================================
    
    WHAT IT DOES:
      Calculates the aerodynamic drag coefficient (Cd) from free-fall data
      
    WHY IT'S USEFUL:
      - Understand your satellite's aerodynamics
      - Predict terminal velocity
      - Design better shapes for future flights
      - Validate simulations
      
    THE PHYSICS:
      During free fall, two forces act on the satellite:
      1. Gravity: Fg = m * g (pulls down)
      2. Drag: Fd = 0.5 * rho * v^2 * Cd * A (slows down)
      
      At terminal velocity: Fg = Fd
      Solving for Cd: Cd = (2 * m * g) / (rho * v_terminal^2 * A)
      
    WHERE:
      m = mass (kg)
      g = 9.81 m/s^2 (gravity)
      rho = 1.225 kg/m^3 (air density at sea level)
      v_terminal = steady falling speed (m/s)
      A = cross-sectional area (m^2)
      Cd = drag coefficient (dimensionless)
      
    TYPICAL Cd VALUES:
      - Sphere: 0.47
      - Cube: 1.05
      - Flat plate: 1.28
      - Streamlined: 0.04
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("FEATURE 2: DRAG COEFFICIENT CALCULATION")
    print("="*80)
    
    # Get DROP phase data
    df['phase'] = df['flags_raw'].apply(decode_flight_phase)
    drop_data = df[df['phase'] == 'DROP'].copy()
    
    if len(drop_data) < 5:
        print("\nNot enough DROP phase data for drag analysis")
        print("Need at least 5 data points during free fall")
        return
    
    print("\nANALYZING FREE FALL DYNAMICS...")
    print("\nTHE PHYSICS:")
    print("  During free fall: Weight (down) vs Drag (up)")
    print("  Fg = m*g")
    print("  Fd = 0.5 * rho * v^2 * Cd * A")
    print("  At terminal velocity: Fg = Fd")
    
    # USER INPUTS (you need to provide these!)
    print("\n" + "-"*80)
    print("REQUIRED PARAMETERS (UPDATE THESE!):")
    print("-"*80)
    
    MASS = 0.5  # kg - YOUR SATELLITE MASS
    AREA = 0.01  # m^2 - Cross-sectional area (10cm x 10cm = 0.01 m^2)
    
    print(f"  Mass: {MASS} kg")
    print(f"  Cross-sectional area: {AREA} m^2 ({np.sqrt(AREA)*100:.1f} cm x {np.sqrt(AREA)*100:.1f} cm)")
    print("\n  ⚠️  UPDATE THESE VALUES IN THE SCRIPT FOR ACCURATE RESULTS!")
    
    # Constants
    g = 9.81  # m/s^2
    rho = 1.225  # kg/m^3 (air density at sea level, 15°C)
    
    # Smooth vertical speed
    if len(drop_data) > 5:
        drop_data['vspeed_smooth'] = signal.savgol_filter(
            drop_data['vertical_speed'], 
            window_length=min(5, len(drop_data)|1), 
            polyorder=2
        )
    else:
        drop_data['vspeed_smooth'] = drop_data['vertical_speed']
    
    # Find terminal velocity (most negative speed during drop)
    terminal_velocity = drop_data['vspeed_smooth'].min()  # Negative = falling
    terminal_velocity_abs = abs(terminal_velocity)
    
    print("\n" + "-"*80)
    print("RESULTS:")
    print("-"*80)
    print(f"\n  Terminal velocity: {terminal_velocity_abs:.2f} m/s")
    print(f"  That's {terminal_velocity_abs * 3.6:.1f} km/h")
    
    # Calculate drag coefficient
    # Cd = (2 * m * g) / (rho * v^2 * A)
    if terminal_velocity_abs > 0.1:  # Avoid division by zero
        Cd = (2 * MASS * g) / (rho * terminal_velocity_abs**2 * AREA)
        
        print(f"\n  CALCULATED DRAG COEFFICIENT (Cd): {Cd:.3f}")
        print("\n  REFERENCE VALUES:")
        print("    - Sphere: 0.47")
        print("    - Cube: 1.05")
        print("    - Flat plate: 1.28")
        print("    - Streamlined object: 0.04")
        
        # Interpret result
        if Cd < 0.5:
            shape = "streamlined (good aerodynamics!)"
        elif Cd < 1.0:
            shape = "moderately aerodynamic"
        elif Cd < 1.5:
            shape = "blunt/boxy"
        else:
            shape = "very high drag (tumbling or unstable)"
        
        print(f"\n  Your satellite appears: {shape}")
        
        # Calculate drag force at terminal velocity
        drag_force = 0.5 * rho * terminal_velocity_abs**2 * Cd * AREA
        weight = MASS * g
        
        print(f"\n  Drag force at terminal velocity: {drag_force:.3f} N")
        print(f"  Weight: {weight:.3f} N")
        print(f"  Force balance check: {abs(drag_force - weight):.3f} N difference")
        print("    (Should be ~0 at terminal velocity)")
    else:
        print("\n  Terminal velocity too low for reliable Cd calculation")
        Cd = None
    
    # Plot analysis
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Altitude vs time
    ax1.plot(drop_data['timestamp'], drop_data['altitude'], 'b-', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Altitude During Free Fall')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Vertical speed vs time
    ax2.plot(drop_data['timestamp'], drop_data['vertical_speed'], 
             'o-', alpha=0.5, label='Raw')
    ax2.plot(drop_data['timestamp'], drop_data['vspeed_smooth'], 
             'r-', linewidth=2, label='Smoothed')
    ax2.axhline(terminal_velocity, color='green', linestyle='--', linewidth=2,
                label=f'Terminal velocity: {terminal_velocity_abs:.2f} m/s')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Vertical Speed (m/s)')
    ax2.set_title('Vertical Speed (negative = falling)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Speed vs altitude
    ax3.plot(drop_data['altitude'], abs(drop_data['vspeed_smooth']), 'bo-', linewidth=2)
    ax3.axhline(terminal_velocity_abs, color='green', linestyle='--', linewidth=2,
                label='Terminal velocity')
    ax3.set_xlabel('Altitude (m)')
    ax3.set_ylabel('Speed (m/s)')
    ax3.set_title('Speed vs Altitude (approaching terminal velocity)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Force diagram
    ax4.text(0.5, 0.9, 'FORCE BALANCE AT TERMINAL VELOCITY', 
             ha='center', fontsize=14, fontweight='bold')
    
    if Cd is not None:
        ax4.text(0.5, 0.75, f'Weight (down): {weight:.3f} N', ha='center', fontsize=12)
        ax4.arrow(0.5, 0.65, 0, -0.15, head_width=0.05, head_length=0.03, fc='red', ec='red')
        
        ax4.text(0.5, 0.45, f'Drag (up): {drag_force:.3f} N', ha='center', fontsize=12)
        ax4.arrow(0.5, 0.35, 0, 0.15, head_width=0.05, head_length=0.03, fc='blue', ec='blue')
        
        ax4.text(0.5, 0.2, f'Cd = {Cd:.3f}', ha='center', fontsize=16, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
        
        ax4.text(0.5, 0.05, f'Shape estimate: {shape}', ha='center', fontsize=11, style='italic')
    
    ax4.set_xlim(0, 1)
    ax4.set_ylim(0, 1)
    ax4.axis('off')
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, 'drag_coefficient_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\n✓ Analysis plot saved to: {output_file}")

def analyze_parachute_deployment(df, output_dir='advanced_features'):
    """
    FEATURE 3: Parachute Deployment Analysis
    =========================================
    
    WHAT IT DOES:
      Detects parachute deployment by looking for sudden deceleration
      
    WHY IT'S USEFUL:
      - Know exact deployment time
      - Measure deployment effectiveness
      - Calculate parachute drag
      - Verify deployment worked properly
      
    HOW IT WORKS:
      Parachute deployment causes sudden upward jerk (deceleration)
      We look for the largest positive change in vertical speed
      (speed becomes less negative = slowing down)
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("FEATURE 3: PARACHUTE DEPLOYMENT ANALYSIS")
    print("="*80)
    print("\nDETECTING PARACHUTE DEPLOYMENT...")
    
    # Calculate acceleration (change in velocity)
    df['accel_vertical'] = df['vertical_speed'].diff() / df['timestamp'].diff()
    
    # Smooth it
    if len(df) > 5:
        df['accel_smooth'] = signal.savgol_filter(
            df['accel_vertical'].fillna(0), 
            window_length=min(5, len(df)|1), 
            polyorder=2
        )
    else:
        df['accel_smooth'] = df['accel_vertical']
    
    # Find largest upward acceleration (parachute opening)
    deployment_idx = df['accel_smooth'].idxmax()
    
    if pd.notna(deployment_idx):
        deployment_time = df.loc[deployment_idx, 'timestamp']
        deployment_alt = df.loc[deployment_idx, 'altitude']
        max_decel = df.loc[deployment_idx, 'accel_smooth']
        
        # Get speeds before and after
        window = 2
        idx_pos = df.index.get_loc(deployment_idx)
        
        speed_before = df['vertical_speed'].iloc[max(0, idx_pos-window):idx_pos].mean()
        speed_after = df['vertical_speed'].iloc[idx_pos:min(len(df), idx_pos+window+1)].mean()
        
        speed_reduction = abs(speed_before) - abs(speed_after)
        
        print("\n" + "-"*80)
        print("DEPLOYMENT DETECTED!")
        print("-"*80)
        print(f"\n  Time: {deployment_time:.2f} seconds")
        print(f"  Altitude: {deployment_alt:.1f} meters")
        print(f"  Max deceleration: {max_decel:.2f} m/s^2 ({max_decel/9.81:.2f} g)")
        print(f"\n  Speed before: {abs(speed_before):.2f} m/s (falling)")
        print(f"  Speed after: {abs(speed_after):.2f} m/s (falling)")
        print(f"  Speed reduction: {speed_reduction:.2f} m/s")
        
        if speed_reduction > 0:
            effectiveness = (speed_reduction / abs(speed_before)) * 100
            print(f"  Effectiveness: {effectiveness:.1f}% speed reduction")
        
        # Plot
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
        
        # Altitude
        ax1.plot(df['timestamp'], df['altitude'], 'b-', linewidth=2)
        ax1.axvline(deployment_time, color='red', linestyle='--', linewidth=2,
                   label=f'Deployment at {deployment_time:.1f}s')
        ax1.scatter([deployment_time], [deployment_alt], color='red', s=200, zorder=10,
                   marker='*', edgecolor='black', linewidth=2)
        ax1.set_ylabel('Altitude (m)')
        ax1.set_title('Parachute Deployment Detection')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Vertical speed
        ax2.plot(df['timestamp'], df['vertical_speed'], 'g-', linewidth=2)
        ax2.axvline(deployment_time, color='red', linestyle='--', linewidth=2)
        ax2.axhline(speed_before, color='orange', linestyle=':', linewidth=2,
                   label=f'Before: {abs(speed_before):.1f} m/s')
        ax2.axhline(speed_after, color='blue', linestyle=':', linewidth=2,
                   label=f'After: {abs(speed_after):.1f} m/s')
        ax2.set_ylabel('Vertical Speed (m/s)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Acceleration (shows the jerk!)
        ax3.plot(df['timestamp'], df['accel_smooth'], 'r-', linewidth=2)
        ax3.axvline(deployment_time, color='red', linestyle='--', linewidth=2)
        ax3.scatter([deployment_time], [max_decel], color='red', s=200, zorder=10,
                   marker='*', edgecolor='black', linewidth=2,
                   label=f'Peak deceleration: {max_decel:.1f} m/s^2')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Vertical Acceleration (m/s^2)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_file = os.path.join(output_dir, 'parachute_deployment.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"\n✓ Analysis plot saved to: {output_file}")
    else:
        print("\nNo clear parachute deployment detected in data")

def visualize_3d_orientation(df, output_dir='advanced_features'):
    """
    FEATURE 4: 3D Orientation Visualization
    ========================================
    
    WHAT IT DOES:
      Creates 3D visualization of satellite orientation over time
      
    WHY IT'S USEFUL:
      - See how satellite tumbles
      - Visualize rotation in 3D space
      - Understand stability
      - Debug orientation sensors
      
    HOW IT WORKS:
      Uses roll/pitch/yaw angles to draw a 3D box showing orientation
      Creates multiple frames showing how orientation changes
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("FEATURE 4: 3D ORIENTATION VISUALIZATION")
    print("="*80)
    print("\nCREATING 3D ORIENTATION PLOTS...")
    
    # Get orientation data
    orient_data = df[['timestamp', 'roll', 'pitch', 'yaw']].dropna()
    
    if len(orient_data) < 1:
        print("\nNo orientation data available")
        return
    
    print(f"\nVisualizingorientation at {len(orient_data)} time points")
    
    def rotation_matrix(roll, pitch, yaw):
        """Create rotation matrix from Euler angles (degrees)"""
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)
        
        # Roll (X-axis rotation)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])
        
        # Pitch (Y-axis rotation)
        Ry = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        # Yaw (Z-axis rotation)
        Rz = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation: Rz * Ry * Rx
        return Rz @ Ry @ Rx
    
    def draw_satellite_box(ax, roll, pitch, yaw):
        """Draw a 3D box representing satellite orientation"""
        # Define box vertices (centered at origin)
        vertices = np.array([
            [-1, -1, -0.5],
            [1, -1, -0.5],
            [1, 1, -0.5],
            [-1, 1, -0.5],
            [-1, -1, 0.5],
            [1, -1, 0.5],
            [1, 1, 0.5],
            [-1, 1, 0.5]
        ])
        
        # Apply rotation
        R = rotation_matrix(roll, pitch, yaw)
        rotated_vertices = vertices @ R.T
        
        # Define faces
        faces = [
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[5], rotated_vertices[4]],
            [rotated_vertices[2], rotated_vertices[3], rotated_vertices[7], rotated_vertices[6]],
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[2], rotated_vertices[3]],
            [rotated_vertices[4], rotated_vertices[5], rotated_vertices[6], rotated_vertices[7]],
            [rotated_vertices[0], rotated_vertices[3], rotated_vertices[7], rotated_vertices[4]],
            [rotated_vertices[1], rotated_vertices[2], rotated_vertices[6], rotated_vertices[5]]
        ]
        
        # Create collection
        face_collection = Poly3DCollection(faces, alpha=0.7, facecolor='cyan', 
                                          edgecolor='black', linewidth=2)
        ax.add_collection3d(face_collection)
        
        # Draw axes
        axis_length = 1.5
        ax.plot([0, axis_length], [0, 0], [0, 0], 'r-', linewidth=3, label='X (Roll)')
        ax.plot([0, 0], [0, axis_length], [0, 0], 'g-', linewidth=3, label='Y (Pitch)')
        ax.plot([0, 0], [0, 0], [0, axis_length], 'b-', linewidth=3, label='Z (Yaw)')
    
    # Select time points to visualize (start, 25%, 50%, 75%, end)
    n_points = len(orient_data)
    indices = [0, n_points//4, n_points//2, 3*n_points//4, n_points-1]
    
    # Create subplots
    fig = plt.figure(figsize=(18, 12))
    
    for i, idx in enumerate(indices):
        row = orient_data.iloc[idx]
        
        ax = fig.add_subplot(2, 3, i+1, projection='3d')
        
        draw_satellite_box(ax, row['roll'], row['pitch'], row['yaw'])
        
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"t={row['timestamp']:.1f}s\n"
                    f"Roll={row['roll']:.1f}° Pitch={row['pitch']:.1f}° Yaw={row['yaw']:.1f}°",
                    fontsize=10)
        ax.legend(fontsize=8)
    
    # Add angle plots
    ax_angles = fig.add_subplot(2, 3, 6)
    ax_angles.plot(orient_data['timestamp'], orient_data['roll'], 'r-', label='Roll', linewidth=2)
    ax_angles.plot(orient_data['timestamp'], orient_data['pitch'], 'g-', label='Pitch', linewidth=2)
    ax_angles.plot(orient_data['timestamp'], orient_data['yaw'], 'b-', label='Yaw', linewidth=2)
    ax_angles.set_xlabel('Time (s)')
    ax_angles.set_ylabel('Angle (degrees)')
    ax_angles.set_title('Orientation Angles Over Time')
    ax_angles.legend()
    ax_angles.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, '3d_orientation.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\n✓ 3D orientation visualization saved to: {output_file}")
    print("\nThe plot shows satellite orientation at 5 different times during flight")
    print("  - Red axis: X (Roll)")
    print("  - Green axis: Y (Pitch)")
    print("  - Blue axis: Z (Yaw)")

def analyze_power_consumption(df, output_dir='advanced_features'):
    """
    FEATURE 5: Power Consumption Analysis
    ======================================
    
    WHAT IT DOES:
      Analyzes battery voltage drop to estimate power consumption
      
    WHY IT'S USEFUL:
      - Estimate battery life
      - Calculate average current draw
      - Find power-hungry phases
      - Plan for longer missions
      
    THE PHYSICS:
      Battery voltage drops as energy is consumed
      Power = Voltage × Current
      Energy = Power × Time
      
      From voltage drop, we can estimate total energy used
      Then calculate average current draw
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("FEATURE 5: POWER CONSUMPTION ANALYSIS")
    print("="*80)
    print("\nANALYZING BATTERY PERFORMANCE...")
    
    # Get battery data
    battery_data = df[['timestamp', 'battery_voltage', 'flags_raw']].dropna()
    
    if len(battery_data) < 2:
        print("\nNot enough battery data for analysis")
        return
    
    battery_data['phase'] = battery_data['flags_raw'].apply(decode_flight_phase)
    
    # Calculate stats
    voltage_start = battery_data['battery_voltage'].iloc[0]
    voltage_end = battery_data['battery_voltage'].iloc[-1]
    voltage_drop = voltage_start - voltage_end
    duration = battery_data['timestamp'].iloc[-1] - battery_data['timestamp'].iloc[0]
    
    print("\n" + "-"*80)
    print("BATTERY STATISTICS:")
    print("-"*80)
    print(f"\n  Starting voltage: {voltage_start:.2f} V")
    print(f"  Ending voltage: {voltage_end:.2f} V")
    print(f"  Voltage drop: {voltage_drop:.2f} V")
    print(f"  Flight duration: {duration:.1f} seconds")
    print(f"  Voltage drop rate: {voltage_drop/duration*3600:.3f} V/hour")
    
    # Estimate based on battery type (user should update!)
    print("\n" + "-"*80)
    print("POWER ESTIMATION (UPDATE BATTERY SPECS!):")
    print("-"*80)
    
    BATTERY_CAPACITY = 2000  # mAh - UPDATE THIS!
    NOMINAL_VOLTAGE = 7.4  # V (2S LiPo) - UPDATE THIS!
    
    print(f"\n  Assumed battery: {BATTERY_CAPACITY} mAh at {NOMINAL_VOLTAGE}V")
    print("  ⚠️  UPDATE THESE VALUES IN SCRIPT FOR ACCURATE RESULTS!")
    
    # Calculate energy used (very rough estimate)
    # Assuming linear voltage drop (not accurate for LiPo, but approximation)
    avg_voltage = (voltage_start + voltage_end) / 2
    
    # Estimate current draw
    # This is a simplified model - real batteries are more complex!
    voltage_percent_drop = voltage_drop / NOMINAL_VOLTAGE
    energy_percent_used = voltage_percent_drop * 100
    
    capacity_used_mAh = BATTERY_CAPACITY * voltage_percent_drop
    avg_current_mA = capacity_used_mAh / (duration / 3600)  # mAh / hours = mA
    
    print(f"\n  Estimated energy used: {energy_percent_used:.1f}% of battery")
    print(f"  Capacity used: {capacity_used_mAh:.0f} mAh")
    print(f"  Average current draw: {avg_current_mA:.0f} mA")
    
    # Estimate remaining flight time at this rate
    remaining_capacity = BATTERY_CAPACITY - capacity_used_mAh
    remaining_time_hours = remaining_capacity / avg_current_mA
    remaining_time_minutes = remaining_time_hours * 60
    
    print(f"\n  Estimated remaining capacity: {remaining_capacity:.0f} mAh")
    print(f"  Estimated remaining flight time: {remaining_time_minutes:.1f} minutes")
    print("    (at current power consumption rate)")
    
    # Power by phase
    print("\n" + "-"*80)
    print("POWER BY FLIGHT PHASE:")
    print("-"*80)
    
    for phase in battery_data['phase'].unique():
        phase_data = battery_data[battery_data['phase'] == phase]
        if len(phase_data) > 1:
            phase_v_start = phase_data['battery_voltage'].iloc[0]
            phase_v_end = phase_data['battery_voltage'].iloc[-1]
            phase_drop = phase_v_start - phase_v_end
            phase_duration = phase_data['timestamp'].iloc[-1] - phase_data['timestamp'].iloc[0]
            
            if phase_duration > 0:
                phase_drop_rate = phase_drop / phase_duration * 3600
                print(f"\n  {phase}:")
                print(f"    Duration: {phase_duration:.1f} s")
                print(f"    Voltage drop: {phase_drop:.3f} V")
                print(f"    Drop rate: {phase_drop_rate:.3f} V/hour")
    
    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # Voltage over time
    ax1.plot(battery_data['timestamp'], battery_data['battery_voltage'], 
             'b-', linewidth=2, marker='o', markersize=4)
    ax1.axhline(voltage_start, color='green', linestyle='--', alpha=0.5,
               label=f'Start: {voltage_start:.2f}V')
    ax1.axhline(voltage_end, color='red', linestyle='--', alpha=0.5,
               label=f'End: {voltage_end:.2f}V')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Battery Voltage (V)')
    ax1.set_title('Battery Voltage During Flight')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Estimated remaining capacity
    time_points = battery_data['timestamp'].values
    voltage_points = battery_data['battery_voltage'].values
    
    # Calculate remaining capacity at each point
    remaining_pct = []
    for v in voltage_points:
        # Rough linear estimate
        pct = ((v - (NOMINAL_VOLTAGE - NOMINAL_VOLTAGE*0.2)) / (NOMINAL_VOLTAGE*0.8)) * 100
        remaining_pct.append(max(0, min(100, pct)))
    
    ax2.plot(time_points, remaining_pct, 'g-', linewidth=2, marker='o', markersize=4)
    ax2.fill_between(time_points, 0, remaining_pct, alpha=0.3, color='green')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Estimated Battery Remaining (%)')
    ax2.set_title('Battery State of Charge (Estimated)')
    ax2.set_ylim([0, 105])
    ax2.grid(True, alpha=0.3)
    
    # Add text box with summary
    summary_text = (
        f"Total Voltage Drop: {voltage_drop:.2f}V\n"
        f"Avg Current: ~{avg_current_mA:.0f} mA\n"
        f"Energy Used: ~{energy_percent_used:.1f}%"
    )
    ax2.text(0.02, 0.98, summary_text, transform=ax2.transAxes,
            verticalalignment='top', fontsize=11,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    output_file = os.path.join(output_dir, 'power_consumption.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\n✓ Power analysis saved to: {output_file}")
    print("\nNOTE: These are rough estimates. For accurate power analysis:")
    print("  - Measure actual battery capacity")
    print("  - Account for battery chemistry (LiPo discharge curve)")
    print("  - Measure current directly with a current sensor")

def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("Usage: python advanced_features.py <csv_file> [output_folder]")
        print("\nExample:")
        print("  python advanced_features.py test.csv")
        print("  python advanced_features.py test.csv my_analysis")
        print("\nThis script provides 5 advanced analysis features:")
        print("  1. Animated Flight Replay (GIF)")
        print("  2. Drag Coefficient Calculation")
        print("  3. Parachute Deployment Detection")
        print("  4. 3D Orientation Visualization")
        print("  5. Power Consumption Analysis")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'advanced_features'
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    print("="*80)
    print("ADVANCED FLIGHT ANALYSIS FEATURES")
    print("="*80)
    print(f"\nInput: {csv_file}")
    print(f"Output: {output_dir}/")
    print("\nRunning all 5 advanced analyses...")
    
    # Load data
    df = pd.read_csv(csv_file)
    
    # Run all features
    try:
        create_animated_replay(df, output_dir)
    except Exception as e:
        print(f"\nError in animation: {e}")
    
    try:
        calculate_drag_coefficient(df, output_dir)
    except Exception as e:
        print(f"\nError in drag calculation: {e}")
    
    try:
        analyze_parachute_deployment(df, output_dir)
    except Exception as e:
        print(f"\nError in parachute analysis: {e}")
    
    try:
        visualize_3d_orientation(df, output_dir)
    except Exception as e:
        print(f"\nError in 3D visualization: {e}")
    
    try:
        analyze_power_consumption(df, output_dir)
    except Exception as e:
        print(f"\nError in power analysis: {e}")
    
    print("\n" + "="*80)
    print("ALL ANALYSES COMPLETE!")
    print("="*80)
    print(f"\nGenerated files in {output_dir}/:")
    print("  - flight_replay.gif (animated trajectory)")
    print("  - drag_coefficient_analysis.png")
    print("  - parachute_deployment.png")
    print("  - 3d_orientation.png")
    print("  - power_consumption.png")
    print("\n" + "="*80)

if __name__ == '__main__':
    main()