#!/usr/bin/env python3
"""
Signal Processing Tutorial for Telemetry Data
Demonstrates: Low-Pass Filtering, FFT Analysis, and MATLAB/Octave Export

This script teaches you signal processing techniques step-by-step!
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
from scipy.io import savemat
import os
import sys

def tutorial_lowpass_filter(df, output_dir='signal_processing_tutorial'):
    """
    TUTORIAL 1: LOW-PASS FILTERING (Removing Noise)
    
    WHY? Sensors always have noise. A low-pass filter removes high-frequency
    noise while keeping the real signal (low-frequency changes).
    
    ANALOGY: Like noise-canceling headphones - removes the hiss, keeps the music!
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("TUTORIAL 1: LOW-PASS FILTERING")
    print("="*80)
    print("\nCONCEPT: Remove high-frequency noise, keep low-frequency signal")
    print("\nWHY WE NEED IT:")
    print("  - ADC measurements have random noise (±0.1 units)")
    print("  - Vibrations cause high-frequency oscillations")
    print("  - GPS coordinates jitter due to signal reflections")
    print("  - We want the TREND, not the noise!\n")
    
    # Example 1: Filter temperature data
    if 'temperature' in df.columns:
        print("Example 1: Smoothing Temperature Data")
        print("-" * 80)
        
        temp_data = df[['timestamp', 'temperature']].dropna()
        
        if len(temp_data) > 11:  # Need at least 11 points for filter
            # Savitzky-Golay filter: fits a polynomial to a sliding window
            # window_length=11 means it looks at 11 points at a time
            # polyorder=3 means it fits a cubic polynomial (smooth curve)
            temp_filtered = signal.savgol_filter(temp_data['temperature'], 
                                                 window_length=11, 
                                                 polyorder=3)
            
            # Calculate noise reduction
            noise_before = np.std(np.diff(temp_data['temperature']))
            noise_after = np.std(np.diff(temp_filtered))
            
            print(f"  Raw data noise (std of differences): {noise_before:.3f} C")
            print(f"  Filtered data noise:                 {noise_after:.3f} C")
            print(f"  Noise reduction:                     {100*(1-noise_after/noise_before):.1f}%")
            
            # Plot comparison
            plt.figure(figsize=(14, 5))
            
            plt.subplot(1, 2, 1)
            plt.plot(temp_data['timestamp'], temp_data['temperature'], 
                    'o-', alpha=0.4, label='Raw (noisy)', markersize=4)
            plt.plot(temp_data['timestamp'], temp_filtered, 
                    linewidth=3, label='Filtered (smooth)', color='red')
            plt.xlabel('Time (s)')
            plt.ylabel('Temperature (C)')
            plt.title('Low-Pass Filter Effect on Temperature')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # Subplot 2: Show the noise we removed
            plt.subplot(1, 2, 2)
            noise = temp_data['temperature'].values - temp_filtered
            plt.plot(temp_data['timestamp'], noise, alpha=0.7)
            plt.axhline(0, color='red', linestyle='--', linewidth=2)
            plt.xlabel('Time (s)')
            plt.ylabel('Removed Noise (C)')
            plt.title('The Noise We Filtered Out')
            plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, '1_lowpass_temperature.png'), dpi=150)
            plt.close()
            
            print(f"  Plot saved: {output_dir}/1_lowpass_temperature.png\n")
    
    # Example 2: Filter GPS coordinates
    if 'latitude' in df.columns and 'longitude' in df.columns:
        print("Example 2: Smoothing GPS Trajectory")
        print("-" * 80)
        
        gps_data = df[['timestamp', 'latitude', 'longitude']].dropna()
        
        if len(gps_data) > 5:
            # For GPS, use smaller window (5 points) since we have fewer samples
            lat_filtered = signal.savgol_filter(gps_data['latitude'], 
                                                window_length=min(5, len(gps_data)|1), 
                                                polyorder=2)
            lon_filtered = signal.savgol_filter(gps_data['longitude'], 
                                                window_length=min(5, len(gps_data)|1), 
                                                polyorder=2)
            
            plt.figure(figsize=(14, 5))
            
            plt.subplot(1, 2, 1)
            plt.plot(gps_data['longitude'], gps_data['latitude'], 
                    'o-', alpha=0.4, label='Raw GPS', markersize=6)
            plt.plot(lon_filtered, lat_filtered, 
                    linewidth=3, label='Filtered Path', color='red')
            plt.xlabel('Longitude')
            plt.ylabel('Latitude')
            plt.title('GPS Path: Raw vs Filtered')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # Calculate how much jitter we removed
            raw_jitter = np.sqrt(np.diff(gps_data['latitude'])**2 + 
                                np.diff(gps_data['longitude'])**2).mean()
            filtered_jitter = np.sqrt(np.diff(lat_filtered)**2 + 
                                     np.diff(lon_filtered)**2).mean()
            
            plt.subplot(1, 2, 2)
            plt.text(0.1, 0.8, 'GPS Smoothing Results:', fontsize=14, fontweight='bold')
            plt.text(0.1, 0.6, f'Raw path jitter: {raw_jitter*111000:.1f} meters/step', fontsize=12)
            plt.text(0.1, 0.5, f'Filtered jitter: {filtered_jitter*111000:.1f} meters/step', fontsize=12)
            plt.text(0.1, 0.4, f'Improvement: {100*(1-filtered_jitter/raw_jitter):.1f}%', 
                    fontsize=12, color='green', fontweight='bold')
            plt.text(0.1, 0.2, 'WHY THIS MATTERS:\nFiltered path shows where you\nACTUALLY went, not GPS noise!', 
                    fontsize=10, style='italic')
            plt.xlim(0, 1)
            plt.ylim(0, 1)
            plt.axis('off')
            
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, '1_lowpass_gps.png'), dpi=150)
            plt.close()
            
            print(f"  Raw GPS jitter:      {raw_jitter*111000:.1f} m per step")
            print(f"  Filtered jitter:     {filtered_jitter*111000:.1f} m per step")
            print(f"  Improvement:         {100*(1-filtered_jitter/raw_jitter):.1f}%")
            print(f"  Plot saved: {output_dir}/1_lowpass_gps.png\n")
    
    print("KEY TAKEAWAY:")
    print("  Low-pass filters smooth out noise while preserving the real signal trend.")
    print("  Use them whenever your data looks 'jittery' or 'noisy'!\n")

def tutorial_fft_analysis(df, output_dir='signal_processing_tutorial'):
    """
    TUTORIAL 2: FFT (Fast Fourier Transform) - Finding Hidden Frequencies
    
    WHY? Time-domain data hides repeating patterns. FFT reveals them!
    
    ANALOGY: Like a music equalizer - shows which frequencies are present.
    If you hear a sound, FFT tells you it's 440 Hz (note: A)
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("TUTORIAL 2: FFT (FREQUENCY ANALYSIS)")
    print("="*80)
    print("\nCONCEPT: Convert time-domain signal -> frequency-domain spectrum")
    print("\nWHY WE NEED IT:")
    print("  - Find vibration frequencies (motor @ 50 Hz?)")
    print("  - Measure tumbling rate (spinning how fast?)")
    print("  - Detect oscillations (parachute swinging?)")
    print("  - Identify noise sources (what's causing the jitter?)\n")
    
    print("HOW FFT WORKS:")
    print("  Time Domain:      'Signal varies over time'")
    print("                    Example: [1, 2, 1, 2, 1, 2, ...] over 3 seconds")
    print("  Frequency Domain: 'Signal has a component at 0.5 Hz'")
    print("                    (0.5 Hz = one cycle every 2 seconds)\n")
    
    # Calculate sampling rate
    time_diff = df['timestamp'].diff().median()
    sampling_rate = 1.0 / time_diff
    
    print(f"Your sampling rate: {sampling_rate:.2f} Hz (one sample every {time_diff:.3f} seconds)")
    print(f"Nyquist frequency: {sampling_rate/2:.2f} Hz (max detectable frequency)")
    print("  ^ You can only detect frequencies up to half your sampling rate!\n")
    
    # Example 1: Gyroscope FFT (find rotation rate)
    if 'gyro_z' in df.columns:
        print("Example 1: Find Tumbling/Rotation Rate from Gyroscope")
        print("-" * 80)
        
        gyro_data = df['gyro_z'].dropna().values
        
        if len(gyro_data) > 10:
            # Perform FFT
            fft_values = fft(gyro_data)
            frequencies = fftfreq(len(gyro_data), d=time_diff)
            
            # Only look at positive frequencies (negative are mirror image)
            positive_freq_idx = frequencies > 0
            frequencies = frequencies[positive_freq_idx]
            fft_magnitude = np.abs(fft_values[positive_freq_idx])
            
            # Find dominant frequency (excluding DC component at 0 Hz)
            non_dc = frequencies > 0.1  # Ignore very low frequencies
            if np.any(non_dc):
                dominant_freq = frequencies[non_dc][np.argmax(fft_magnitude[non_dc])]
                dominant_power = fft_magnitude[non_dc].max()
                
                print(f"  Dominant rotation frequency: {dominant_freq:.2f} Hz")
                print(f"  Period: {1/dominant_freq:.2f} seconds per rotation")
                print(f"  RPM: {dominant_freq * 60:.1f} rotations per minute\n")
                
                # Plot
                fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
                
                # Time domain
                time_axis = np.arange(len(gyro_data)) * time_diff
                ax1.plot(time_axis, gyro_data, linewidth=1)
                ax1.set_xlabel('Time (s)')
                ax1.set_ylabel('Gyro Z (deg/s)')
                ax1.set_title('Time Domain: Gyroscope Data')
                ax1.grid(True, alpha=0.3)
                ax1.text(0.02, 0.95, 'You see oscillations, but how fast?', 
                        transform=ax1.transAxes, fontsize=10, 
                        verticalalignment='top', bbox=dict(boxstyle='round', 
                        facecolor='yellow', alpha=0.5))
                
                # Frequency domain
                ax2.stem(frequencies, fft_magnitude)
                ax2.axvline(dominant_freq, color='red', linestyle='--', linewidth=2, 
                           label=f'Dominant: {dominant_freq:.2f} Hz')
                ax2.set_xlabel('Frequency (Hz)')
                ax2.set_ylabel('Magnitude')
                ax2.set_title('Frequency Domain: FFT Shows Rotation Rate!')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
                ax2.text(0.02, 0.95, f'Peak = {dominant_freq:.2f} Hz -> Rotating {dominant_freq:.2f} times/second!', 
                        transform=ax2.transAxes, fontsize=10, 
                        verticalalignment='top', bbox=dict(boxstyle='round', 
                        facecolor='lightgreen', alpha=0.5))
                
                plt.tight_layout()
                plt.savefig(os.path.join(output_dir, '2_fft_gyroscope.png'), dpi=150)
                plt.close()
                
                print(f"  Plot saved: {output_dir}/2_fft_gyroscope.png\n")
    
    # Example 2: Accelerometer FFT (find vibration frequency)
    if 'accel_z' in df.columns:
        print("Example 2: Detect Vibration Frequencies from Accelerometer")
        print("-" * 80)
        
        accel_data = df['accel_z'].dropna().values
        
        if len(accel_data) > 10:
            # Remove DC component (gravity)
            accel_data = accel_data - np.mean(accel_data)
            
            # Perform FFT
            fft_values = fft(accel_data)
            frequencies = fftfreq(len(accel_data), d=time_diff)
            
            positive_freq_idx = frequencies > 0
            frequencies = frequencies[positive_freq_idx]
            fft_magnitude = np.abs(fft_values[positive_freq_idx])
            
            # Find top 3 frequencies
            top_3_idx = np.argsort(fft_magnitude)[-3:][::-1]
            
            print("  Top 3 vibration frequencies detected:")
            for i, idx in enumerate(top_3_idx, 1):
                if frequencies[idx] > 0.1:  # Skip DC
                    print(f"    {i}. {frequencies[idx]:.2f} Hz (magnitude: {fft_magnitude[idx]:.1f})")
            
            plt.figure(figsize=(14, 5))
            plt.stem(frequencies, fft_magnitude)
            
            # Mark top frequencies
            for idx in top_3_idx:
                if frequencies[idx] > 0.1:
                    plt.axvline(frequencies[idx], color='red', linestyle='--', 
                               alpha=0.5, label=f'{frequencies[idx]:.2f} Hz')
            
            plt.xlabel('Frequency (Hz)')
            plt.ylabel('Magnitude')
            plt.title('Vibration Frequency Spectrum')
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.xlim(0, sampling_rate/2)  # Only show up to Nyquist frequency
            
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, '2_fft_accelerometer.png'), dpi=150)
            plt.close()
            
            print(f"\n  Plot saved: {output_dir}/2_fft_accelerometer.png\n")
    
    print("KEY TAKEAWAY:")
    print("  FFT transforms time-domain signals into frequency-domain spectra.")
    print("  Use it to find: rotation rates, vibration frequencies, oscillation periods!")
    print("  Remember: You can only detect up to Nyquist frequency (half sampling rate).\n")

def export_to_octave(df, output_dir='signal_processing_tutorial'):
    """
    TUTORIAL 3: Export to Octave/MATLAB
    
    WHY? Octave has powerful built-in functions for advanced analysis:
    - Kalman filtering (sensor fusion)
    - Control system design
    - Advanced signal processing toolbox
    - Easy matrix operations
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\n" + "="*80)
    print("TUTORIAL 3: EXPORTING TO OCTAVE")
    print("="*80)
    print("\nWHY USE OCTAVE?")
    print("  - Built-in advanced filters (Butterworth, Chebyshev, Kalman)")
    print("  - Easy matrix math (perfect for sensor data)")
    print("  - Control system toolbox (if you want to model dynamics)")
    print("  - Free and open-source (unlike MATLAB!)\n")
    
    # Prepare data dictionary
    export_data = {}
    
    print("Preparing data for export...")
    print("-" * 80)
    
    # Export all columns
    for col in df.columns:
        clean_data = df[col].dropna().values
        if len(clean_data) > 0:
            export_data[col.replace(' ', '_')] = clean_data
            print(f"  ✓ {col:20s} ({len(clean_data)} samples)")
    
    # Add sampling rate info
    time_diff = df['timestamp'].diff().median()
    export_data['sampling_rate'] = np.array([1.0 / time_diff])
    export_data['sampling_period'] = np.array([time_diff])
    
    print(f"\n  Sampling rate: {1/time_diff:.2f} Hz")
    print(f"  Sampling period: {time_diff:.3f} seconds")
    
    # Save to .mat file
    output_file = os.path.join(output_dir, 'telemetry_data.mat')
    savemat(output_file, export_data)
    
    print(f"\n  ✓ Data exported to: {output_file}\n")
    
    # Create Octave tutorial script
    create_octave_tutorial(output_dir)
    
    print("KEY TAKEAWAY:")
    print("  Your data is now in .mat format, ready for Octave!")
    print("  Run the tutorial scripts in Octave to learn advanced techniques.\n")

def create_octave_tutorial(output_dir):
    """
    Create comprehensive Octave tutorial scripts
    """
    
    # Script 1: Basic loading and plotting
    script1 = """% ============================================================================
% OCTAVE TUTORIAL 1: Loading and Plotting Data
% ============================================================================
% This script teaches you the BASICS of Octave
%
% RUN THIS FIRST to make sure everything works!
% ============================================================================

clear all;  % Clear all variables from memory
close all;  % Close all plot windows
clc;        % Clear command window

disp('==================================================');
disp('TUTORIAL 1: LOADING AND PLOTTING');
disp('==================================================');
disp(' ');

% STEP 1: Load the data
% --------------------
% The 'load' command reads .mat files
% All variables are automatically created in your workspace

disp('Loading telemetry_data.mat...');
load('telemetry_data.mat');

% Check what variables we have
disp('Available variables:');
whos  % This shows all loaded variables and their sizes

disp(' ');
disp('Press ENTER to continue...'); pause;

% STEP 2: Basic plotting
% ----------------------
% Octave uses 'plot()' just like MATLAB

figure(1);  % Create figure window 1
plot(timestamp, altitude);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs Time');
grid on;  % Add grid lines

disp('Plot created! Check Figure 1.');
disp(' ');
disp('Press ENTER to continue...'); pause;

% STEP 3: Multiple subplots
% -------------------------
% subplot(rows, cols, index) creates subplot grids

figure(2);

subplot(3, 1, 1);  % 3 rows, 1 column, position 1
plot(timestamp, accel_x);
ylabel('Accel X (m/s²)');
title('Accelerometer Data');
grid on;

subplot(3, 1, 2);
plot(timestamp, accel_y);
ylabel('Accel Y (m/s²)');
grid on;

subplot(3, 1, 3);
plot(timestamp, accel_z);
xlabel('Time (s)');
ylabel('Accel Z (m/s²)');
grid on;

disp('Multiple subplots created! Check Figure 2.');
disp(' ');
disp('Press ENTER to continue...'); pause;

% STEP 4: Basic statistics
% ------------------------
% Octave has built-in statistical functions

disp('BASIC STATISTICS:');
disp('==================');
fprintf('Altitude: min=%.2f m, max=%.2f m, mean=%.2f m\\n', ...
        min(altitude), max(altitude), mean(altitude));

fprintf('Temperature: min=%.1f C, max=%.1f C, mean=%.1f C\\n', ...
        min(temperature), max(temperature), mean(temperature));

% Calculate standard deviation (how much data varies)
temp_std = std(temperature);
fprintf('Temperature std dev: %.2f C (measures spread)\\n', temp_std);

disp(' ');
disp('TUTORIAL 1 COMPLETE!');
disp('You now know how to load data and make basic plots.');
disp(' ');
disp('Next: Run octave_tutorial_2_filtering.m');
"""
    
    with open(os.path.join(output_dir, 'octave_tutorial_1_basics.m'), 'w', encoding='utf-8') as f:
        f.write(script1)
    
    # Script 2: Filtering
    script2 = """% ============================================================================
% OCTAVE TUTORIAL 2: Signal Filtering
% ============================================================================
% This script teaches you FILTERING techniques
%
% YOU'LL LEARN:
%   - Moving average (simplest filter)
%   - Butterworth low-pass filter (professional filter)
%   - How to choose filter parameters
% ============================================================================

clear all;
close all;
clc;

disp('==================================================');
disp('TUTORIAL 2: SIGNAL FILTERING');
disp('==================================================');
disp(' ');

load('telemetry_data.mat');

%% METHOD 1: Moving Average Filter (Simplest!)
%  ============================================
%  CONCEPT: Replace each point with average of nearby points
%  
%  Example: [1, 5, 2, 6, 3] with window=3
%  Point 2: avg(1,5,2) = 2.67
%  Point 3: avg(5,2,6) = 4.33
%  Point 4: avg(2,6,3) = 3.67

disp('METHOD 1: Moving Average Filter');
disp('--------------------------------');

window_size = 5;  % Average over 5 points
disp(sprintf('Using window size: %d points', window_size));

% Apply moving average using 'filter' function
% ones(1,window_size) creates [1,1,1,1,1]
% Dividing by window_size gives average
b = ones(1, window_size) / window_size;
temp_smoothed_ma = filter(b, 1, temperature);

% Plot comparison
figure(1);
plot(timestamp, temperature, 'b-', 'DisplayName', 'Raw');
hold on;  % Keep this plot, add more to it
plot(timestamp, temp_smoothed_ma, 'r-', 'LineWidth', 2, 'DisplayName', 'Moving Avg');
hold off;
xlabel('Time (s)');
ylabel('Temperature (C)');
title('Moving Average Filter');
legend('show');
grid on;

% Calculate noise reduction
noise_before = std(diff(temperature));  % diff() computes differences
noise_after = std(diff(temp_smoothed_ma));
reduction = 100 * (1 - noise_after/noise_before);

fprintf('Noise reduction: %.1f%%\\n', reduction);

disp(' ');
disp('Press ENTER to continue...'); pause;

%% METHOD 2: Butterworth Low-Pass Filter (Professional!)
%  ======================================================
%  CONCEPT: Removes frequencies above a cutoff
%  
%  Cutoff frequency: 0.5 Hz means "keep frequencies below 0.5 Hz"
%  If your data changes faster than 0.5 Hz, it's probably noise!

disp(' ');
disp('METHOD 2: Butterworth Filter');
disp('-----------------------------');

% Design parameters
cutoff_freq = 0.3;  % Hz (cycles per second)
filter_order = 4;   % Higher order = sharper cutoff

disp(sprintf('Cutoff frequency: %.1f Hz', cutoff_freq));
disp(sprintf('Filter order: %d', filter_order));

% Design the filter
% butter() returns filter coefficients
% sampling_rate tells it your data rate
[b, a] = butter(filter_order, cutoff_freq / (sampling_rate/2));

% Apply filter
temp_smoothed_butter = filtfilt(b, a, temperature);
% filtfilt applies filter forwards AND backwards (no phase shift!)

% Plot all three together
figure(2);
plot(timestamp, temperature, 'b-', 'DisplayName', 'Raw');
hold on;
plot(timestamp, temp_smoothed_ma, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Moving Avg');
plot(timestamp, temp_smoothed_butter, 'r-', 'LineWidth', 2, 'DisplayName', 'Butterworth');
hold off;
xlabel('Time (s)');
ylabel('Temperature (C)');
title('Comparison of Filters');
legend('show');
grid on;

% Calculate noise for Butterworth
noise_butter = std(diff(temp_smoothed_butter));
reduction_butter = 100 * (1 - noise_butter/noise_before);

fprintf('\\nNoise Reduction Comparison:\\n');
fprintf('  Moving Average: %.1f%%\\n', reduction);
fprintf('  Butterworth:    %.1f%%\\n', reduction_butter);

disp(' ');
disp('WHICH FILTER TO USE?');
disp('  - Moving Average: Simple, fast, good for quick smoothing');
disp('  - Butterworth: Professional, preserves signal shape better');

disp(' ');
disp('TUTORIAL 2 COMPLETE!');
disp('Next: Run octave_tutorial_3_fft.m');
"""
    
    with open(os.path.join(output_dir, 'octave_tutorial_2_filtering.m'), 'w', encoding='utf-8') as f:
        f.write(script2)
    
    # Script 3: FFT Analysis
    script3 = """% ============================================================================
% OCTAVE TUTORIAL 3: FFT (Frequency Analysis)
% ============================================================================
% This script teaches you FREQUENCY ANALYSIS
%
% YOU'LL LEARN:
%   - What FFT actually does
%   - How to interpret frequency spectra
%   - Finding dominant frequencies (rotation rates, vibrations)
% ============================================================================

clear all;
close all;
clc;

disp('==================================================');
disp('TUTORIAL 3: FFT (FREQUENCY ANALYSIS)');
disp('==================================================');
disp(' ');

load('telemetry_data.mat');

%% CONCEPT: What is FFT?
%  =====================
%  FFT = Fast Fourier Transform
%  
%  It answers: "Which frequencies are present in my signal?"
%  
%  Example:
%    Time domain: [1,0,-1,0,1,0,-1,0,...]  <- Looks like oscillation
%    Frequency domain: Peak at 0.25 Hz     <- Oscillates 0.25 times/second
%
%  REAL WORLD USES:
%    - Find motor vibration frequency (50 Hz?)
%    - Measure satellite tumbling rate
%    - Detect oscillations after parachute opens

disp('Analyzing Gyroscope Data (rotation rate)...');
disp(' ');

% Remove DC component (average value)
gyro_z_ac = gyro_z - mean(gyro_z);

% Compute FFT
N = length(gyro_z_ac);  % Number of samples
Y = fft(gyro_z_ac);     % Compute FFT

% Create frequency axis
% Frequencies range from 0 to sampling_rate
f = sampling_rate * (0:(N/2)) / N;

% Take magnitude and normalize
% We only use first half (positive frequencies)
P2 = abs(Y / N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);

% Find dominant frequency (ignore DC at f=0)
[max_mag, max_idx] = max(P1(2:end));
dominant_freq = f(max_idx + 1);

fprintf('RESULTS:\\n');
fprintf('--------\\n');
fprintf('Dominant frequency: %.3f Hz\\n', dominant_freq);
fprintf('Period: %.2f seconds per rotation\\n', 1/dominant_freq);
fprintf('RPM: %.1f rotations per minute\\n', dominant_freq * 60);

% Plot
figure(1);

subplot(2, 1, 1);
time_axis = (0:N-1) / sampling_rate;
plot(time_axis, gyro_z);
xlabel('Time (s)');
ylabel('Gyro Z (deg/s)');
title('Time Domain: Gyroscope Signal');
grid on;

subplot(2, 1, 2);
plot(f, P1, 'LineWidth', 2);
hold on;
% Mark dominant frequency
plot(dominant_freq, max_mag, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(dominant_freq, max_mag*1.1, sprintf('  %.3f Hz', dominant_freq));
hold off;
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency Domain: FFT Spectrum');
grid on;
xlim([0 sampling_rate/2]);  % Only show up to Nyquist frequency

disp(' ');
disp('The peak in the frequency plot shows your rotation rate!');

disp(' ');
disp('Press ENTER to continue...'); pause;

%% PRACTICAL EXAMPLE: Filter Out Specific Frequency
%  =================================================
%  Say motor vibrates at 1.2 Hz and you want to remove it

disp(' ');
disp('PRACTICAL EXAMPLE: Removing Specific Frequency');
disp('----------------------------------------------');

% Let's remove the dominant frequency we found
freq_to_remove = dominant_freq;

% Design notch filter (removes one specific frequency)
% Quality factor Q=10 means narrow notch
Q = 10;
wo = freq_to_remove / (sampling_rate/2);  % Normalized frequency
bw = wo / Q;

% Create notch filter
[b_notch, a_notch] = iirnotch(wo, bw);

% Apply filter
gyro_z_filtered = filtfilt(b_notch, a_notch, gyro_z);

% Compute FFT of filtered signal
Y_filtered = fft(gyro_z_filtered - mean(gyro_z_filtered));
P2_filtered = abs(Y_filtered / N);
P1_filtered = P2_filtered(1:floor(N/2)+1);
P1_filtered(2:end-1) = 2 * P1_filtered(2:end-1);

% Plot comparison
figure(2);
subplot(2, 1, 1);
plot(f, P1, 'b-', 'DisplayName', 'Original');
hold on;
plot(f, P1_filtered, 'r-', 'DisplayName', 'Notch Filtered');
hold off;
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title(sprintf('Notch Filter Removed %.3f Hz', freq_to_remove));
legend('show');
grid on;
xlim([0 sampling_rate/2]);

subplot(2, 1, 2);
plot(time_axis, gyro_z, 'b-', 'DisplayName', 'Original');
hold on;
plot(time_axis, gyro_z_filtered, 'r-', 'DisplayName', 'Filtered');
hold off;
xlabel('Time (s)');
ylabel('Gyro Z (deg/s)');
title('Time Domain: Oscillation Removed');
legend('show');
grid on;

disp(' ');
disp('TUTORIAL 3 COMPLETE!');
disp('Next: Run octave_tutorial_4_kalman.m');
"""
    
    with open(os.path.join(output_dir, 'octave_tutorial_3_fft.m'), 'w', encoding='utf-8') as f:
        f.write(script3)
    
    # Script 4: Kalman Filter (Advanced!)
    script4 = """% ============================================================================
% OCTAVE TUTORIAL 4: Kalman Filter (ADVANCED!)
% ============================================================================
% This script teaches you the KALMAN FILTER
%
% WHAT IS IT?
%   The Kalman filter is an OPTIMAL estimator that combines:
%   - Noisy measurements (what sensors tell us)
%   - Physical model (what we expect based on physics)
%   
%   Result: Best possible estimate of true state!
%
% REAL WORLD USES:
%   - GPS + IMU sensor fusion (your phone does this!)
%   - Rocket guidance
%   - Self-driving cars
%   - Weather prediction
% ============================================================================

clear all;
close all;
clc;

disp('==================================================');
disp('TUTORIAL 4: KALMAN FILTER');
disp('==================================================');
disp(' ');

load('telemetry_data.mat');

%% THE PROBLEM
%  ===========
%  GPS gives noisy position measurements
%  We want smooth, accurate position estimate
%
%  SOLUTION: Kalman filter combines:
%    1. Physics (objects don't teleport!)
%    2. Measurements (GPS readings)

disp('THE PROBLEM: Noisy GPS Measurements');
disp('------------------------------------');
disp(' ');
disp('GPS coordinates jump around due to:');
disp('  - Satellite geometry changes');
disp('  - Signal reflections (multipath)');
disp('  - Atmospheric effects');
disp(' ');
disp('Kalman filter will smooth this!');
disp(' ');
disp('Press ENTER to start filtering...'); pause;

%% KALMAN FILTER SETUP
%  ===================

% State vector: [position; velocity]
% We're tracking altitude and its rate of change

% Get altitude data
z = altitude;  % Measurements
N = length(z);

% Initialize state estimate
x = [z(1); 0];  % Start at first altitude with 0 velocity

% State transition matrix (physics model)
% Next position = current position + velocity * dt
% Next velocity = current velocity (constant velocity model)
dt = sampling_period;
A = [1, dt;
     0,  1];

% Measurement matrix (we only measure position, not velocity)
H = [1, 0];

% Process noise covariance (how much we trust our physics model)
% Q is small = we trust the model
Q = [0.1, 0;
     0, 0.1];

% Measurement noise covariance (how much we trust GPS)
% R is large = GPS is noisy
R = 5.0;  % Variance of altitude measurements (adjust based on your data)

% Initial estimate covariance
P = [10, 0;
     0, 10];

% Storage for results
x_estimates = zeros(2, N);
x_estimates(:, 1) = x;

disp('KALMAN FILTER PARAMETERS:');
disp('-------------------------');
disp('State: [altitude; vertical_velocity]');
fprintf('Sampling period: %.3f seconds\\n', dt);
fprintf('Process noise: %.2f (trust in physics model)\\n', Q(1,1));
fprintf('Measurement noise: %.2f (trust in GPS)\\n', R);
disp(' ');

%% RUN THE KALMAN FILTER
%  =====================

disp('Running Kalman filter...');

for k = 2:N
    % PREDICT step (use physics model)
    % -------------------------------
    % Where do we think we'll be next?
    x_pred = A * x;           % Predicted state
    P_pred = A * P * A' + Q;  % Predicted covariance
    
    % UPDATE step (incorporate measurement)
    % -----------------------------------
    % Kalman gain: How much do we trust the measurement vs prediction?
    K = P_pred * H' / (H * P_pred * H' + R);
    
    % Update estimate with measurement
    y = z(k) - H * x_pred;  % Innovation (measurement - prediction)
    x = x_pred + K * y;      % Corrected estimate
    P = (eye(2) - K * H) * P_pred;  % Corrected covariance
    
    % Store estimate
    x_estimates(:, k) = x;
end

disp('Done!');
disp(' ');

% Extract filtered altitude and velocity
altitude_filtered = x_estimates(1, :);
velocity_filtered = x_estimates(2, :);

%% RESULTS
%  =======

% Calculate improvements
raw_noise = std(diff(altitude));
filtered_noise = std(diff(altitude_filtered));
improvement = 100 * (1 - filtered_noise / raw_noise);

fprintf('RESULTS:\\n');
fprintf('--------\\n');
fprintf('Raw altitude noise: %.3f m\\n', raw_noise);
fprintf('Filtered altitude noise: %.3f m\\n', filtered_noise);
fprintf('Improvement: %.1f%%\\n', improvement);
disp(' ');

% Plot results
figure(1);

subplot(3, 1, 1);
time_axis = (0:N-1) * dt;
plot(time_axis, altitude, 'b.', 'MarkerSize', 8, 'DisplayName', 'GPS Measurements');
hold on;
plot(time_axis, altitude_filtered, 'r-', 'LineWidth', 2, 'DisplayName', 'Kalman Filtered');
hold off;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Kalman Filter: Altitude Estimation');
legend('show');
grid on;

subplot(3, 1, 2);
plot(time_axis, velocity_filtered, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Kalman Filter: Estimated Vertical Velocity');
grid on;
text(0.05, 0.95, 'Notice: Kalman filter estimates velocity even though we never measured it!', ...
     'Units', 'normalized', 'BackgroundColor', 'yellow');

subplot(3, 1, 3);
noise = altitude - altitude_filtered';
plot(time_axis, noise, 'k-');
xlabel('Time (s)');
ylabel('Removed Noise (m)');
title('Noise Removed by Kalman Filter');
grid on;

disp(' ');
disp('KEY INSIGHTS:');
disp('-------------');
disp('1. Kalman filter produces SMOOTHER estimates than raw GPS');
disp('2. It also estimates VELOCITY (even though we only measure position!)');
disp('3. It combines physics (model) with measurements (sensors)');
disp(' ');
disp('This is the same technique used in:');
disp('  - Your smartphone GPS');
disp('  - Self-driving cars');
disp('  - Spacecraft navigation');
disp('  - Weather forecasting');

disp(' ');
disp('==================================================');
disp('ALL TUTORIALS COMPLETE!');
disp('==================================================');
disp(' ');
disp('You now know:');
disp('  ✓ How to load and plot data in Octave');
disp('  ✓ How to filter signals (moving average, Butterworth)');
disp('  ✓ How to analyze frequencies with FFT');
disp('  ✓ How to use Kalman filters for optimal estimation');
disp(' ');
disp('NEXT STEPS:');
disp('  1. Experiment with different filter parameters');
disp('  2. Try applying these to other sensors (gyro, accel)');
disp('  3. Read Octave documentation: https://octave.org/doc/');
"""
    
    with open(os.path.join(output_dir, 'octave_tutorial_4_kalman.m'), 'w', encoding='utf-8') as f:
        f.write(script4)
    
    # Create README
    readme = """OCTAVE TUTORIAL SUITE
=====================

This folder contains everything you need to learn signal processing in Octave!

FILES:
------
1. telemetry_data.mat              - Your flight data in MATLAB/Octave format
2. octave_tutorial_1_basics.m      - START HERE! Loading and plotting basics
3. octave_tutorial_2_filtering.m   - Learn filtering techniques
4. octave_tutorial_3_fft.m         - Frequency analysis with FFT
5. octave_tutorial_4_kalman.m      - Advanced: Kalman filtering

HOW TO USE:
-----------
1. Install Octave: https://www.gnu.org/software/octave/
2. Open Octave
3. Navigate to this folder: cd('/path/to/signal_processing_tutorial')
4. Run tutorials in order:
   >> octave_tutorial_1_basics
   >> octave_tutorial_2_filtering
   >> octave_tutorial_3_fft
   >> octave_tutorial_4_kalman

Each tutorial:
- Explains WHAT it does
- Shows WHY it matters
- Demonstrates HOW to do it
- Creates plots to visualize results

WHAT YOU'LL LEARN:
------------------
✓ Octave basics (variables, plotting, functions)
✓ Signal filtering (remove noise)
✓ FFT analysis (find frequencies)
✓ Kalman filtering (optimal estimation)

All tutorials have DETAILED COMMENTS explaining every line!

TIPS:
-----
- Don't rush! Read the comments carefully
- Experiment by changing parameters
- Press ENTER when tutorials pause
- Check the plots that appear

Have fun learning!
"""
    
    with open(os.path.join(output_dir, 'README_OCTAVE.txt'), 'w', encoding='utf-8') as f:
        f.write(readme)
    
    print("\n  ✓ Created 4 Octave tutorial scripts:")
    print(f"    1. {output_dir}/octave_tutorial_1_basics.m")
    print(f"    2. {output_dir}/octave_tutorial_2_filtering.m")
    print(f"    3. {output_dir}/octave_tutorial_3_fft.m")
    print(f"    4. {output_dir}/octave_tutorial_4_kalman.m")
    print(f"  ✓ Created README: {output_dir}/README_OCTAVE.txt")

def main():
    """Main function to run all tutorials"""
    
    if len(sys.argv) < 2:
        print("Usage: python signal_processing_tutorial.py <csv_file> [output_folder]")
        print("\nExample:")
        print("  python signal_processing_tutorial.py test.csv")
        print("  python signal_processing_tutorial.py test.csv my_analysis")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'signal_processing_tutorial'
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    print("\n" + "="*80)
    print("SIGNAL PROCESSING TUTORIAL SUITE")
    print("="*80)
    print(f"\nInput: {csv_file}")
    print(f"Output: {output_dir}/")
    print("\nThis tutorial will teach you:")
    print("  1. Low-pass filtering (remove noise)")
    print("  2. FFT analysis (find frequencies)")
    print("  3. Octave export (advanced analysis)")
    print("\nPress ENTER to start..."); input()
    
    # Load data
    df = pd.read_csv(csv_file)
    
    # Run tutorials
    tutorial_lowpass_filter(df, output_dir)
    tutorial_fft_analysis(df, output_dir)
    export_to_octave(df, output_dir)
    
    print("\n" + "="*80)
    print("TUTORIAL COMPLETE!")
    print("="*80)
    print(f"\nGenerated files in {output_dir}/:")
    print("  Python outputs:")
    print("    - 1_lowpass_temperature.png")
    print("    - 1_lowpass_gps.png")
    print("    - 2_fft_gyroscope.png")
    print("    - 2_fft_accelerometer.png")
    print("\n  Octave files:")
    print("    - telemetry_data.mat")
    print("    - octave_tutorial_1_basics.m")
    print("    - octave_tutorial_2_filtering.m")
    print("    - octave_tutorial_3_fft.m")
    print("    - octave_tutorial_4_kalman.m")
    print("    - README_OCTAVE.txt")
    print("\nNEXT STEPS:")
    print("  1. Review the Python plots to see filtering and FFT results")
    print("  2. Install Octave: https://www.gnu.org/software/octave/")
    print("  3. Run the Octave tutorials in order (1->2->3->4)")
    print("  4. Experiment with different parameters!")
    print("\n" + "="*80)

if __name__ == '__main__':
    main()