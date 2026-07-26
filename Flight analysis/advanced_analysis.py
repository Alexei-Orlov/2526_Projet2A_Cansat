#!/usr/bin/env python3
"""Advanced telemetry analyses for DATA_xxx.CSV files.

Features:
  1. Rotation rate analysis (gyroscope)
  2. GPS trajectory (top view + altitude profile)
  3. Anomaly detection (data gaps, GPS jumps, accel saturation)
  4. Mechanical energy analysis (potential + kinetic, vertical speed derived)
  5. KML export for Google Earth

Usage:
  python advanced_analysis.py DATA_001.CSV
  python advanced_analysis.py DATA_001.CSV --output ./analysis_out
  python advanced_analysis.py DATA_001.CSV --t-start 120 --t-end 180
"""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import lidar_common as lc


def _prepare(df):
    df = df.copy().reset_index(drop=True)
    df['_t'] = df[lc.TIMESTAMP_COL] / 1000.0
    df['_t_rel'] = df['_t'] - df['_t'].iloc[0]
    if lc.ALT_COL in df.columns:
        raw = df[lc.ALT_COL].diff() / df['_t'].diff()
        df['vertical_speed'] = raw.rolling(5, center=True, min_periods=1).mean()
    return df


# ---------------------------------------------------------------------------
# 1. Rotation rate
# ---------------------------------------------------------------------------

def analyze_rotation_rate(df, output_dir):
    if not all(c in df.columns for c in ('gyro_x', 'gyro_y', 'gyro_z')):
        return ['Rotation analysis skipped (no gyro columns)']

    df['_rot_mag'] = np.sqrt(df['gyro_x']**2 + df['gyro_y']**2 + df['gyro_z']**2)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7))

    ax1.plot(df['_t'], df['gyro_x'], label='X', alpha=0.8)
    ax1.plot(df['_t'], df['gyro_y'], label='Y', alpha=0.8)
    ax1.plot(df['_t'], df['gyro_z'], label='Z', alpha=0.8)
    ax1.set_ylabel('Angular rate (deg/s)')
    ax1.set_title('Gyroscope components')
    ax1.legend(); ax1.grid(True, alpha=0.3)

    ax2.plot(df['_t'], df['_rot_mag'], color='red', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Total rotation rate (deg/s)')
    ax2.set_title('Rotation magnitude')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(output_dir, 'rotation_analysis.png')
    plt.savefig(out, dpi=150); plt.close()

    return [
        f'Rotation analysis -> {out}',
        f'Max rotation:  {df["_rot_mag"].max():.1f} deg/s',
        f'Mean rotation: {df["_rot_mag"].mean():.1f} deg/s',
    ]


# ---------------------------------------------------------------------------
# 2. GPS trajectory
# ---------------------------------------------------------------------------

def analyze_gps_trajectory(df, output_dir):
    gps = df[[lc.LAT_COL, lc.LON_COL, lc.ALT_COL, '_t']].dropna()
    if len(gps) == 0:
        return ['GPS trajectory skipped (no data)']

    # Filter out zero-fix rows
    gps = gps[(gps[lc.LAT_COL] != 0) | (gps[lc.LON_COL] != 0)]
    if len(gps) < 2:
        return ['GPS trajectory skipped (all coordinates are 0 — no fix)']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    sc = ax1.scatter(gps[lc.LON_COL], gps[lc.LAT_COL],
                     c=gps[lc.ALT_COL], cmap='viridis', s=20)
    ax1.plot(gps[lc.LON_COL], gps[lc.LAT_COL], 'k--', alpha=0.3, linewidth=1)
    ax1.plot(gps[lc.LON_COL].iloc[0], gps[lc.LAT_COL].iloc[0],
             'go', markersize=12, label='Start')
    ax1.plot(gps[lc.LON_COL].iloc[-1], gps[lc.LAT_COL].iloc[-1],
             'rs', markersize=12, label='End')
    ax1.set_xlabel('Longitude (deg)'); ax1.set_ylabel('Latitude (deg)')
    ax1.set_title('GPS trajectory'); ax1.legend(); ax1.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax1, label='Altitude (m)')

    lat0 = gps[lc.LAT_COL].iloc[0]
    lon0 = gps[lc.LON_COL].iloc[0]
    horiz = np.sqrt(((gps[lc.LAT_COL] - lat0) * 111320)**2 +
                    ((gps[lc.LON_COL] - lon0) * 111320 * np.cos(np.radians(lat0)))**2)

    ax2.plot(horiz, gps[lc.ALT_COL], linewidth=2)
    ax2.set_xlabel('Horizontal distance (m)'); ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs horizontal distance'); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(output_dir, 'gps_trajectory.png')
    plt.savefig(out, dpi=150); plt.close()

    return [
        f'GPS trajectory -> {out}',
        f'Total horizontal displacement: {horiz.iloc[-1]:.1f} m',
    ]


# ---------------------------------------------------------------------------
# 3. Anomaly detection
# ---------------------------------------------------------------------------

def detect_anomalies(df):
    results = ['=' * 70, 'ANOMALY DETECTION', '=' * 70]

    dt = df['_t'].diff()
    expected = dt.median()
    gaps = dt[dt > 3 * expected]
    if len(gaps):
        results.append(f'WARNING: {len(gaps)} data gaps > 3x median interval ({expected*1000:.0f} ms)')
        for ts in gaps.index[:5]:
            results.append(f'  gap at t={df.loc[ts, "_t"]:.2f}s  ({dt[ts]*1000:.0f} ms)')
    else:
        results.append('OK: no significant data gaps')

    gps = df[[lc.LAT_COL, lc.LON_COL]].dropna()
    gps_valid = gps[(gps[lc.LAT_COL] != 0) | (gps[lc.LON_COL] != 0)]
    if len(gps_valid) > 1:
        jumps = ((gps_valid[lc.LAT_COL].diff().abs() > 0.001) |
                 (gps_valid[lc.LON_COL].diff().abs() > 0.001)).sum()
        if jumps:
            results.append(f'WARNING: {jumps} GPS position jumps > ~110 m')
        else:
            results.append('OK: no large GPS jumps')

    if 'accel_x' in df.columns:
        mag = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
        sat = (mag > 15).sum()
        if sat:
            results.append(f'WARNING: {sat} rows with |accel| > 15 m/s² (near BNO055 saturation)')
        else:
            results.append('OK: no accelerometer saturation events')

    if 'temperature' in df.columns:
        spikes = df['temperature'].diff().abs()
        n = (spikes > 5).sum()
        if n:
            results.append(f'WARNING: {n} temperature spikes > 5°C between consecutive rows')
        else:
            results.append('OK: no temperature spikes')

    return results


# ---------------------------------------------------------------------------
# 4. Energy analysis
# ---------------------------------------------------------------------------

def calculate_energy(df, output_dir, mass_kg=0.5):
    if lc.ALT_COL not in df.columns or 'vertical_speed' not in df.columns:
        return ['Energy analysis skipped (altitude or vertical_speed missing)']

    g = 9.81
    alt0 = df[lc.ALT_COL].dropna().iloc[0]
    df = df.copy()
    df['E_pot'] = mass_kg * g * (df[lc.ALT_COL] - alt0)
    df['E_kin'] = 0.5 * mass_kg * df['vertical_speed']**2
    df['E_tot'] = df['E_pot'] + df['E_kin']

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle(f'Mechanical energy — mass {mass_kg} kg', fontweight='bold')

    ax1.plot(df['_t'], df[lc.ALT_COL], linewidth=2)
    ax1.set_ylabel('Altitude (m)'); ax1.grid(True, alpha=0.3)

    ax2.plot(df['_t'], df['E_pot'], label='Potential (mgh)')
    ax2.plot(df['_t'], df['E_kin'], label='Kinetic (½mv²)')
    ax2.plot(df['_t'], df['E_tot'], label='Total', linestyle='--', linewidth=2)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Energy (J)')
    ax2.legend(); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(output_dir, 'energy_analysis.png')
    plt.savefig(out, dpi=150); plt.close()

    return [
        f'Energy analysis -> {out}',
        f'Max potential energy: {df["E_pot"].max():.2f} J',
        f'Max kinetic energy:   {df["E_kin"].max():.2f} J',
    ]


# ---------------------------------------------------------------------------
# 5. KML export
# ---------------------------------------------------------------------------

def export_kml(df, output_dir):
    gps = df[[lc.LAT_COL, lc.LON_COL, lc.ALT_COL]].dropna()
    gps = gps[(gps[lc.LAT_COL] != 0) | (gps[lc.LON_COL] != 0)]
    if len(gps) == 0:
        return ['KML export skipped (no valid GPS fix)']

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
        '  <name>CanSat flight trajectory</name>',
        '  <Placemark><LineString>',
        '    <altitudeMode>absolute</altitudeMode><coordinates>',
    ]
    for _, r in gps.iterrows():
        lines.append(f'      {r[lc.LON_COL]},{r[lc.LAT_COL]},{r[lc.ALT_COL]}')
    lines += ['    </coordinates></LineString></Placemark>',
              '</Document></kml>']

    out = os.path.join(output_dir, 'flight_trajectory.kml')
    with open(out, 'w') as f:
        f.write('\n'.join(lines))
    return [f'KML exported -> {out}  (open in Google Earth)']


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run_all_analyses(csv_file, output_dir='data_out/advanced_analysis_output',
                     t_start=None, t_end=None):
    os.makedirs(output_dir, exist_ok=True)
    df = lc.filter_time_window(pd.read_csv(csv_file), t_start, t_end)
    if df.empty:
        raise SystemExit(f'No data in the requested time window [{t_start}, {t_end}]s')
    df = _prepare(df)

    results = []
    if t_start is not None or t_end is not None:
        results.append(f'Time window: [{df["_t"].iloc[0]:.2f}s -> {df["_t"].iloc[-1]:.2f}s]'
                       f'  ({len(df)} rows)')
    for fn in (analyze_rotation_rate, analyze_gps_trajectory,
               calculate_energy, export_kml):
        try:
            results.extend(fn(df, output_dir))
        except Exception as e:
            results.append(f'  ERROR in {fn.__name__}: {e}')

    results.extend(detect_anomalies(df))

    report = os.path.join(output_dir, 'analysis_report.txt')
    with open(report, 'w', encoding='utf-8') as f:
        f.write('\n'.join(results))
    return results, report


def main():
    parser = argparse.ArgumentParser(
        description='Advanced telemetry analyses for DATA_xxx.CSV')
    parser.add_argument('input', help='Path to DATA_xxx.CSV')
    parser.add_argument('--output', '-o', default='data_out/advanced_analysis_output')
    lc.add_time_window_args(parser)
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f'Error: {args.input} not found'); sys.exit(1)

    print('=' * 70)
    print('ADVANCED TELEMETRY ANALYSIS')
    print('=' * 70)
    results, report = run_all_analyses(args.input, args.output,
                                       t_start=args.t_start, t_end=args.t_end)
    for line in results:
        print(line)
    print(f'\nReport saved to {report}')
    print('=' * 70)


if __name__ == '__main__':
    main()
