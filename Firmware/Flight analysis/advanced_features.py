#!/usr/bin/env python3
"""Advanced physical analysis features for DATA_xxx.CSV telemetry.

Features:
  1. Drag coefficient (Cd) from terminal velocity during DROP phase
  2. Parachute deployment detection (vertical speed jerk)
  3. Power consumption (battery voltage drain per phase)

Usage:
  python advanced_features.py DATA_001.CSV
  python advanced_features.py DATA_001.CSV --output ./features_out
  python advanced_features.py DATA_001.CSV --mass 0.35 --area 0.008
  python advanced_features.py DATA_001.CSV --t-start 120 --t-end 180
"""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal as spsig

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import lidar_common as lc


def _prepare(df):
    df = df.copy().reset_index(drop=True)
    df['_t'] = df[lc.TIMESTAMP_COL] / 1000.0
    df['_t_rel'] = df['_t'] - df['_t'].iloc[0]
    if lc.ALT_COL in df.columns:
        raw = df[lc.ALT_COL].diff() / df['_t'].diff()
        df['vertical_speed'] = raw.rolling(5, center=True, min_periods=1).mean()
    df['_phase'] = df[lc.PHASE_COL].map(lc.decode_phase) if lc.PHASE_COL in df.columns else 'UNKNOWN'
    return df


# ---------------------------------------------------------------------------
# 1. Drag coefficient
# ---------------------------------------------------------------------------

def calculate_drag_coefficient(df, output_dir, mass_kg=0.5, area_m2=0.01):
    if 'vertical_speed' not in df.columns:
        print('Drag analysis skipped (no altitude to derive vertical speed)')
        return

    drop = df[df['_phase'] == 'DROP'].copy()
    if len(drop) < 10:
        print('Drag analysis skipped (fewer than 10 DROP phase rows)')
        return

    g = 9.81
    rho = 1.225  # kg/m³ at sea level

    # Smooth vertical speed (negative = falling)
    n = min(len(drop) - 1, 7) | 1
    drop['vspeed_s'] = spsig.savgol_filter(drop['vertical_speed'], n, 2)

    v_term = drop['vspeed_s'].min()        # most negative = terminal
    v_abs  = abs(v_term)

    print(f'Terminal velocity: {v_abs:.2f} m/s  ({v_abs * 3.6:.1f} km/h)')

    Cd = None
    if v_abs > 0.5:
        Cd = (2 * mass_kg * g) / (rho * v_abs**2 * area_m2)
        shape = ('streamlined' if Cd < 0.5 else
                 'moderately aerodynamic' if Cd < 1.0 else
                 'blunt/boxy' if Cd < 1.5 else 'very high drag / tumbling')
        print(f'Cd = {Cd:.3f}  ->  {shape}')
        drag_force = 0.5 * rho * v_abs**2 * Cd * area_m2
        print(f'Drag at Vt: {drag_force:.3f} N  |  Weight: {mass_kg * g:.3f} N')

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Drag coefficient analysis — mass {mass_kg} kg, A {area_m2} m2',
                 fontweight='bold')

    ax1.plot(drop['_t'], drop[lc.ALT_COL])
    ax1.set_ylabel('Altitude (m)'); ax1.set_title('Altitude during DROP')
    ax1.grid(True, alpha=0.3)

    ax2.plot(drop['_t'], drop['vertical_speed'], alpha=0.5, label='raw')
    ax2.plot(drop['_t'], drop['vspeed_s'], 'r-', linewidth=2, label='smoothed')
    ax2.axhline(v_term, color='green', linestyle='--',
                label=f'Vt = {v_abs:.2f} m/s')
    ax2.set_ylabel('Vertical speed (m/s)')
    ax2.set_title('Vertical speed (negative = falling)')
    ax2.legend(); ax2.grid(True, alpha=0.3)

    ax3.plot(drop[lc.ALT_COL], drop['vspeed_s'].abs(), 'bo-', linewidth=1.5)
    ax3.axhline(v_abs, color='green', linestyle='--', label='Terminal velocity')
    ax3.set_xlabel('Altitude (m)'); ax3.set_ylabel('Speed (m/s)')
    ax3.set_title('Speed approaching terminal velocity')
    ax3.legend(); ax3.grid(True, alpha=0.3)

    ax4.axis('off')
    if Cd is not None:
        info = (f'Cd = {Cd:.3f}\n\nTerminal velocity: {v_abs:.2f} m/s\n'
                f'Shape estimate: {shape}\n\n'
                'Reference Cd:\n  Sphere: 0.47\n  Cube: 1.05\n'
                '  Flat plate: 1.28\n  Streamlined: 0.04')
        ax4.text(0.1, 0.9, info, transform=ax4.transAxes,
                 verticalalignment='top', fontsize=11,
                 bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    out = os.path.join(output_dir, 'drag_coefficient.png')
    plt.savefig(out, dpi=150, bbox_inches='tight'); plt.close()
    print(f'Plot saved to {out}')


# ---------------------------------------------------------------------------
# 2. Parachute deployment detection
# ---------------------------------------------------------------------------

def analyze_parachute_deployment(df, output_dir):
    if 'vertical_speed' not in df.columns:
        print('Parachute analysis skipped (no altitude to derive vertical speed)')
        return

    # Vertical jerk (d²z/dt²) from smoothed speed
    n = min(len(df) - 1, 7) | 1
    vsmooth = spsig.savgol_filter(df['vertical_speed'].fillna(0), n, 2)
    accel_v = np.gradient(vsmooth, df['_t'].values)

    idx = int(np.argmax(accel_v))  # largest upward acceleration = chute opening
    t_dep = df['_t'].iloc[idx]
    alt_dep = df[lc.ALT_COL].iloc[idx] if lc.ALT_COL in df.columns else float('nan')
    max_decel = accel_v[idx]

    window = 3
    v_before = float(np.mean(vsmooth[max(0, idx - window):idx]))
    v_after  = float(np.mean(vsmooth[idx:min(len(vsmooth), idx + window + 1)]))
    reduction = abs(v_before) - abs(v_after)

    print(f'Deployment detected at t={t_dep:.2f}s  alt={alt_dep:.1f}m')
    print(f'Peak deceleration: {max_decel:.2f} m/s2  ({max_decel/9.81:.2f} g)')
    print(f'Speed before: {abs(v_before):.2f} m/s -> after: {abs(v_after):.2f} m/s'
          f'  ({-reduction:.2f} m/s)')

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(13, 10))
    fig.suptitle('Parachute deployment detection', fontweight='bold')

    if lc.ALT_COL in df.columns:
        ax1.plot(df['_t'], df[lc.ALT_COL])
        ax1.axvline(t_dep, color='red', linestyle='--', linewidth=2,
                    label=f'Deployment t={t_dep:.1f}s')
        ax1.scatter([t_dep], [alt_dep], color='red', s=200, zorder=10, marker='*')
        ax1.set_ylabel('Altitude (m)'); ax1.legend(); ax1.grid(True, alpha=0.3)

    ax2.plot(df['_t'], vsmooth, 'g-')
    ax2.axvline(t_dep, color='red', linestyle='--', linewidth=2)
    ax2.axhline(v_before, color='orange', linestyle=':', label=f'Before {abs(v_before):.1f} m/s')
    ax2.axhline(v_after,  color='blue',   linestyle=':', label=f'After  {abs(v_after):.1f} m/s')
    ax2.set_ylabel('Vertical speed (m/s)'); ax2.legend(); ax2.grid(True, alpha=0.3)

    ax3.plot(df['_t'], accel_v, 'r-')
    ax3.axvline(t_dep, color='red', linestyle='--', linewidth=2)
    ax3.scatter([t_dep], [max_decel], color='red', s=200, zorder=10, marker='*',
                label=f'Peak {max_decel:.1f} m/s²')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Vertical accel (m/s²)')
    ax3.legend(); ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(output_dir, 'parachute_deployment.png')
    plt.savefig(out, dpi=150, bbox_inches='tight'); plt.close()
    print(f'Plot saved to {out}')


# ---------------------------------------------------------------------------
# 3. Power consumption
# ---------------------------------------------------------------------------

def analyze_power_consumption(df, output_dir,
                               battery_capacity_mah=2000, nominal_voltage=7.4):
    if 'battery_voltage' not in df.columns:
        print('Power analysis skipped (no battery_voltage column)')
        return

    batt = df[['_t', 'battery_voltage', '_phase']].dropna()
    v0 = batt['battery_voltage'].iloc[0]
    v1 = batt['battery_voltage'].iloc[-1]
    drop = v0 - v1
    dur = batt['_t'].iloc[-1] - batt['_t'].iloc[0]

    pct_used = drop / nominal_voltage * 100
    mah_used = battery_capacity_mah * drop / nominal_voltage
    avg_mA   = mah_used / (dur / 3600) if dur > 0 else 0

    print(f'Battery: {v0:.2f}V -> {v1:.2f}V  (drop {drop:.3f}V over {dur:.1f}s)')
    print(f'Estimated consumed: {mah_used:.0f} mAh ({pct_used:.1f}%)  |  avg {avg_mA:.0f} mA')

    print('\nPer phase:')
    for phase in batt['_phase'].unique():
        p = batt[batt['_phase'] == phase]
        if len(p) < 2:
            continue
        pd_drop = p['battery_voltage'].iloc[0] - p['battery_voltage'].iloc[-1]
        pd_dur  = p['_t'].iloc[-1] - p['_t'].iloc[0]
        print(f'  {phase:15s}: {pd_drop:.3f}V in {pd_dur:.1f}s')

    # Estimated remaining pct over time (linear approx)
    pct_remaining = ((batt['battery_voltage'] - (nominal_voltage * 0.8)) /
                     (nominal_voltage * 0.2) * 100).clip(0, 100)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8))
    fig.suptitle('Battery / power consumption', fontweight='bold')

    ax1.plot(batt['_t'], batt['battery_voltage'], 'b-', linewidth=2)
    ax1.axhline(v0, color='green', linestyle='--', alpha=0.5, label=f'Start {v0:.2f}V')
    ax1.axhline(v1, color='red',   linestyle='--', alpha=0.5, label=f'End {v1:.2f}V')
    ax1.set_ylabel('Battery voltage (V)'); ax1.legend(); ax1.grid(True, alpha=0.3)

    ax2.plot(batt['_t'], pct_remaining, 'g-', linewidth=2)
    ax2.fill_between(batt['_t'], 0, pct_remaining, alpha=0.2, color='green')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Est. capacity remaining (%)')
    ax2.set_ylim(0, 105); ax2.grid(True, alpha=0.3)

    summary = (f'Drop: {drop:.2f}V\nUsed: ~{mah_used:.0f} mAh  ({pct_used:.1f}%)\n'
               f'Avg current: ~{avg_mA:.0f} mA')
    ax2.text(0.02, 0.97, summary, transform=ax2.transAxes,
             verticalalignment='top', fontsize=10,
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()
    out = os.path.join(output_dir, 'power_consumption.png')
    plt.savefig(out, dpi=150, bbox_inches='tight'); plt.close()
    print(f'Plot saved to {out}')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Physical feature analysis for DATA_xxx.CSV telemetry')
    parser.add_argument('input', help='Path to DATA_xxx.CSV')
    parser.add_argument('--output', '-o', default='data_out/advanced_features_output')
    parser.add_argument('--mass', type=float, default=0.5, help='CanSat mass in kg (default 0.5)')
    parser.add_argument('--area', type=float, default=0.01,
                        help='Cross-section area in m2 for Cd (default 0.01 = 10×10 cm)')
    lc.add_time_window_args(parser)
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f'Error: {args.input} not found'); sys.exit(1)

    os.makedirs(args.output, exist_ok=True)
    df = lc.filter_time_window(pd.read_csv(args.input), args.t_start, args.t_end)
    if df.empty:
        print(f'Error: no data in the requested time window '
              f'[{args.t_start}, {args.t_end}]s'); sys.exit(1)
    df = _prepare(df)

    print('=' * 70)
    print('ADVANCED FEATURE ANALYSIS')
    print(f'Input: {args.input}  |  mass={args.mass} kg  area={args.area} m2')
    if args.t_start is not None or args.t_end is not None:
        print(f'Time window: [{df["_t"].iloc[0]:.2f}s -> {df["_t"].iloc[-1]:.2f}s]'
              f'  ({len(df)} rows)')
    print('=' * 70)

    print('\n--- Drag coefficient ---')
    calculate_drag_coefficient(df, args.output, args.mass, args.area)

    print('\n--- Parachute deployment ---')
    analyze_parachute_deployment(df, args.output)

    print('\n--- Power consumption ---')
    analyze_power_consumption(df, args.output)

    print('\nDone.')


if __name__ == '__main__':
    main()
