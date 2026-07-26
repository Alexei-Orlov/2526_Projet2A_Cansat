#!/usr/bin/env python3
"""Dashboard plots for DATA_xxx.CSV telemetry files.

Schema (firmware): tx_timestamp_ms, accel_x/y/z, gyro_x/y/z, roll, pitch, yaw,
                   temperature, altitude, latitude, longitude, satellites,
                   flags_raw, battery_voltage

Usage:
  python analyze_telemetry.py DATA_001.CSV
  python analyze_telemetry.py DATA_001.CSV --output ./report_folder
  python analyze_telemetry.py DATA_001.CSV --plots altitude,battery
  python analyze_telemetry.py DATA_001.CSV --plots 1,5
  python analyze_telemetry.py DATA_001.CSV --list-plots
  python analyze_telemetry.py DATA_001.CSV --t-start 120 --t-end 180
"""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Rectangle

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import lidar_common as lc

PHASE_COLORS = {
    'GO_FOR_LAUNCH': '#3498db',
    'ASCENSION':     '#2ecc71',
    'DROP':          '#e74c3c',
    'RECOVERY':      '#f39c12',
    'UNKNOWN':       '#95a5a6',
}

# key, columns, y-axis label
PLOT_DEFS = [
    ('accel',       ['accel_x', 'accel_y', 'accel_z'], 'Linear accel (m/s²)'),
    ('gyro',        ['gyro_x', 'gyro_y', 'gyro_z'],     'Angular rate (deg/s)'),
    ('orientation', ['roll', 'pitch', 'yaw'],           'Orientation (deg)'),
    ('temperature', ['temperature'],                    'Temperature (deg C)'),
    ('altitude',    [lc.ALT_COL],                       'Altitude (m)'),
    ('vspeed',      ['vertical_speed'],                 'Vertical speed (m/s) — derived'),
    ('latitude',    [lc.LAT_COL],                       'Latitude (deg)'),
    ('longitude',   [lc.LON_COL],                       'Longitude (deg)'),
    ('satellites',  ['satellites'],                     'GPS satellites'),
    ('battery',     ['battery_voltage'],                'Battery (V)'),
]


def _available_plots(df):
    return [(key, cols, lbl) for key, cols, lbl in PLOT_DEFS if any(c in df.columns for c in cols)]


def _select_plots(available, selection):
    """Filter `available` (key, cols, label) triples by a comma-separated
    selection of keys (e.g. 'altitude,battery') and/or 1-based indices
    (e.g. '1,5') referring to the printed --list-plots numbering."""
    if not selection:
        return available
    by_key = {key: (key, cols, lbl) for key, cols, lbl in available}
    chosen = []
    seen = set()
    for token in selection.split(','):
        token = token.strip()
        if not token:
            continue
        if token.isdigit():
            idx = int(token) - 1
            if idx < 0 or idx >= len(available):
                raise ValueError(f"Plot index out of range: {token} "
                                  f"(valid: 1-{len(available)})")
            match = available[idx]
        elif token in by_key:
            match = by_key[token]
        else:
            valid = ', '.join(k for k, _, _ in available)
            raise ValueError(f"Unknown plot key: '{token}' (valid keys: {valid})")
        if match[0] not in seen:
            seen.add(match[0])
            chosen.append(match)
    return chosen


def list_plots(csv_filename):
    df = _load(csv_filename)
    available = _available_plots(df)
    print('Available plots:')
    for i, (key, _, lbl) in enumerate(available, start=1):
        print(f'  {i}. {key:12s} - {lbl}')


def _load(csv_filename, t_start=None, t_end=None):
    df = lc.filter_time_window(pd.read_csv(csv_filename), t_start, t_end)
    if df.empty:
        raise ValueError('No data in the requested time window '
                         f'[{t_start}, {t_end}]s')
    return _prepare(df)


def _prepare(df):
    df = df.copy().reset_index(drop=True)
    df['_t'] = df[lc.TIMESTAMP_COL] / 1000.0
    df['_t_rel'] = df['_t'] - df['_t'].iloc[0]
    if lc.ALT_COL in df.columns:
        raw_vspeed = df[lc.ALT_COL].diff() / df['_t'].diff()
        df['vertical_speed'] = raw_vspeed.rolling(5, center=True, min_periods=1).mean()
    return df


def _phase_transitions(df):
    if lc.PHASE_COL not in df.columns:
        return []
    decoded = df[lc.PHASE_COL].map(lc.decode_phase)
    change_idx = decoded.index[decoded != decoded.shift()].tolist()
    transitions = []
    for i, idx in enumerate(change_idx):
        t0 = df.loc[idx, '_t']
        t1 = df.loc[change_idx[i + 1], '_t'] if i < len(change_idx) - 1 else df['_t'].iloc[-1]
        transitions.append((decoded[idx], t0, t1))
    return transitions


def _add_bands(ax, transitions):
    y0, y1 = ax.get_ylim()
    for phase, t0, t1 in transitions:
        rect = Rectangle((t0, y0), t1 - t0, y1 - y0,
                         facecolor=PHASE_COLORS.get(phase, '#cccccc'),
                         alpha=0.15, zorder=0)
        ax.add_patch(rect)
    ax.set_ylim(y0, y1)


def plot_telemetry_data(csv_filename, output_dir=None, selected_plots=None,
                        t_start=None, t_end=None):
    df = _load(csv_filename, t_start, t_end)
    transitions = _phase_transitions(df)
    title = os.path.splitext(os.path.basename(csv_filename))[0]

    print(f"Loaded {len(df)} rows  |  duration {df['_t_rel'].iloc[-1]:.1f}s")
    if transitions:
        for phase, t0, t1 in transitions:
            print(f"  {phase:15s}: {t0:.2f}s -> {t1:.2f}s")

    available = _available_plots(df)
    plots = [(cols, lbl) for _, cols, lbl in _select_plots(available, selected_plots)]

    if not plots:
        print('No plots to display (empty selection).')
        return

    ncols = 2 if len(plots) > 1 else 1
    nrows = (len(plots) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(16 if ncols > 1 else 8, nrows * 3 + 1),
                              squeeze=False)
    fig.suptitle(f'Telemetry — {title}', fontsize=14, fontweight='bold')
    axes_flat = list(axes.flat)

    for ax, (cols, ylabel) in zip(axes_flat, plots):
        for c in cols:
            if c in df.columns:
                sub = df[['_t', c]].dropna()
                ax.plot(sub['_t'], sub[c], linewidth=1.2, label=c,
                        marker='o', markersize=1.5)
        _add_bands(ax, transitions)
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=7)

    for ax in axes_flat[len(plots):]:
        ax.set_visible(False)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        out = os.path.join(output_dir, f'{title}_analysis.png')
        plt.savefig(out, dpi=150, bbox_inches='tight')
        print(f'Plot saved to {out}')
    else:
        plt.show()
    plt.close()


def generate_flight_report(csv_filename, output_file=None, t_start=None, t_end=None):
    df = _load(csv_filename, t_start, t_end)
    transitions = _phase_transitions(df)
    title = os.path.splitext(os.path.basename(csv_filename))[0]

    L = ['=' * 70, f'FLIGHT TELEMETRY REPORT: {title}', '=' * 70, '']
    if t_start is not None or t_end is not None:
        L += [f'Time window:   [{df["_t"].iloc[0]:.2f}s -> {df["_t"].iloc[-1]:.2f}s]', '']

    dur = df['_t_rel'].iloc[-1]
    L += ['OVERVIEW', '-' * 70,
          f'Rows:          {len(df)}',
          f'Duration:      {dur:.3f} s',
          f'Data rate:     {len(df) / max(dur, 1e-3):.1f} Hz', '']

    if lc.ALT_COL in df.columns:
        L += ['ALTITUDE', '-' * 70,
              f'Start:         {df[lc.ALT_COL].iloc[0]:.2f} m',
              f'Max:           {df[lc.ALT_COL].max():.2f} m',
              f'End:           {df[lc.ALT_COL].iloc[-1]:.2f} m', '']

    if 'vertical_speed' in df.columns:
        L += ['VERTICAL SPEED (derived)', '-' * 70,
              f'Max ascent:    {df["vertical_speed"].max():.2f} m/s',
              f'Max descent:   {df["vertical_speed"].min():.2f} m/s', '']

    if 'satellites' in df.columns:
        no_fix = (df['satellites'] == 0).sum()
        L += ['GPS', '-' * 70,
              f'Avg sats:      {df["satellites"].mean():.1f}',
              f'Min / Max:     {df["satellites"].min()} / {df["satellites"].max()}',
              f'No-fix rows:   {no_fix} ({100 * no_fix / len(df):.1f}%)', '']

    if 'battery_voltage' in df.columns:
        drop = df['battery_voltage'].iloc[0] - df['battery_voltage'].iloc[-1]
        L += ['BATTERY', '-' * 70,
              f'Start:         {df["battery_voltage"].iloc[0]:.3f} V',
              f'End:           {df["battery_voltage"].iloc[-1]:.3f} V',
              f'Drop:          {drop:.3f} V', '']

    if 'temperature' in df.columns:
        L += ['TEMPERATURE', '-' * 70,
              f'Min / Max / Avg: {df["temperature"].min():.1f} / '
              f'{df["temperature"].max():.1f} / {df["temperature"].mean():.1f} deg C', '']

    L += ['FLIGHT PHASES', '-' * 70]
    for phase, t0, t1 in transitions:
        n = ((df['_t'] >= t0) & (df['_t'] < t1)).sum()
        L.append(f'{phase:15s} : {t0:7.2f}s -> {t1:7.2f}s  ({t1 - t0:.2f}s, {n} rows)')
    L += ['', '=' * 70]

    text = '\n'.join(L)
    if output_file:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'Report saved to {output_file}')
    else:
        print(text)
    return text


def main():
    parser = argparse.ArgumentParser(
        description='Dashboard plots and report for DATA_xxx.CSV telemetry')
    parser.add_argument('input', help='Path to DATA_xxx.CSV')
    parser.add_argument('--output', '-o', default=None,
                        help='Output directory (default: show interactively)')
    parser.add_argument('--plots', '-p', default=None,
                        help="Comma-separated subset of plots to display, by key or "
                             "1-based index (see --list-plots), e.g. 'altitude,battery' "
                             "or '1,5'. Default: show all.")
    parser.add_argument('--list-plots', action='store_true',
                        help='List available plot keys/indices for this file and exit')
    lc.add_time_window_args(parser)
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f'Error: {args.input} not found')
        sys.exit(1)

    if args.list_plots:
        list_plots(args.input)
        return

    try:
        if args.output:
            os.makedirs(args.output, exist_ok=True)
            base = os.path.splitext(os.path.basename(args.input))[0]
            generate_flight_report(args.input,
                                    os.path.join(args.output, f'{base}_report.txt'),
                                    t_start=args.t_start, t_end=args.t_end)
        else:
            generate_flight_report(args.input, t_start=args.t_start, t_end=args.t_end)

        plot_telemetry_data(args.input, args.output, args.plots,
                            t_start=args.t_start, t_end=args.t_end)
    except ValueError as e:
        print(f'Error: {e}')
        sys.exit(1)
    print('Done.')


if __name__ == '__main__':
    main()
