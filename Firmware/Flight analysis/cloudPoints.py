import argparse

import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

import lidar_common as lc


def parse_args():
    parser = argparse.ArgumentParser(description='Build a trajectory-compensated 3D point cloud from CANSAT LIDAR telemetry')
    parser.add_argument('--input', '-i', default='data_in/LIDA_001.CSV', help='Path to the LIDA_xxx.CSV telemetry file')
    parser.add_argument('--phases', nargs='*', default=['drop'], help='Flight phase(s) to keep (e.g. drop). Pass nothing to keep all phases.')
    parser.add_argument('--min-range', type=float, default=lc.DEFAULT_MIN_RANGE_M, help='Discard distances below this value (LW20/C rated min range)')
    parser.add_argument('--max-range', type=float, default=lc.DEFAULT_MAX_RANGE_M, help='Discard distances above this value (LW20/C rated max range)')
    parser.add_argument('--boresight', default='lidar_mount', choices=list(lc.BORESIGHTS), help='Direction the LIDAR beam points in the body frame')
    parser.add_argument('--color-by', default='distance', help="Column to color the point cloud by: 'distance' (default), 'time' (relative seconds since first hit), or any CSV column name (e.g. altitude)")
    parser.add_argument('--no-trajectory', action='store_true', help='Do not draw the CANSAT trajectory on the plot')
    parser.add_argument('--alt-source', default='baro', choices=list(lc.ALT_SOURCES), help="Altitude source for the vertical translation: 'baro' (default, historical behavior) or 'gnss' (use the gnss_alt column, newer firmware only)")
    parser.add_argument('--ignore-gps', action='store_true', help='Ignore lat/long (e.g. noisy/jumpy fix), keep only altitude for vertical translation')
    parser.add_argument('--ignore-baro', action='store_true', help='Ignore the altitude (whatever --alt-source selects), keep only lat/long for horizontal translation (combine with --ignore-gps for a rotation-only cloud)')
    parser.add_argument('--output-html', default=None, help='Save the figure to this HTML file')
    parser.add_argument('--export-xyz', default=None, help='Export the point cloud to a plain text .xyz file')
    parser.add_argument('--no-slerp', action='store_true', help='Skip SLERP interpolation between bursts (raw IMU snapshots)')
    lc.add_time_window_args(parser, relative=True)
    parser.add_argument('--no-show', action='store_true', help='Do not open the interactive plot in a browser')
    return parser.parse_args()


def main():
    args = parse_args()

    required_columns = [
        lc.DISTANCE_COL, lc.LAT_COL, lc.LON_COL, lc.ALT_COL,
        lc.ROLL_COL, lc.PITCH_COL, lc.YAW_COL,
    ]
    df = lc.load_telemetry(args.input, required_columns)
    df = lc.select_altitude_source(df, args.alt_source)
    df = lc.filter_time_window(df, args.t_start, args.t_end, relative=True)
    df_clean = lc.clean_telemetry(df, min_range=args.min_range, max_range=args.max_range)
    df_clean = lc.filter_phases(df_clean, args.phases)

    if df_clean.empty:
        raise SystemExit('No data left after time-window/cleaning/phase filtering, nothing to plot.')

    if not args.no_slerp:
        df_clean = lc.interpolate_burst_orientations(df_clean)

    total_points = len(df_clean)
    df_world, ref = lc.compute_world_points(
        df_clean,
        boresight=args.boresight,
        ignore_gps=args.ignore_gps,
        ignore_altitude=args.ignore_baro,
    )
    df_world['_t_rel'] = (df_world[lc.TIMESTAMP_COL] - df_world[lc.TIMESTAMP_COL].iloc[0]) / 1000.0

    color_col = '_t_rel' if args.color_by == 'time' else args.color_by

    df_world['euclidean_quartile'] = pd.qcut(df_world[lc.DISTANCE_COL], q=4, duplicates='drop')

    print(f"\nReference point (lat, long, altitude): {ref}")
    print("\n--- Points per Quartile (Euclidean Distance) ---")
    quartile_stats = df_world['euclidean_quartile'].value_counts().sort_index()
    for interval, count in quartile_stats.items():
        percentage = (count / total_points) * 100
        print(f"Distance range {interval}: {count} points ({percentage:.2f}%)")

    print(f"\nTotal Valid Points: {total_points}")
    print(f"Max Distance Recorded: {df_world[lc.DISTANCE_COL].max():.2f}\n")

    if args.export_xyz:
        df_world[['x', 'y', 'z']].to_csv(args.export_xyz, sep=' ', header=False, index=False)
        print(f"Point cloud exported to {args.export_xyz}")

    fig = px.scatter_3d(
        df_world,
        x='x',
        y='y',
        z='z',
        color=color_col,
        color_continuous_scale='Viridis',
        title='CANSAT 3D Point Cloud (trajectory-compensated, ENU meters)',
        opacity=0.7,
    )
    fig.update_traces(marker=dict(size=3))

    if not args.no_trajectory:
        fig.add_trace(go.Scatter3d(
            x=df_world['cansat_x'],
            y=df_world['cansat_y'],
            z=df_world['cansat_z'],
            mode='lines+markers',
            marker=dict(size=2, color='red'),
            line=dict(color='red', width=3),
            name='CANSAT trajectory',
        ))

    if args.output_html:
        fig.write_html(args.output_html)
        print(f"Figure saved to {args.output_html}")

    if not args.no_show:
        fig.show()


if __name__ == '__main__':
    main()
