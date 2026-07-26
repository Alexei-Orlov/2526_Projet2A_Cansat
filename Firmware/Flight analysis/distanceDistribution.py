import argparse

import plotly.express as px

import lidar_common as lc


def parse_args():
    parser = argparse.ArgumentParser(description='Plot the distribution of CANSAT LIDAR distances')
    parser.add_argument('--input', '-i', default='data_in/LIDA_001.CSV', help='Path to the LIDA_xxx.CSV telemetry file')
    parser.add_argument('--phases', nargs='*', default=None, help='Flight phase(s) to keep (e.g. drop). Defaults to all phases.')
    parser.add_argument('--min-range', type=float, default=lc.DEFAULT_MIN_RANGE_M, help='Discard distances below this value (LW20/C rated min range)')
    parser.add_argument('--max-range', type=float, default=lc.DEFAULT_MAX_RANGE_M, help='Discard distances above this value (LW20/C rated max range)')
    parser.add_argument('--nbins', type=int, default=50, help='Number of histogram bins')
    parser.add_argument('--output-html', default=None, help='Save the figure to this HTML file')
    parser.add_argument('--no-show', action='store_true', help='Do not open the interactive plot in a browser')
    return parser.parse_args()


def main():
    args = parse_args()

    df = lc.load_telemetry(args.input, required_columns=[lc.DISTANCE_COL])
    df_clean = lc.clean_telemetry(df, min_range=args.min_range, max_range=args.max_range)
    df_clean = lc.filter_phases(df_clean, args.phases)

    if df_clean.empty:
        raise SystemExit('No data left after cleaning/phase filtering, nothing to plot.')

    fig = px.histogram(
        df_clean,
        x=lc.DISTANCE_COL,
        nbins=args.nbins,
        title='Distribution of Euclidean Distances (CANSAT LIDAR)',
        labels={lc.DISTANCE_COL: 'Euclidean Distance', 'count': 'Number of Points'},
        color_discrete_sequence=['#00CC96'],
        marginal='box',
    )

    fig.update_layout(
        bargap=0.05,
        xaxis_title='Euclidean Distance',
        yaxis_title='Point Count',
        showlegend=False,
    )

    if args.output_html:
        fig.write_html(args.output_html)
        print(f"Figure saved to {args.output_html}")

    if not args.no_show:
        fig.show()


if __name__ == '__main__':
    main()
