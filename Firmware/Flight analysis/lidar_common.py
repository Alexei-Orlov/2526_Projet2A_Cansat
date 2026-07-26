"""Shared loading, cleaning and georeferencing logic for CANSAT LIDAR telemetry.

Column names below match the firmware's CSV header:
tx_timestamp_ms,distance,roll,pitch,yaw,accel_x,accel_y,accel_z,latitude,longitude,altitude,flags_raw
Newer firmware appends a gnss_alt column (GNSS-derived altitude); 'altitude'
remains the barometric one.
"""
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp

TIMESTAMP_COL = 'tx_timestamp_ms'
DISTANCE_COL = 'distance'
LAT_COL = 'latitude'
LON_COL = 'longitude'
ALT_COL = 'altitude'
GNSS_ALT_COL = 'gnss_alt'
BARO_ALT_BACKUP_COL = 'altitude_baro'
ALT_SOURCES = ('baro', 'gnss')
ROLL_COL = 'roll'
PITCH_COL = 'pitch'
YAW_COL = 'yaw'
QUAT_W_COL = 'quat_w'
QUAT_X_COL = 'quat_x'
QUAT_Y_COL = 'quat_y'
QUAT_Z_COL = 'quat_z'
PHASE_COL = 'flags_raw'

QUAT_COLS = (QUAT_W_COL, QUAT_X_COL, QUAT_Y_COL, QUAT_Z_COL)


def has_quaternion(df):
    return all(c in df.columns for c in QUAT_COLS)

EARTH_RADIUS_M = 6378137.0

# lidar_flags is cumulative: each state ORs in its own bit on top of every
# bit set by the states it has already passed through (currentState >=
# STATE_X), so the highest set bit identifies the current state.
#   READY (= GO_FOR_LAUNCH): 0x01
#   ASCENSION:               0x01 | 0x02 = 0x03
#   DROP:                    0x01 | 0x02 | 0x04 = 0x07
#   RECOVERY:                0x01 | 0x02 | 0x04 | 0x08 = 0x0F
PHASE_BITS = (
    (0x08, 'RECOVERY'),
    (0x04, 'DROP'),
    (0x02, 'ASCENSION'),
    (0x01, 'GO_FOR_LAUNCH'),
)


def decode_phase(flags_raw):
    flags = int(flags_raw)
    for bit, name in PHASE_BITS:
        if flags & bit:
            return name
    return 'UNKNOWN'

# Boresight directions in the CANSAT body frame, matching the IMU datasheet
# axes (x=forward, y=left, z=up, right-handed -- NOT the aerospace NED
# forward-right-down convention).
_SQRT2_2 = 0.7071067811865476
BORESIGHTS = {
    'forward': (1.0, 0.0, 0.0),
    'backward': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'up': (0.0, 0.0, 1.0),
    'down': (0.0, 0.0, -1.0),
    # Actual LW20/C mount: 45 deg off the body Y (left) axis, rotated 315 deg
    # about the body X (forward) axis per the IMU datasheet's right-hand
    # convention -- tilts the beam 45 deg left and 45 deg down.
    'lidar_mount': (0.0, _SQRT2_2, -_SQRT2_2),
}

# LightWare LW20/C datasheet (rated range, confirmed): 0.2 m to 100 m.
# The sensor's behavior on lost/no signal isn't documented publicly -- check
# empirically (point it at the sky or beyond 100 m) before trusting this as
# an error-code filter, it currently only enforces the rated measurement range.
DEFAULT_MIN_RANGE_M = 0.2
DEFAULT_MAX_RANGE_M = 100.0


def load_telemetry(path, required_columns):
    df = pd.read_csv(path)
    missing = [c for c in required_columns if c not in df.columns]
    if missing:
        raise ValueError(
            f"Missing expected column(s) {missing} in {path}. "
            f"Available columns: {list(df.columns)}"
        )
    return df


def select_altitude_source(df, source='baro'):
    """Pick which sensor feeds the 'altitude' column used by all downstream code.

    'baro' (default) leaves the DataFrame untouched -- identical to the
    historical behavior, and the only valid choice for older CSVs that have
    no gnss_alt column. 'gnss' overwrites 'altitude' with gnss_alt (keeping
    the original barometric values in 'altitude_baro' for comparison), so
    georeferencing, coloring and stats all transparently use GNSS altitude.
    """
    if source == 'baro':
        return df
    if source != 'gnss':
        raise ValueError(f"Unknown altitude source '{source}', choose from {ALT_SOURCES}")
    if GNSS_ALT_COL not in df.columns:
        raise ValueError(
            f"Altitude source 'gnss' requested but column '{GNSS_ALT_COL}' is missing "
            f"(older firmware CSV?). Available columns: {list(df.columns)}"
        )
    df = df.copy()
    df[BARO_ALT_BACKUP_COL] = df[ALT_COL]
    df[ALT_COL] = df[GNSS_ALT_COL]
    return df


def clean_telemetry(df, min_range=DEFAULT_MIN_RANGE_M, max_range=DEFAULT_MAX_RANGE_M):
    df_clean = df.dropna(subset=[DISTANCE_COL])
    if min_range is not None:
        df_clean = df_clean[df_clean[DISTANCE_COL] >= min_range]
    if max_range is not None:
        df_clean = df_clean[df_clean[DISTANCE_COL] <= max_range]
    if has_quaternion(df_clean):
        # The firmware logs all-zero quaternions until the BNO055 fusion has
        # started; they carry no orientation and crash Rotation.from_quat.
        quat_norm = np.linalg.norm(df_clean[list(QUAT_COLS)].to_numpy(), axis=1)
        df_clean = df_clean[quat_norm > 1e-6]
    return df_clean.reset_index(drop=True)


def _normalize_phase(value):
    return str(value).strip().upper().replace(' ', '_')


def filter_time_window(df, t_start=None, t_end=None, relative=False):
    """Keep rows whose timestamp (in seconds) lies within [t_start, t_end].

    Times are tx_timestamp_ms / 1000 — the same scale as the plot x-axes.
    With relative=True they are counted from the file's first row instead
    (useful when the firmware clock does not start at zero).
    Either bound may be None to leave that side open.
    """
    if t_start is None and t_end is None:
        return df
    t = df[TIMESTAMP_COL] / 1000.0
    if relative:
        t = t - t.iloc[0]
    mask = pd.Series(True, index=df.index)
    if t_start is not None:
        mask &= t >= t_start
    if t_end is not None:
        mask &= t <= t_end
    return df[mask].copy()


def add_time_window_args(parser, relative=False):
    """Attach the standard --t-start/--t-end options to an argparse parser."""
    ref = ('seconds since the first sample' if relative
           else 'seconds, same scale as the plot time axis (tx_timestamp_ms/1000)')
    parser.add_argument('--t-start', type=float, default=None,
                        help=f'Only analyze data from this time on ({ref})')
    parser.add_argument('--t-end', type=float, default=None,
                        help=f'Only analyze data up to this time ({ref})')


def filter_phases(df, phases):
    if not phases or PHASE_COL not in df.columns:
        return df
    wanted = {_normalize_phase(p) for p in phases}
    mask = df[PHASE_COL].map(decode_phase).isin(wanted)
    return df[mask].copy()


def interpolate_burst_orientations(df):
    """SLERP orientation within each firmware burst of identical-timestamp rows.

    The DMA pipeline captures one IMU snapshot per burst (typically 5 LIDAR
    hits sharing the same tx_timestamp_ms). Without interpolation the 3D cloud
    shows visible orientation "steps": 5 identical-orientation points, then a
    sudden jump to the next burst. This assigns each hit an estimated
    sub-timestamp and SLERPs between consecutive burst orientations.

    Uses quaternions when available; falls back to Euler-derived rotations.
    """
    if len(df) < 2:
        return df

    df = df.copy()
    ts = df[TIMESTAMP_COL].to_numpy()
    burst_ts_int, first_occ = np.unique(ts, return_index=True)
    burst_ts = burst_ts_int.astype(float)

    if len(burst_ts) < 2:
        return df

    burst_rows = df.iloc[first_occ]
    if has_quaternion(df):
        key_rots = Rotation.from_quat(
            burst_rows[[QUAT_X_COL, QUAT_Y_COL, QUAT_Z_COL, QUAT_W_COL]].to_numpy()
        )
    else:
        key_rots = Rotation.from_euler(
            'ZYX',
            burst_rows[[YAW_COL, PITCH_COL, ROLL_COL]].to_numpy(),
            degrees=True,
        )

    slerp = Slerp(burst_ts, key_rots)

    sub_idx = df.groupby(TIMESTAMP_COL, sort=False).cumcount().to_numpy()
    burst_size = df.groupby(TIMESTAMP_COL, sort=False)[TIMESTAMP_COL].transform('count').to_numpy()

    # dt to next burst; last burst reuses the previous interval to avoid extrapolation
    dt_next = np.empty(len(burst_ts))
    dt_next[:-1] = np.diff(burst_ts)
    dt_next[-1] = dt_next[-2]

    row_burst_idx = np.searchsorted(burst_ts, ts.astype(float))
    dt_per_row = dt_next[row_burst_idx]

    est_ts = ts.astype(float) + sub_idx * dt_per_row / burst_size
    est_ts = np.clip(est_ts, burst_ts[0], burst_ts[-1])

    interp_rots = slerp(est_ts)

    if has_quaternion(df):
        q = interp_rots.as_quat()  # (x, y, z, w) scalar-last
        df[QUAT_X_COL] = q[:, 0]
        df[QUAT_Y_COL] = q[:, 1]
        df[QUAT_Z_COL] = q[:, 2]
        df[QUAT_W_COL] = q[:, 3]
    else:
        e = interp_rots.as_euler('ZYX', degrees=True)
        df[YAW_COL] = e[:, 0]
        df[PITCH_COL] = e[:, 1]
        df[ROLL_COL] = e[:, 2]

    return df


def geodetic_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """Equirectangular local-tangent-plane approximation.

    Adequate for short-range CANSAT flights (a few km); not for long-range
    or high-latitude trajectories.
    """
    lat_rad = np.radians(lat)
    ref_lat_rad = np.radians(ref_lat)
    east = np.radians(lon - ref_lon) * np.cos(ref_lat_rad) * EARTH_RADIUS_M
    north = np.radians(lat - ref_lat) * EARTH_RADIUS_M
    up = alt - ref_alt
    return east, north, up


def compute_world_points(df, boresight='forward', ref=None, ignore_gps=False, ignore_altitude=False):
    """Translate+rotate each LIDAR hit into a common ENU world frame.

    Each row's hit point = CANSAT position (from lat/long/altitude, converted
    to local ENU meters) + the LIDAR ray (boresight rotated by the CANSAT's
    own roll/pitch/yaw at that timestamp, scaled by the measured distance).
    This compensates for the CANSAT moving/rotating between LIDAR samples,
    unlike treating the sensor as fixed at the origin.

    If ignore_gps is True, the horizontal (lat/long) component is dropped --
    useful when the GPS fix is noisy/jumpy -- and only the (barometer-derived)
    altitude is used for vertical translation, so the CANSAT is treated as
    staying above the same horizontal point while it descends.
    """
    if isinstance(boresight, str):
        if boresight not in BORESIGHTS:
            raise ValueError(f"Unknown boresight '{boresight}', choose from {list(BORESIGHTS)}")
        boresight = BORESIGHTS[boresight]

    if ref is None:
        ref_lat = df[LAT_COL].iloc[0]
        ref_lon = df[LON_COL].iloc[0]
        ref_alt = df[ALT_COL].iloc[0]
    else:
        ref_lat, ref_lon, ref_alt = ref

    n = len(df)
    if ignore_gps:
        east, north = np.zeros(n), np.zeros(n)
    else:
        east, north, _ = geodetic_to_enu(
            df[LAT_COL].to_numpy(), df[LON_COL].to_numpy(), df[ALT_COL].to_numpy(),
            ref_lat, ref_lon, ref_alt,
        )
    if ignore_altitude:
        up = np.zeros(n)
    else:
        up = df[ALT_COL].to_numpy() - ref_alt

    if has_quaternion(df):
        # Preferred: the BNO055's own fused quaternion, immune to the
        # Euler angles' large-tilt/gimbal-lock unreliability. CSV stores
        # w,x,y,z (Bosch order); scipy expects x,y,z,w (scalar-last).
        rotations = Rotation.from_quat(
            df[[QUAT_X_COL, QUAT_Y_COL, QUAT_Z_COL, QUAT_W_COL]].to_numpy()
        )
    else:
        # Fallback: Euler angles, assumed to follow the IMU's own
        # right-handed, Z-up convention (heading clockwise from North
        # about Z, then pitch about the rotated Y, then roll about the
        # rotated X). Unreliable at large pitch/roll (BNO055 Euler output
        # is only well-behaved for moderate tilt) -- prefer quaternion data.
        rotations = Rotation.from_euler(
            'ZYX',
            np.column_stack([df[YAW_COL], df[PITCH_COL], df[ROLL_COL]]),
            degrees=True,
        )
    ray_body = np.tile(boresight, (len(df), 1)) * df[DISTANCE_COL].to_numpy()[:, None]
    ray_world = rotations.apply(ray_body)
    # body (forward, left, up) aligns with world (North, West, Up) at zero attitude
    ray_north, ray_west, ray_up = ray_world[:, 0], ray_world[:, 1], ray_world[:, 2]
    ray_east = -ray_west

    out = df.copy()
    out['cansat_x'] = east
    out['cansat_y'] = north
    out['cansat_z'] = up
    out['x'] = east + ray_east
    out['y'] = north + ray_north
    out['z'] = up + ray_up
    return out, (ref_lat, ref_lon, ref_alt)
