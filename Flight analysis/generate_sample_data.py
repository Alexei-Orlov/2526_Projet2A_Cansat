"""Generate a synthetic LIDA_xxx.CSV for testing, until a real telemetry file is available.

Simulates a rough flight profile: GO_FOR_LAUNCH (on pad) -> ASCENSION (rising,
LIDAR blind) -> DROP (descending while tumbling, LIDAR pinging the ground)
-> RECOVERY (back on the ground). Column names follow lidar_common.py and
must be kept in sync if the real CSV uses different headers.
"""
import argparse
import os

import numpy as np
import pandas as pd

import lidar_common as lc

LAUNCH_LAT, LAUNCH_LON, LAUNCH_ALT = 45.1885, 5.7245, 220.0

# Cumulative bitmask written by the firmware (currentState >= STATE_X for each bit).
PHASE_FLAGS = {'GO_FOR_LAUNCH': 0x01, 'ASCENSION': 0x03, 'DROP': 0x07, 'RECOVERY': 0x0F}


def simulate(seed=0, hz=5):
    rng = np.random.default_rng(seed)

    phases = [
        ('GO_FOR_LAUNCH', 10),
        ('ASCENSION', 40),
        ('DROP', 90),
        ('RECOVERY', 10),
    ]
    rows = []
    t = 0.0
    apogee_alt = LAUNCH_ALT + 800.0
    lat, lon = LAUNCH_LAT, LAUNCH_LON

    drop_duration = phases[2][1]
    drop_elapsed = 0.0

    for phase, duration in phases:
        n = int(duration * hz)
        for _ in range(n):
            if phase == 'GO_FOR_LAUNCH':
                alt = LAUNCH_ALT + rng.normal(0, 0.05)
                roll, pitch, yaw = 0.0, 90.0, 0.0  # pad-stationary, nose up
                distance = 0.0  # sensor not yet reporting
            elif phase == 'ASCENSION':
                frac = t / duration if duration else 1.0
                alt = LAUNCH_ALT + frac * (apogee_alt - LAUNCH_ALT) + rng.normal(0, 1.0)
                roll, pitch, yaw = 0.0, 85.0, rng.uniform(0, 360)
                distance = 0.0  # LIDAR pointing forward/up, no ground return
            elif phase == 'DROP':
                drop_elapsed += 1.0 / hz
                frac = drop_elapsed / drop_duration
                alt = apogee_alt - frac * (apogee_alt - LAUNCH_ALT + 5.0)
                lat = LAUNCH_LAT + 0.0008 * frac + rng.normal(0, 0.000005)
                lon = LAUNCH_LON + 0.0005 * frac + rng.normal(0, 0.000005)
                roll = (rng.uniform(-1, 1) * 20) % 360
                pitch = -90.0 + rng.normal(0, 5)  # LIDAR boresight ~down
                yaw = (frac * 720 + rng.normal(0, 10)) % 360
                ground_alt = LAUNCH_ALT + 3.0 * np.sin(frac * 6)  # uneven terrain
                distance = max(alt - ground_alt + rng.normal(0, 0.3), 0.1)
            else:  # RECOVERY
                alt = LAUNCH_ALT + rng.normal(0, 0.2)
                roll, pitch, yaw = rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(0, 360)
                distance = 0.0

            accel_x = rng.normal(0, 0.5)
            accel_y = rng.normal(0, 0.5)
            accel_z = rng.normal(-9.81 if phase != 'ASCENSION' else -25.0, 1.0)

            rows.append({
                lc.TIMESTAMP_COL: round(t * 1000),
                lc.DISTANCE_COL: round(distance, 3),
                lc.ROLL_COL: round(roll, 2),
                lc.PITCH_COL: round(pitch, 2),
                lc.YAW_COL: round(yaw, 2),
                'accel_x': round(accel_x, 3),
                'accel_y': round(accel_y, 3),
                'accel_z': round(accel_z, 3),
                lc.LAT_COL: lat,
                lc.LON_COL: lon,
                lc.ALT_COL: round(alt, 2),
                lc.PHASE_COL: PHASE_FLAGS[phase],
            })
            t += 1.0 / hz

    return pd.DataFrame(rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate a synthetic LIDA_xxx.CSV sample file')
    parser.add_argument('--output', '-o', default='data_in/LIDA_sample.csv')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    df = simulate(seed=args.seed)
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    df.to_csv(args.output, index=False)
    print(f"Wrote {len(df)} rows to {args.output}")
