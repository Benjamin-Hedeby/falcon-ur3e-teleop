#!/usr/bin/env python3
"""Resample one recorded trial (record_trial.sh) to 500 Hz for admittance analysis.

    ros2 run falcon_ros2 analyze_trial.py <bag_dir> <csv> [--tool-frame tool0] [--out DIR]

Writes <name>_processed.csv and <name>_forces.png next to the CSV (or into --out), with <name> the
CSV's name. Columns of the processed CSV (vectors in base_link unless noted):
  t                         s, from the first sample
  clutch, limit_tripped     0/1, from the bridge's ~/clutch and ~/limit_tripped (held)
  x, y, z                   m, tool position from TF (base_link -> --tool-frame; default tool0,
                            the frame the bridge commands)
  vx..vz, ax..az            m/s, m/s^2: Savitzky-Golay derivatives of the position (25 samples,
                            order 3), never double differencing
  fx_raw..tz_raw            N, Nm: wrist wrench as recorded, in its own frame (TCP on the UR)
  fx..tz                    N, Nm: the same rotated into base_link
  target_x..target_z        m, commanded target from ~/target (rate/position modes; NaN in twist
                            mode or between clutch periods)
  f_mag                     N, |force|
  in_contact                0/1: f_mag above mean + 5 std of the first 1 s with the clutch held
"""
import argparse
import csv
import math
from pathlib import Path
import sys

import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy.signal import savgol_filter
from tf2_ros import Buffer

RATE = 500.0           # Hz, output grid
SG_WINDOW, SG_ORDER = 25, 3
MAX_GAP = 0.2          # s: no interpolation across longer gaps (target, wrench)
CONTACT_SIGMA = 5.0

WRENCH = '/force_torque_sensor_broadcaster/wrench'
TARGET = '/falcon_ur_teleop/target'
CLUTCH = '/falcon_ur_teleop/clutch'
TRIPPED = '/falcon_ur_teleop/limit_tripped'


def stamp_of(msg, t_recv):
    """Header stamp if there is a non-zero one, else the bag's receive time (s)."""
    h = getattr(msg, 'header', None)
    if h is not None and (h.stamp.sec or h.stamp.nanosec):
        return h.stamp.sec + h.stamp.nanosec * 1e-9
    return t_recv


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    data = {name: [] for name in types}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        data[topic].append((t_ns * 1e-9, deserialize_message(raw, types[topic])))
    return data


def rotate(q, v):
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def interp_gapped(t_grid, t, v):
    """Linear interpolation, NaN outside the data and across gaps longer than MAX_GAP."""
    out = np.full((len(t_grid), v.shape[1]), np.nan)
    if len(t) < 2:
        return out
    for k in range(v.shape[1]):
        out[:, k] = np.interp(t_grid, t, v[:, k], left=np.nan, right=np.nan)
    i = np.clip(np.searchsorted(t, t_grid), 1, len(t) - 1)
    out[(t[i] - t[i - 1]) > MAX_GAP] = np.nan
    return out


def hold(t_grid, t, v, default=0.0):
    """Zero-order hold (last value)."""
    out = np.full(len(t_grid), default)
    if len(t):
        i = np.searchsorted(t, t_grid, side='right') - 1
        ok = i >= 0
        out[ok] = np.asarray(v)[i[ok]]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('csv')
    ap.add_argument('--tool-frame', default='tool0')
    ap.add_argument('--out', default=None, help='output directory (default: next to the CSV)')
    args = ap.parse_args()
    csv_path = Path(args.csv)
    name = csv_path.stem
    out_dir = Path(args.out) if args.out else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    data = read_bag(args.bag)
    if WRENCH not in data or not data[WRENCH]:
        sys.exit(f'No {WRENCH} messages in {args.bag}')

    # TF: a buffer long enough to hold the whole trial
    t_all = [t for msgs in data.values() for t, _ in msgs]
    tf = Buffer(cache_time=Duration(seconds=max(60.0, max(t_all) - min(t_all) + 10.0)))
    for _, m in data.get('/tf_static', []):
        for tr in m.transforms:
            tf.set_transform_static(tr, 'bag')
    for _, m in data.get('/tf', []):
        for tr in m.transforms:
            tf.set_transform(tr, 'bag')

    def lookup(target, source, t):
        try:
            return tf.lookup_transform_core(target, source, Time(nanoseconds=int(t * 1e9)))
        except Exception:
            return None

    # Wrench samples: raw (own frame) and rotated into base_link at their own stamps
    w_t, w_raw, w_base = [], [], []
    for t_recv, m in data[WRENCH]:
        t = stamp_of(m, t_recv)
        f, tq = m.wrench.force, m.wrench.torque
        raw = (f.x, f.y, f.z, tq.x, tq.y, tq.z)
        tr = None
        for frame in (m.header.frame_id, args.tool_frame):  # like the bridge's fallback
            tr = lookup('base_link', frame, t)
            if tr is not None and all(math.isfinite(c) for c in (tr.transform.rotation.x, tr.transform.rotation.w)):
                break
            tr = None
        base = rotate(tr.transform.rotation, raw[:3]) + rotate(tr.transform.rotation, raw[3:]) \
            if tr is not None else (math.nan,) * 6
        w_t.append(t)
        w_raw.append(raw)
        w_base.append(base)
    w_t = np.array(w_t)
    order = np.argsort(w_t)
    w_t, w_raw, w_base = w_t[order], np.array(w_raw)[order], np.array(w_base)[order]

    t0, t1 = w_t[0], w_t[-1]
    grid = np.arange(t0, t1, 1.0 / RATE)

    pos = np.full((len(grid), 3), np.nan)
    for k, t in enumerate(grid):
        tr = lookup('base_link', args.tool_frame, t)
        if tr is not None:
            p = tr.transform.translation
            pos[k] = (p.x, p.y, p.z)
    valid = np.all(np.isfinite(pos), axis=1)
    if valid.sum() < SG_WINDOW:
        sys.exit(f'Too few TF poses for base_link -> {args.tool_frame} (frames in the bag: '
                 f'{sorted(tf.all_frames_as_string().split())[:6]}...)')
    # Fill short TF gaps so the filter runs over them; the filled samples stay marked NaN in x/y/z
    filled = pos.copy()
    for k in range(3):
        filled[:, k] = np.interp(grid, grid[valid], pos[valid, k])
    dt = 1.0 / RATE
    vel = savgol_filter(filled, SG_WINDOW, SG_ORDER, deriv=1, delta=dt, axis=0)
    acc = savgol_filter(filled, SG_WINDOW, SG_ORDER, deriv=2, delta=dt, axis=0)
    vel[~valid], acc[~valid] = np.nan, np.nan

    wr = interp_gapped(grid, w_t, w_raw)
    wb = interp_gapped(grid, w_t, w_base)

    tg_t, tg_v = [], []
    for t_recv, m in data.get(TARGET, []):
        tg_t.append(stamp_of(m, t_recv))
        p = m.pose.position
        tg_v.append((p.x, p.y, p.z))
    target = interp_gapped(grid, np.array(tg_t), np.array(tg_v).reshape(-1, 3))

    def bool_topic(topic):
        msgs = data.get(topic, [])
        return hold(grid, np.array([t for t, _ in msgs]), [float(m.data) for _, m in msgs])

    clutch = bool_topic(CLUTCH)
    tripped = bool_topic(TRIPPED)
    if not data.get(TRIPPED):  # fall back on the bridge CSV
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        if rows and 'limit_tripped' in rows[0] and 'ros_time' in rows[0]:
            tripped = hold(grid, np.array([float(r['ros_time']) for r in rows]),
                           [float(r['limit_tripped']) for r in rows])

    fmag = np.linalg.norm(wb[:, :3], axis=1)
    held = np.nonzero(clutch > 0.5)[0]
    start = held[0] if len(held) else 0
    base_win = slice(start, start + int(RATE))
    base_f = fmag[base_win][np.isfinite(fmag[base_win])]
    threshold = base_f.mean() + CONTACT_SIGMA * base_f.std() if len(base_f) > 10 else math.nan
    in_contact = (fmag > threshold).astype(int) if math.isfinite(threshold) else np.zeros(len(grid), int)

    cols = (['t', 'clutch', 'limit_tripped', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']
            + [c + '_raw' for c in ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')]
            + ['fx', 'fy', 'fz', 'tx', 'ty', 'tz', 'target_x', 'target_y', 'target_z', 'f_mag', 'in_contact'])
    table = np.column_stack([grid - t0, clutch, tripped, pos, vel, acc, wr, wb, target, fmag, in_contact])
    out_csv = out_dir / f'{name}_processed.csv'
    np.savetxt(out_csv, table, delimiter=',', header=','.join(cols), comments='', fmt='%.6g')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    depth = (pos[start, 2] - pos[:, 2]) * 1000 if np.isfinite(pos[start, 2]) else -pos[:, 2] * 1000
    fig, axs = plt.subplots(2, 3, figsize=(13, 7))
    c = np.where(in_contact > 0, 'tab:red', 'tab:blue')
    for i, n in enumerate('xyz'):
        axs[0, i].scatter(depth, wb[:, i], s=2, c=c)
        axs[0, i].set_xlabel('insertion depth (down from the first clutch press) [mm]')
        axs[0, i].set_ylabel(f'F{n} [N]')
        axs[1, i].scatter(vel[:, i] * 1000, wb[:, i], s=2, c=c)
        axs[1, i].set_xlabel(f'v{n} [mm/s]')
        axs[1, i].set_ylabel(f'F{n} [N]')
        for a in axs[:, i]:
            a.grid(alpha=0.3)
    fig.suptitle(f'{name}: force (base_link) vs depth and velocity; red = in contact '
                 f'(|F| > {threshold:.2f} N)')
    fig.tight_layout()
    out_png = out_dir / f'{name}_forces.png'
    fig.savefig(out_png, dpi=90)

    dur = grid[-1] - grid[0]
    peak = np.nanmax(fmag) if np.isfinite(fmag).any() else math.nan
    print(f'{name}: {dur:.1f} s, {len(grid)} samples at {RATE:.0f} Hz')
    print(f'  tool poses from TF: {valid.mean() * 100:.0f} %, wrench: {np.isfinite(wb[:, 0]).mean() * 100:.0f} %')
    print(f'  contact threshold {threshold:.2f} N, in contact {in_contact.mean() * 100:.1f} % of the time, '
          f'peak |F| {peak:.1f} N, limit tripped {int(np.sum(np.diff(tripped) > 0.5))} time(s)')
    print(f'  -> {out_csv}\n  -> {out_png}')


if __name__ == '__main__':
    main()
