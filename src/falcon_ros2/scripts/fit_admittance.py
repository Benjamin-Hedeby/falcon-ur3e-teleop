#!/usr/bin/env python3
"""Fit admittance parameters (M, D, K per axis) to recorded contact trials.

    ros2 run falcon_ros2 fit_admittance.py <trial>_processed.csv [more ...] [--axes x y]

Input: the 500 Hz files from analyze_trial.py. Idea: in trials where the operator guides the peg
by force feedback alone (no admittance Insert), the operator acts as the compliant controller.
Their response to contact is fitted with the admittance model

    F(t - tau) = M a(t) + D v(t) + K x(t) + c_segment

per axis (base_link), over every contact segment (clutch held, in contact, limit not tripped):
F is the contact force on the tool, a/v the tool acceleration/velocity, x its position (K acts
relative to a per-segment reference, absorbed in c_segment), tau the operator's reaction delay.

Two stages. A linear least-squares fit of that equation gives a first estimate, but it needs the
acceleration, whose noise (from differentiating position twice) biases M towards zero, which a
longer tau then absorbs. So the estimate is refined by an output-error fit: the model is simulated
from the measured force and M, D, K, c are adjusted until the simulated positions match the
measured ones (no second derivative). tau is searched on a grid; the best position fit wins.

D and K are well determined. M is not: a mass and a reaction delay both just delay the response,
so several (M, tau) pairs fit almost equally well (tested on synthetic data: D and K within 1-3 %,
M anywhere from a few kg to the true value). The script reports that range, and suggests M from
D and a response time M/D (RESPONSE_TIME, the current bridge default) instead of the fitted M.

What responds to the force is the operator's COMMANDED motion (--signal command, the default when
the trial has cmd_v* columns from /twist_controller/twist): v is the commanded velocity, x its
integral, a its smoothed derivative. The tool's measured motion (--signal motion) is the wrong
signal in contact: the socket's geometry constrains it, so a fit to it describes the socket, not
the operator (on real data it gave M of thousands of kg and K of ~5000 N/m).

Use the sideways axes (X, Y): there the operator intends no motion, so motion during contact is
reaction to the force. Along Z the commanded push dominates; its fit is reported but should not be
trusted (use F/v while pushing instead).

Prints the fit per axis and the bridge parameters (adm_mass, adm_damping, adm_stiffness), and saves
a fit plot next to the first input.
"""
import argparse
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
from scipy.signal import StateSpace, lsim

MIN_SEGMENT = 0.3   # s, shorter contact segments are skipped
MAX_DELAY = 0.5     # s, reaction delays tried: 0 .. MAX_DELAY
DELAY_STEP = 0.01   # s
AXES = {'x': 0, 'y': 1, 'z': 2}


def load(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    need = ('t', 'clutch', 'limit_tripped', 'x', 'vx', 'ax', 'fx', 'in_contact')
    missing = [n for n in need if n not in d.dtype.names]
    if missing:
        sys.exit(f'{path}: missing columns {missing} (expected analyze_trial.py output)')
    return d


def use_command(d, rate):
    """Replace x, v*, a* by the commanded motion: v = cmd_v, x = its integral, a = its derivative."""
    for n in 'xyz':
        v = d['cmd_v' + n].copy()
        filled = np.nan_to_num(v)
        d[n] = np.cumsum(filled) / rate
        a = savgol_filter(filled, 25, 3, deriv=1, delta=1.0 / rate)
        a[~np.isfinite(v)] = np.nan
        d['v' + n], d['a' + n] = v, a


def segments(d, rate):
    """(start, stop) index pairs of contact segments long enough to fit."""
    ok = (d['clutch'] > 0.5) & (d['in_contact'] > 0.5) & (d['limit_tripped'] < 0.5)
    for n in 'xyz':
        ok &= np.isfinite(d[n]) & np.isfinite(d['v' + n]) & np.isfinite(d['a' + n]) & np.isfinite(d['f' + n])
    edges = np.flatnonzero(np.diff(np.r_[0, ok.astype(int), 0]))
    return [(a, b) for a, b in zip(edges[::2], edges[1::2]) if (b - a) / rate >= MIN_SEGMENT]


def design(trials, axis, delay):
    """Stack rows [a, v, x, one-hot segment] and the delayed force for all segments."""
    rows, rhs, seg_ids, n_seg = [], [], [], 0
    for d, rate, segs in trials:
        lag = int(round(delay * rate))
        for a, b in segs:
            a = max(a, lag)  # force sample at t - delay must exist
            if b - a < MIN_SEGMENT * rate:
                continue
            i = np.arange(a, b)
            rows.append(np.c_[d['a' + axis][i], d['v' + axis][i], d[axis][i]])
            rhs.append(d['f' + axis][i - lag])
            seg_ids.append(np.full(len(i), n_seg))
            n_seg += 1
    if not rows:
        return None
    X = np.vstack(rows)
    ids = np.concatenate(seg_ids)
    X = np.c_[X, np.eye(n_seg)[ids]]  # per-segment intercepts
    return X, np.concatenate(rhs), n_seg


SIM_RATE = 100.0    # Hz, the output-error fit simulates at this rate (motions are much slower)
DELAY_SPAN = 0.10   # s, stage 2 searches the delay within this of the stage-1 value
NEAR_BEST = 1.05    # fits within 5 % of the best position RMSE count as "equally good"
RESPONSE_TIME = 0.04  # s, M/D used for the suggested M (bridge default: 40 / 1000)


def segment_data(trials, axis, delay):
    """Per segment, downsampled to SIM_RATE: time from its start, position relative to its
    start, start velocity, and the force delayed by `delay`."""
    out = []
    for d, rate, segs in trials:
        lag = int(round(delay * rate))
        step = max(1, int(round(rate / SIM_RATE)))
        for a, b in segs:
            a = max(a, lag)
            if b - a < MIN_SEGMENT * rate:
                continue
            i = np.arange(a, b, step)
            # Exact uniform time from the sample index (the CSV's rounded times aren't)
            out.append((np.arange(len(i)) * step / rate, d[axis][i] - d[axis][a], d['v' + axis][a],
                        d['f' + axis][i - lag]))
    return out


def responses(seg, M, D, K):
    """Position responses of M x'' + D x' + K x = u from the segment start, in one simulation:
    to u = F (zero start), to u = 1 (zero start), and to u = 0 from the measured start velocity.
    The model response for a force offset c is then x_F - c x_1 + x_v."""
    t, _, v0, f = seg
    A1 = np.array([[0.0, 1.0], [-K / M, -D / M]])
    A = np.kron(np.eye(3), A1)
    B = np.zeros((6, 2))
    B[1, 0] = B[3, 1] = 1.0 / M
    C = np.zeros((3, 6))
    C[0, 0] = C[1, 2] = C[2, 4] = 1.0
    _, y, _ = lsim(StateSpace(A, B, C, np.zeros((3, 2))), np.c_[f, np.ones_like(f)], t,
                   X0=[0, 0, 0, 0, 0, v0])
    return y[:, 0], y[:, 1], y[:, 2]


def best_offset(seg, xF, x1, xv):
    """Least-squares force offset c for one segment (the response is linear in c)."""
    r = xF + xv - seg[1]
    den = float(x1 @ x1)
    return float(r @ x1) / den if den > 0 else 0.0


def simulate(seg, M, D, K, c):
    xF, x1, xv = responses(seg, M, D, K)
    return xF - c * x1 + xv


def output_error_fit(segs, guess):
    """Refine (M, D, K), with each segment's offset solved exactly, so the simulated positions
    match the measured ones."""
    def residuals(p):
        out = []
        for s in segs:
            xF, x1, xv = responses(s, *p)
            out.append(xF - best_offset(s, xF, x1, xv) * x1 + xv - s[1])
        return np.concatenate(out)

    p0 = np.maximum(np.asarray(guess, dtype=float), [0.5, 1.0, 0.0])
    r = least_squares(residuals, p0, bounds=([0.1, 0.0, 0.0], np.inf), x_scale='jac',
                      max_nfev=100)
    pos = np.concatenate([s[1] for s in segs])
    rmse = np.sqrt(np.mean(r.fun ** 2))
    r2 = 1.0 - np.mean(r.fun ** 2) / pos.var() if pos.var() > 0 else 0.0
    offsets = [best_offset(s, *responses(s, *r.x)) for s in segs]
    return r.x, offsets, rmse, r2


def fit_axis(trials, axis):
    # Stage 1: equation-error least squares over the whole delay grid (first estimate)
    first = None
    for delay in np.arange(0.0, MAX_DELAY + 1e-9, DELAY_STEP):
        dm = design(trials, axis, delay)
        if dm is None:
            return None
        X, y, n_seg = dm
        coef, *_ = np.linalg.lstsq(X, y, rcond=None)
        res = y - X @ coef
        r2 = 1.0 - res.var() / y.var() if y.var() > 0 else 0.0
        if first is None or r2 > first['r2']:
            first = dict(delay=delay, coef=coef, r2=r2, n=len(y), segments=n_seg)
    # Stage 2: output-error fit near the stage-1 delay, started from the stage-1 values
    best, tried = None, []
    lo = max(0.0, first['delay'] - DELAY_SPAN)
    for delay in np.arange(lo, min(MAX_DELAY, first['delay'] + DELAY_SPAN) + 1e-9, DELAY_STEP):
        segs = segment_data(trials, axis, delay)
        guess = (best['M'], best['D'], best['K']) if best else first['coef'][:3]
        p, offsets, rmse, r2 = output_error_fit(segs, guess)
        tried.append((rmse, delay, p[0]))
        if best is None or rmse < best['rmse']:
            best = dict(delay=delay, M=p[0], D=p[1], K=p[2], c=offsets, rmse=rmse, r2=r2,
                        n=first['n'], segments=len(segs), segs=segs,
                        first=dict(M=first['coef'][0], D=first['coef'][1], K=first['coef'][2],
                                   delay=first['delay']))
    near = [(dl, m) for r, dl, m in tried if r <= best['rmse'] * NEAR_BEST]
    best['M_range'] = (min(m for _, m in near), max(m for _, m in near))
    best['delay_range'] = (min(dl for dl, _ in near), max(dl for dl, _ in near))
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('processed', nargs='+')
    ap.add_argument('--axes', nargs='+', default=['x', 'y', 'z'], choices=list(AXES))
    ap.add_argument('--max-delay', type=float, default=MAX_DELAY,
                    help=f'longest reaction delay searched, s (default {MAX_DELAY})')
    ap.add_argument('--signal', choices=['command', 'motion'], default=None,
                    help='motion the force is fitted to (default: command if recorded)')
    args = ap.parse_args()
    globals()['MAX_DELAY'] = args.max_delay  # read by fit_axis()

    trials = []
    for p in args.processed:
        d = load(p)
        rate = 1.0 / np.median(np.diff(d['t']))
        has_cmd = 'cmd_vx' in d.dtype.names and np.isfinite(d['cmd_vx']).any()
        signal = args.signal or ('command' if has_cmd else 'motion')
        if signal == 'command':
            if not has_cmd:
                sys.exit(f'{p}: no commanded velocity (cmd_v*): re-run analyze_trial.py, or use '
                         '--signal motion')
            use_command(d, rate)
        segs = segments(d, rate)
        what = 'commanded' if signal == 'command' else 'measured'
        print(f'{Path(p).name}: {len(segs)} contact segment(s), '
              f'{sum(b - a for a, b in segs) / rate:.1f} s in contact (fitting the {what} motion)')
        trials.append((d, rate, segs))
    if not any(s for _, _, s in trials):
        sys.exit('No contact segments to fit (clutch held, in contact, limit not tripped, >= '
                 f'{MIN_SEGMENT} s)')

    fits = {}
    print(f"\n{'axis':4} {'M [kg]':>9} {'D [Ns/m]':>10} {'K [N/m]':>9} {'delay [s]':>9} "
          f"{'pos R2':>7} {'pos RMSE':>9} {'segments':>8}")
    for ax in args.axes:
        f = fit_axis(trials, ax)
        if f is None:
            print(f'{ax:4} (no data)')
            continue
        fits[ax] = f
        print(f"{ax:4} {f['M']:9.1f} {f['D']:10.1f} {f['K']:9.1f} {f['delay']:9.2f} "
              f"{f['r2']:7.2f} {f['rmse'] * 1000:7.2f}mm {f['segments']:8d}"
              + ('   <- commanded push dominates: unreliable' if ax == 'z' else ''))
        print(f"     equally good fits (RMSE within {(NEAR_BEST - 1) * 100:.0f} %): "
              f"M {f['M_range'][0]:.1f}..{f['M_range'][1]:.1f} kg with delay "
              f"{f['delay_range'][0]:.2f}..{f['delay_range'][1]:.2f} s")

    lateral = [fits[a] for a in ('x', 'y') if a in fits]
    if lateral:
        bad = [a for a in ('x', 'y') if a in fits and fits[a]['r2'] < 0.5]
        if bad:
            print(f'\nPoor fit on {bad} (position R2 < 0.5): the motion there is not an admittance-like '
                  'response (too little contact data, or the operator steered actively). '
                  'Record more trials.')
        print('\nSuggested bridge parameters from the sideways axes (Z kept as it is: set its D from the '
              'push force per speed you want, F = D_z v):')
        dd = [max(f['D'], 0.0) for f in lateral]
        m = [RESPONSE_TIME * d for d in dd]  # M from D: the fitted M is not well determined
        k = [max(f['K'], 0.0) for f in lateral]
        print(f'  ros2 param set /falcon_ur_teleop adm_mass "[{m[0]:.1f}, {m[-1]:.1f}, <z>]"'
              f'   # M = {RESPONSE_TIME * 1000:.0f} ms x D, not fitted')
        print(f'  ros2 param set /falcon_ur_teleop adm_damping "[{dd[0]:.1f}, {dd[-1]:.1f}, <z>]"')
        print(f'  ros2 param set /falcon_ur_teleop adm_stiffness "[{k[0]:.1f}, {k[-1]:.1f}, <z>]"')
        print('  Check M/D (response time) is well above the loop delay, and try them in the MuJoCo '
              'simulation before the robot.')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(fits), 1, figsize=(11, 3 * len(fits)), squeeze=False)
    for row, (ax, f) in zip(axs[:, 0], fits.items()):
        meas = np.concatenate([s[1] for s in f['segs']]) * 1000
        pred = np.concatenate([simulate(s, f['M'], f['D'], f['K'], c) for s, c in
                               zip(f['segs'], f['c'])]) * 1000
        row.plot(meas, lw=0.8, label=f'measured {ax} (from each segment start)')
        row.plot(pred, lw=0.8, label=f'admittance fit: M={f["M"]:.1f} D={f["D"]:.0f} K={f["K"]:.0f}, '
                                     f'delay {f["delay"]:.2f} s, R2={f["r2"]:.2f}')
        row.set_ylabel('mm')
        row.legend(loc='upper right', fontsize=8)
        row.grid(alpha=0.3)
    axs[-1, 0].set_xlabel('contact samples (all segments, concatenated)')
    fig.tight_layout()
    out = Path(args.processed[0]).with_name('admittance_fit.png')
    fig.savefig(out, dpi=90)
    print(f'\nplot: {out}')


if __name__ == '__main__':
    main()
