"""
Analyze force_motor_log.csv and print a compact summary for pasting into chat.

Usage:
    uv run python analyze_force_motor_signals.py [path/to/force_motor_log.csv]
"""

import sys
from pathlib import Path

import numpy as np
from scipy.signal import correlate

IDLE_THRESHOLD    = 2.0   # force below this → idle
CONTACT_THRESHOLD = 5.0   # force above this → gripping

csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / "force_motor_log.csv"

data = np.genfromtxt(csv_path, delimiter=",", skip_header=1)
t, force, current_mA, load = data[:, 0], data[:, 1], data[:, 2], data[:, 3]

N = len(t)
duration = t[-1] - t[0]
dt = np.diff(t)
rate = 1.0 / np.median(dt)

idle    = force < IDLE_THRESHOLD
contact = force > CONTACT_THRESHOLD

def stats(arr, mask=None):
    x = arr[mask] if mask is not None else arr
    return np.mean(x), np.std(x), np.min(x), np.max(x)

def snr_db(signal, contact_mask, idle_mask):
    sig_var  = np.var(signal[contact_mask]) if contact_mask.sum() > 1 else np.nan
    idle_var = np.var(signal[idle_mask])    if idle_mask.sum()    > 1 else np.nan
    if idle_var == 0 or np.isnan(idle_var):
        return np.nan
    return 10 * np.log10(sig_var / idle_var)

def xcorr_lag_s(a, b, rate):
    a_n = (a - a.mean()) / (a.std() + 1e-9)
    b_n = (b - b.mean()) / (b.std() + 1e-9)
    cc = correlate(a_n, b_n, mode="full")
    lags = np.arange(-(len(a) - 1), len(a))
    peak_lag = lags[np.argmax(cc)]
    return peak_lag, peak_lag / rate

def pearson(a, b, mask=None):
    x, y = (a[mask], b[mask]) if mask is not None else (a, b)
    if len(x) < 2:
        return np.nan
    return np.corrcoef(x, y)[0, 1]

# Contact episode count and onset indices (rising edges of force threshold)
transitions = np.diff(contact.astype(int))
force_onset_indices = np.where(transitions == 1)[0] + 1  # sample where force first crosses threshold
n_contact_episodes = len(force_onset_indices)

# Per-episode onset timing: find when each motor signal first moves beyond its idle baseline
# Use baseline + 2*std as the motor contact threshold, computed from idle samples
def onset_lags_s(signal, force_onsets, idle_mask, t, lookahead=30):
    """
    For each force contact onset, search the preceding `lookahead` samples for when
    the motor signal first crosses idle_mean + 2*idle_std.
    Returns array of lag values in seconds (negative = motor leads force).
    """
    if idle_mask.sum() < 2:
        return np.array([])
    threshold = signal[idle_mask].mean() + 2 * signal[idle_mask].std()
    lags = []
    for onset_i in force_onsets:
        search_start = max(0, onset_i - lookahead)
        window = signal[search_start:onset_i + 1]
        crossings = np.where(window > threshold)[0]
        if len(crossings) == 0:
            # motor signal crossed threshold at or after force onset — find it after
            after = signal[onset_i:]
            after_cross = np.where(after > threshold)[0]
            if len(after_cross) == 0:
                continue
            motor_i = onset_i + after_cross[0]
        else:
            motor_i = search_start + crossings[0]
        lags.append(t[motor_i] - t[onset_i])
    return np.array(lags)

print(f"{'='*54}")
print(f"  SAMPLES: {N}   DURATION: {duration:.1f}s   RATE: {rate:.1f} Hz")
print(f"  IDLE samples: {idle.sum()} ({100*idle.mean():.0f}%)   "
      f"CONTACT samples: {contact.sum()} ({100*contact.mean():.0f}%)")
print(f"  Contact episodes: {n_contact_episodes}")
print(f"{'='*54}")

print(f"\nBASIC STATS (all samples)")
print(f"{'':20s} {'force':>9} {'curr_mA':>9} {'load':>9}")
for label, arr in [("min", np.array([force.min(), current_mA.min(), load.min()])),
                   ("max", np.array([force.max(), current_mA.max(), load.max()])),
                   ("mean", np.array([force.mean(), current_mA.mean(), load.mean()])),
                   ("std",  np.array([force.std(),  current_mA.std(),  load.std()]))]:
    print(f"  {label:<18s} {arr[0]:>9.1f} {arr[1]:>9.1f} {arr[2]:>9.1f}")

print(f"\nIDLE BASELINE (force < {IDLE_THRESHOLD})")
for label, arr in [("curr_mA", current_mA), ("load", load)]:
    m, s, _, _ = stats(arr, idle)
    print(f"  {label:<10s} mean ± std:  {m:.1f} ± {s:.1f}")

if contact.sum() > 1:
    print(f"\nGRIPPING (force > {CONTACT_THRESHOLD})")
    baseline_mA = current_mA[idle].mean() if idle.sum() > 0 else 0.0
    fm, fs, _, _ = stats(force,      contact)
    cm, cs, _, _ = stats(current_mA, contact)
    lm, ls, _, _ = stats(load,       contact)
    print(f"  force     mean ± std:  {fm:.1f} ± {fs:.1f}")
    print(f"  curr_mA   mean ± std:  {cm:.1f} ± {cs:.1f}  (baseline-sub: {cm-baseline_mA:.1f} ± {cs:.1f})")
    print(f"  load      mean ± std:  {lm:.1f} ± {ls:.1f}")

    print(f"\nCORRELATION (Pearson, gripping samples only)")
    print(f"  force vs curr_mA:  r = {pearson(force, current_mA, contact):.3f}")
    print(f"  force vs load:     r = {pearson(force, load,       contact):.3f}")

print(f"\nCORRELATION (Pearson, all samples)")
print(f"  force vs curr_mA:  r = {pearson(force, current_mA):.3f}")
print(f"  force vs load:     r = {pearson(force, load):.3f}")

lag_samp, lag_s = xcorr_lag_s(current_mA, force, rate)
print(f"\nCROSS-CORRELATION LAG (full signal, positive = motor leads force)")
print(f"  curr_mA peak lag:  {lag_samp:+d} samples ({lag_s:+.3f}s)")
lag_samp, lag_s = xcorr_lag_s(load, force, rate)
print(f"  load    peak lag:  {lag_samp:+d} samples ({lag_s:+.3f}s)")

print(f"\nSNR  gripping_variance / idle_variance  (dB)")
print(f"  force:    {snr_db(force,      contact, idle):.1f} dB")
print(f"  curr_mA:  {snr_db(current_mA, contact, idle):.1f} dB")
print(f"  load:     {snr_db(load,        contact, idle):.1f} dB")

print(f"\nCONTACT ONSET TIMING  (negative = motor signal moves first)")
print(f"  (lookahead window: 30 samples before each force threshold crossing)")
for label, signal in [("curr_mA", current_mA), ("load", load)]:
    lags = onset_lags_s(signal, force_onset_indices, idle, t)
    if len(lags) == 0:
        print(f"  {label:<10s} insufficient data")
    else:
        print(f"  {label:<10s} episodes analyzed: {len(lags)}")
        print(f"             lag median: {np.median(lags):+.3f}s   mean: {np.mean(lags):+.3f}s   "
              f"min: {np.min(lags):+.3f}s   max: {np.max(lags):+.3f}s")
print(f"{'='*54}")
