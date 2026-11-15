#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "pandas", "pyarrow", "matplotlib"]
# ///
"""
Try the self-steering (autosteer.py) on recorded runs, without the ramp.

  uv run steering_replay.py RUN_DIR ...
        --model FILE     use this steering_model.json instead of fitting one from the runs
        --save FILE      write the fitted model here (default: steering_model.json beside the runs)
        --sims N         simulated drives per starting position (default 4; 0 = skip the simulation)
        --set NAME=VALUE change a controller or estimator setting (see Params in autosteer.py)
        --skip RUN:FROM-TO  leave out part of a run, in seconds from its start (a stretch off the
                         sidewalk, say): --skip 2026-11-14_15-09-40:30-78

RUN_DIR is a folder written by export_run.py, with geometry.parquet from
sidewalk_geometry.py in it. Pictures are not needed, only the tables.

It does four things, and prints what it found for each:

  1. fit      how the ramp moves: speed per ERPM, and how fast the actuator steers
  2. check    predict 2 s ahead from the recorded taps alone, and compare with what the camera saw
  3. replay   run the estimator over each run, and compare the taps the controller
              wanted with the ones the driver made
  4. simulate let the controller drive the fitted model, with the noise and the
              dropouts of the real pictures, from starts on and off the centre line

and writes replay_<run>.png into each run folder and steering_sim.png beside the runs.
"""
import argparse
import json
import os
import sys

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from autosteer import DEG, Controller, Estimator, Params

MOVING_ERPM = 1500          # faster than this counts as driving
AHEAD_S = 2.0               # how far ahead the model check predicts
NEAR_S = 0.5                # a tap within this of the driver's counts as the same tap
HALF_WIDTH = 0.60           # of the sidewalk, m


# ---------- the recorded data ----------

class Run:
    def __init__(self, d, skip=()):
        self.dir = d = os.path.expanduser(d).rstrip("/")
        self.name = os.path.basename(d)
        c = pd.read_parquet(os.path.join(d, "control.parquet"))
        self.t0 = float(c.t.iloc[0])
        self.t = c.t.values - self.t0
        self.dt = float(np.median(np.diff(self.t)))
        self.erpm = c[[k for k in c.columns if k.startswith("vesc_erpm_")]].mean(axis=1).ffill().fillna(0.0).values
        self.steer = np.nan_to_num(c.steer.values)
        g = pd.read_parquet(os.path.join(d, "imu_gyro.parquet"))
        tg = g.t.values - self.t0
        csum = np.concatenate([[0.0], np.cumsum(g.y.values)])
        i1 = np.searchsorted(tg, self.t, side="right")
        i0 = np.searchsorted(tg, self.t - self.dt, side="right")
        self.gyro = np.where(i1 > i0, (csum[i1] - csum[i0]) / np.maximum(i1 - i0, 1), np.nan)   # mean over each tick
        self.gyro = pd.Series(self.gyro).ffill().fillna(0.0).values
        still = np.abs(self.erpm) < 20
        self.gyro_bias = float(np.median(self.gyro[still])) if still.sum() > 50 else 0.0
        self.turned = np.cumsum((self.gyro - self.gyro_bias) * self.dt)      # heading by gyro alone, from an arbitrary start
        gp = os.path.join(d, "geometry.parquet")
        self.vision = None
        if os.path.isfile(gp):
            v = pd.read_parquet(gp)
            v = v[v.edges > 0]
            self.vision = pd.DataFrame({"t": v.t.values - self.t0, "offset": v.offset_m.values,
                                        "heading": v.heading_rad.values, "edges": v.edges.values})
            self.vision_all_t = pd.read_parquet(gp).t.values - self.t0
        self.moving = np.abs(self.erpm) > MOVING_ERPM
        for a, b in skip:               # not driving on a sidewalk there: no pictures, and not counted as driving
            self.moving[(self.t >= a) & (self.t < b)] = False
            if self.vision is not None:
                self.vision = self.vision[(self.vision.t < a) | (self.vision.t >= b)].reset_index(drop=True)

    def drives(self, min_s=3.0):
        """Index ranges of continuous driving."""
        idx = np.flatnonzero(self.moving)
        if not len(idx):
            return []
        parts = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        return [(s[0], s[-1] + 1) for s in parts if len(s) * self.dt >= min_s]


# ---------- 1. fit ----------

def robust_slope(x, y):
    """y = k x, not thrown by a few wild points."""
    k = np.median(y[np.abs(x) > np.percentile(np.abs(x), 50)] / x[np.abs(x) > np.percentile(np.abs(x), 50)])
    for _ in range(20):
        r = y - k * x
        s = 1.4826 * np.median(np.abs(r)) + 1e-9
        w = np.minimum(1.0, 1.5 * s / np.maximum(np.abs(r), 1e-12))
        k = (w * x) @ y / ((w * x) @ x)
    return float(k), float(s)


def speed_pairs(run, L, ahead=AHEAD_S):
    """
    For moments a known time apart, both driving: how far the camera moved sideways
    beyond what the turning alone explains, against (ERPM x heading) added up in between.
    Their ratio is the speed per ERPM.
    """
    v = run.vision
    n = int(round(ahead / run.dt))
    xs, ys, sign = [], [], []
    ti = np.searchsorted(run.t, v.t.values).clip(0, len(run.t) - 1)
    # each picture's heading, steadied by its neighbours (carried to its own moment by the gyro)
    head = np.array([np.median((v.heading.values[a:b] - (run.turned[ti[a:b]] - run.turned[i])))
                     for i, a, b in zip(ti, np.searchsorted(v.t.values, v.t.values - 0.25),
                                        np.searchsorted(v.t.values, v.t.values + 0.25))])
    for k, i in enumerate(ti):
        j = np.searchsorted(v.t.values, v.t.values[k] + ahead)
        if j >= len(v) or abs(v.t.values[j] - v.t.values[k] - ahead) > 0.06:
            continue
        i1 = ti[j]
        if not run.moving[i:i1].all() or np.sign(run.erpm[i]) != np.sign(run.erpm[i1 - 1]):
            continue
        turned = run.turned[i:i1] - run.turned[i]
        xs.append(np.sum(run.erpm[i:i1] * (head[k] + turned)) * run.dt)
        ys.append(v.offset.values[j] - v.offset.values[k] - L * (run.turned[i1] - run.turned[i]))
        sign.append(np.sign(run.erpm[i]))
    return np.array(xs), np.array(ys), np.array(sign)


def steer_seen(run, p):
    """The steer angle the gyro shows while driving (NaN otherwise), smoothed over 0.2 s."""
    v = p.m_per_erpm * run.erpm
    raw = np.where(run.moving, p.wheelbase_m * (run.gyro - run.gyro_bias) / np.where(run.moving, v, 1.0), np.nan)
    return pd.Series(raw).rolling(11, center=True, min_periods=6).mean().values


def tapped(run, dead_s):
    """Seconds of tapping so far (right minus left), leaving out the first dead_s of every tap."""
    u = run.steer
    age = np.zeros(len(u))
    for i in range(len(u)):
        age[i] = (age[i - 1] + run.dt if i and u[i] == u[i - 1] else run.dt) if u[i] else 0.0
    return np.cumsum(np.where(age > dead_s, u, 0.0)) * run.dt


def fit(runs, p):
    print("1. fit")
    L = p.wheelbase_m
    seen = [r for r in runs if r.vision is not None]
    if seen:
        x, y, s = (np.concatenate(a) for a in zip(*(speed_pairs(r, L) for r in seen)))
        for name, m in (("forwards", s > 0), ("backwards", s < 0), ("both", s != 0)):
            if m.sum() < 30:
                continue
            k, sd = robust_slope(x[m], y[m])
            cruise = np.median(np.abs(np.concatenate([r.erpm[r.moving] for r in seen])))
            print(f"   speed {name:9s}: {k * 1e4:.2f}e-4 m/s per ERPM  = {k * cruise:.2f} m/s at cruise"
                  f"   ({m.sum()} pairs of pictures {AHEAD_S:.0f} s apart, typical miss {sd * 100:.1f} cm)")
            if name == "both":
                p.m_per_erpm = k
        wheel = 3.51 * 0.0254 * np.pi
        print(f"   with 3.51\" wheels that is {wheel / 60 / p.m_per_erpm:.1f} ERPM per wheel rpm (motor pole pairs x gearing)")
    else:
        print("   no geometry.parquet in these runs: keeping the speed per ERPM as given")

    best = None
    for dead in np.arange(0, 0.11, 0.02):
        num = den = tot = 0.0
        for r in runs:
            d, U = steer_seen(r, p), tapped(r, dead)
            for a, b in r.drives():
                a += int(0.5 / r.dt)                    # leave out pulling away
                m = np.isfinite(d[a:b])
                if m.sum() < 50:
                    continue
                uu, dd = U[a:b][m] - U[a:b][m].mean(), d[a:b][m] - d[a:b][m].mean()
                num, den, tot = num + uu @ dd, den + uu @ uu, tot + dd @ dd
        if den > 0:
            k = num / den
            left = tot - k * num
            if best is None or left < best[2]:
                best = (dead, k, left, 1 - left / tot)
    if best:
        p.tap_dead_s, p.steer_rate = float(best[0]), float(best[1])
        print(f"   actuator: {p.steer_rate / DEG:.1f} deg of steer per second of tapping, after the first {p.tap_dead_s * 1000:.0f} ms of a tap"
              f"   (explains {best[3] * 100:.0f}% of how the steer angle moves within a drive)")
    return p


# ---------- 2 and 3. replay ----------

def replay(run, p, record=True):
    """Run the estimator over a run as it happened. Returns per-tick arrays."""
    est, ctl = Estimator(p), Controller(p)
    n = len(run.t)
    out = {k: np.full(n, np.nan) for k in ("offset", "heading", "steer", "target", "want", "gyro_miss", "sd_offset")}
    vis_miss = []
    vt = run.vision.t.values if run.vision is not None else np.zeros(0)
    vi = 0
    for i in range(n):
        est.predict(run.dt, run.erpm[i], run.steer[i - 1] if i else 0.0)
        est.last.pop("gyro", None)
        est.gyro(run.gyro[i])
        out["gyro_miss"][i] = est.last.get("gyro", np.nan)
        while vi < len(vt) and vt[vi] <= run.t[i]:
            row = run.vision.iloc[vi]
            used = est.vision(row.offset, row.heading, int(row.edges))
            vis_miss.append((vi, i, *est.last["vision"][:2], used, int(row.edges)))
            vi += 1
        out["offset"][i], out["heading"][i], out["steer"][i] = est.x
        out["sd_offset"][i] = np.sqrt(est.P[0, 0])
        out["target"][i] = ctl.aim(est) if abs(est.v) >= p.min_speed else np.nan
        out["want"][i] = ctl.want(est)
    out["vision_miss"] = pd.DataFrame(vis_miss, columns=["row", "tick", "offset", "heading", "used", "edges"])
    return out


def predict_ahead(run, p, est_out, ahead=AHEAD_S):
    """From the estimate at one moment, roll the model forward on the recorded taps and ERPM only."""
    n = int(round(ahead / run.dt))
    L = p.wheelbase_m
    v = run.vision
    rows = []
    U = tapped(run, p.tap_dead_s)
    ti = np.searchsorted(run.t, v.t.values).clip(0, len(run.t) - 1)
    for k in range(0, len(v), 3):
        i = ti[k] - n
        if i < 0 or not run.moving[i:i + n].all() or not np.isfinite(est_out["steer"][i]):
            continue
        y, th, d0 = est_out["offset"][i], est_out["heading"][i], est_out["steer"][i]
        y0 = y
        for j in range(i, i + n):
            vv = p.m_per_erpm * run.erpm[j]
            d = d0 + p.steer_rate * (U[j] - U[i])
            y += vv * (th + d) * run.dt
            th += vv * d / L * run.dt
        rows.append((y - v.offset.values[k], th - v.heading.values[k], y0 - v.offset.values[k],
                     est_out["heading"][i] - v.heading.values[k], np.sign(run.erpm[i])))
    return np.array(rows).reshape(-1, 5)


def compare_taps(run, out):
    """The driver's taps against what the controller wanted at the time."""
    near = int(round(NEAR_S / run.dt))
    u, want = run.steer, np.nan_to_num(out["want"])
    starts = [i for i in range(len(u)) if u[i] and (i == 0 or u[i - 1] != u[i]) and run.moving[i]]
    same = other = none = 0
    for i in starts:
        w = want[max(0, i - near):i + near]
        if (w == u[i]).any():
            same += 1
        elif (w == -u[i]).any():
            other += 1
        else:
            none += 1
    # the other way round: stretches where the controller wanted a tap, and whether the driver made one
    wanted = alone = 0
    i = 0
    while i < len(want):
        if want[i] and run.moving[i]:
            j = i
            while j < len(want) and want[j] == want[i]:
                j += 1
            if (j - i) * run.dt >= 0.2:
                wanted += 1
                alone += not (u[max(0, i - near):j + near] == want[i]).any()
            i = j
        else:
            i += 1
    return {"taps": len(starts), "same": same, "opposite": other, "not wanted": none, "wanted": wanted, "wanted alone": alone}


def plot_replay(run, p, out, path):
    drives = run.drives(5.0)
    if not drives:
        return
    drives = drives[:8]
    fig, axes = plt.subplots(4, len(drives), figsize=(4.2 * len(drives), 8), squeeze=False, sharex="col")
    seen = steer_seen(run, p)
    for c, (a, b) in enumerate(drives):
        t = run.t[a:b]
        v = run.vision[(run.vision.t >= t[0]) & (run.vision.t <= t[-1])]
        ax = axes[0, c]
        ax.set_title(f"{'forwards' if run.erpm[a] > 0 else 'backwards'}, from {t[0]:.0f} s", fontsize=9)
        ax.scatter(v.t, v.offset * 100, s=5, c=np.where(v.edges == 2, "tab:blue", "tab:cyan"), label="camera")
        ax.plot(t, out["offset"][a:b] * 100, "k", lw=1, label="estimate")
        ax.axhline(0, c="gray", lw=0.5)
        ax.set_ylim(-60, 60)
        ax = axes[1, c]
        ax.scatter(v.t, v.heading / DEG, s=5, c=np.where(v.edges == 2, "tab:blue", "tab:cyan"))
        ax.plot(t, out["heading"][a:b] / DEG, "k", lw=1)
        ax.axhline(0, c="gray", lw=0.5)
        ax.set_ylim(-15, 15)
        ax = axes[2, c]
        ax.plot(t, seen[a:b] / DEG, c="tab:orange", lw=0.8, label="from the gyro")
        ax.plot(t, out["steer"][a:b] / DEG, "k", lw=1, label="estimate")
        ax.plot(t, out["target"][a:b] / DEG, c="tab:green", lw=1, label="controller's target")
        ax.set_ylim(-12, 12)
        ax = axes[3, c]
        ax.fill_between(t, 0, run.steer[a:b], step="post", color="tab:red", alpha=0.7, label="driver")
        ax.fill_between(t, 0, 0.6 * np.nan_to_num(out["want"][a:b]), step="post", color="tab:green", alpha=0.6, label="controller wants")
        ax.set_ylim(-1.2, 1.2)
        ax.set_xlabel("s")
    for r, name in enumerate(("offset, cm (+ right)", "heading, deg (+ clockwise)", "steer angle, deg (+ right)", "taps (+ right)")):
        axes[r, 0].set_ylabel(name)
        if axes[r, 0].get_legend_handles_labels()[0]:
            axes[r, 0].legend(fontsize=7, loc="upper left")
    fig.suptitle(f"{run.name}: the estimator and controller run over the recording (the driver steered, not the controller)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)


# ---------- 4. simulate ----------

def noise_bank(runs, outs, p):
    """Stretches of real measurement error, to be played back in the simulation: (vision rows, gyro per tick)."""
    vis, gyro = [], []
    for run, out in zip(runs, outs):
        if run.vision is None:
            continue
        miss = out["vision_miss"]
        all_t = run.vision_all_t
        for a, b in run.drives(5.0):
            a += int(1.0 / run.dt)
            g = out["gyro_miss"][a:b]
            if np.isfinite(g).all():
                gyro.append(g - g.mean())
            # every labelled picture in the drive: its error, or NaN where there was no result
            ts = all_t[(all_t >= run.t[a]) & (all_t < run.t[b - 1])]
            seq = np.full((len(ts), 3), np.nan)
            m = miss[(miss.tick >= a) & (miss.tick < b)]
            where = np.searchsorted(ts, run.vision.t.values[m.row.values]).clip(0, len(ts) - 1)
            seq[where] = np.c_[m.offset.values, m.heading.values, m.edges.values]
            if len(ts) > 20:
                vis.append(seq)
    return vis, gyro


def simulate(p, bank, rng, offset0, heading0, direction, seconds=18.0, dt=0.02, cruise=1.1,
             vision_every=5, vision_late=8, command_late=2, truth=None):
    """
    One drive with the controller in charge. The 'true' ramp follows the same kind of
    model as the estimator's, but with its own actuator speed and dead time, a wrong
    starting steer angle, and measurements carrying recorded errors and dropouts.
    """
    vis, gyro = bank
    truth = truth or {}
    rate = p.steer_rate * truth.get("rate", rng.uniform(0.6, 1.4))
    dead = truth.get("dead", rng.uniform(0.0, 0.08))
    L = p.wheelbase_m
    y, th, d = offset0, heading0, truth.get("steer0", rng.uniform(-4, 4) * DEG)
    bias = rng.normal(0, 0.01)
    vseq = vis[rng.integers(len(vis))] if vis else np.zeros((1, 3))
    gseq = gyro[rng.integers(len(gyro))] if gyro else np.zeros(1)
    vk, gk = rng.integers(len(vseq)), rng.integers(len(gseq))
    # set off where the camera can see the sidewalk (not in the middle of a driveway), as one would
    seen = np.isfinite(np.r_[vseq[:, 0], vseq[:15, 0]])
    good = [k for k in range(len(vseq)) if seen[k:k + 15].sum() >= 12]
    if good:
        vk = good[rng.integers(len(good))]
    est, ctl = Estimator(p), Controller(p)
    n = int(seconds / dt)
    log = np.zeros((n, 8))
    sent, past, tap_age, v, u = [0] * (command_late + 1), [], 0.0, 0.0, 0
    for i in range(n):
        t = i * dt
        v_want = direction * cruise if t >= 1.0 else 0.0            # a second standing, then off
        v += np.clip(v_want - v, -1.0 * dt, 1.0 * dt)
        applied = sent[0]
        tap_age = tap_age + dt if applied else 0.0
        if applied and tap_age > dead:
            d = float(np.clip(d + rate * applied * dt, -15 * DEG, 15 * DEG))
        w = v * np.tan(d) / L
        y += (v * np.sin(th) + L * np.cos(th) * w) * dt
        th += w * dt
        past.append((y, th))
        # what the ramp's computer sees
        est.predict(dt, v / p.m_per_erpm, u)
        est.gyro(w + bias + (gseq[(gk + i) % len(gseq)] if abs(v) > 0.3 else rng.normal(0, 0.003)))
        if i % vision_every == 0 and i >= vision_late:
            e = vseq[(vk + i // vision_every) % len(vseq)]
            if np.isfinite(e[0]):
                yy, tt = past[i - vision_late]
                est.vision(yy + e[0], tt + e[1], int(e[2]))
        u = ctl.step(est, dt)
        sent = sent[1:] + [u]
        log[i] = t, y, th, d, u, est.x[0], y - L * np.sin(th), v
    return log


def score(log, dt=0.02):
    t, y, th, d, u, est_y, y_rear, v = log.T
    go = t >= 1.0
    worst = np.maximum(np.abs(y), np.abs(y_rear))[go]
    late = t >= t[-1] - 6.0
    starts = np.flatnonzero((u != 0) & (np.roll(u, 1) != u))
    return {"worst": float(worst.max()), "end": float(np.sqrt(np.mean(y[late] ** 2))),
            "end_worst": float(np.maximum(np.abs(y), np.abs(y_rear))[late].max()),
            "taps_per_s": len(starts) / (go.sum() * dt), "tapping": float((u[go] != 0).mean()),
            "swing": float(np.ptp(y[late]))}


def run_sims(p, bank, sims, path):
    print("4. simulate: the controller drives the fitted model, for 18 s (about 19 m) from each start")
    rng = np.random.default_rng(0)
    starts = [(o, h) for o in (-0.30, -0.15, 0.0, 0.15, 0.30) for h in (-8, 0, 8)]
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex=True)
    rows = []
    for c, direction in enumerate((1, -1)):
        name = "forwards" if direction > 0 else "backwards"
        res = []
        for o, h in starts:
            for _ in range(sims):
                log = simulate(p, bank, rng, o, h * DEG, direction)
                s = score(log)
                s.update(offset0=o, heading0=h)
                res.append(s)
                axes[0, c].plot(log[:, 0], log[:, 1] * 100, lw=0.6, alpha=0.6, c=plt.cm.coolwarm((o + 0.3) / 0.6))
                axes[1, c].plot(log[:, 0], log[:, 3] / DEG, lw=0.5, alpha=0.5, c=plt.cm.coolwarm((o + 0.3) / 0.6))
        r = pd.DataFrame(res)
        rows.append(r.assign(direction=name))
        axes[0, c].set_title(f"{name}: the camera's offset from the centre line, cm")
        axes[0, c].axhline(0, c="gray", lw=0.5)
        for e in (-HALF_WIDTH * 100, HALF_WIDTH * 100):
            axes[0, c].axhline(e, c="k", lw=1, ls="--")
        axes[0, c].set_ylim(-75, 75)
        axes[1, c].set_title("steer angle, deg")
        axes[1, c].set_xlabel("s")
        print(f"   {name}: {len(r)} drives")
        print(f"      furthest either axle got from the centre line: typically {r.worst.median() * 100:.0f} cm, worst {r.worst.max() * 100:.0f} cm"
              f"   (the edge is at {HALF_WIDTH * 100:.0f} cm; the starts are up to 30 cm off)")
        print(f"      over the last 6 s: typically {r.end.median() * 100:.1f} cm off centre, worst drive {r.end.max() * 100:.1f} cm,"
              f" furthest {r.end_worst.max() * 100:.0f} cm, swinging {r.swing.median() * 100:.1f} cm side to side")
        print(f"      {r.taps_per_s.median():.1f} taps a second, tapping {r.tapping.median() * 100:.0f}% of the time")
        on = r[r.offset0 == 0]
        print(f"      starting on the centre line, pointed straight or 8 deg off: furthest {on.worst.max() * 100:.0f} cm")
    fig.suptitle("simulated drives, coloured by starting offset (blue = 30 cm left, red = 30 cm right); dashed = sidewalk edges", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)
    return pd.concat(rows)


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="exported run folders")
    ap.add_argument("--model")
    ap.add_argument("--save")
    ap.add_argument("--sims", type=int, default=4)
    ap.add_argument("--set", nargs="+", default=[], metavar="NAME=VALUE")
    ap.add_argument("--skip", nargs="+", default=[], metavar="RUN:FROM-TO")
    args = ap.parse_args()

    skip = {}
    for s in args.skip:
        name, _, span = s.rpartition(":")
        a, _, b = span.partition("-")
        skip.setdefault(name, []).append((float(a), float(b)))
    runs = [Run(d, skip.pop(os.path.basename(d.rstrip("/")), ())) for d in args.runs]
    if skip:
        sys.exit(f"--skip names a run that isn't here: {', '.join(skip)}")
    p = Params()
    if args.model:
        with open(args.model) as f:
            for k, v in json.load(f).items():
                setattr(p, k, v)
        print(f"1. fit: skipped, using {args.model}")
    else:
        p = fit(runs, p)
    for s in args.set:
        k, v = s.split("=")
        if not hasattr(p, k):
            sys.exit(f"no such setting: {k}")
        setattr(p, k, float(v))
    if not args.model:
        save = args.save or os.path.join(os.path.dirname(runs[0].dir), "steering_model.json")
        with open(save, "w") as f:
            json.dump(p.to_json(), f, indent=1)
        print(f"   written to {save}")

    outs = [replay(r, p) for r in runs]

    print(f"2. check: where the model says the ramp will be {AHEAD_S:.0f} s later, from the taps alone, against the camera")
    for r, out in zip(runs, outs):
        if r.vision is None:
            continue
        e = predict_ahead(r, p, out)
        for name, m in (("forwards", e[:, 4] > 0), ("backwards", e[:, 4] < 0)):
            if m.sum() < 10:
                continue
            a = np.abs(e[m])
            print(f"   {r.name} {name:9s}: offset out by {np.median(a[:, 0]) * 100:.1f} cm typically, {np.percentile(a[:, 0], 90) * 100:.1f} cm at worst tenth"
                  f" (assuming it stays put: {np.median(a[:, 2]) * 100:.1f} / {np.percentile(a[:, 2], 90) * 100:.1f});"
                  f" heading {np.median(a[:, 1]) / DEG:.1f} deg / {np.percentile(a[:, 1], 90) / DEG:.1f}"
                  f" (stays put: {np.median(a[:, 3]) / DEG:.1f} / {np.percentile(a[:, 3], 90) / DEG:.1f})   [{m.sum()} moments]")

    print(f"3. replay: the driver's taps while driving, against what the controller wanted within {NEAR_S} s")
    for r, out in zip(runs, outs):
        if r.vision is None:
            print(f"   {r.name}: no geometry.parquet, skipped")
            continue
        c = compare_taps(r, out)
        vm = out["vision_miss"]
        drv = vm[r.moving[vm.tick.values]]
        n = max(c["taps"], 1)
        print(f"   {r.name}: {c['taps']} taps: controller wanted the same {c['same'] * 100 // n}%, the opposite {c['opposite'] * 100 // n}%,"
              f" nothing {c['not wanted'] * 100 // n}%.  Controller wanted a tap {c['wanted']} times; the driver made none near {c['wanted alone']} of them.")
        if len(drv):
            ok = drv[drv.used]
            print(f"      pictures while driving: {len(drv)} with a result, {100 - len(ok) * 100 // len(drv)}% thrown out as not believable;"
                  f" the rest differ from the estimate by {ok.offset.abs().median() * 100:.1f} cm and {ok.heading.abs().median() / DEG:.1f} deg typically")
        m = r.moving & np.isfinite(out["offset"])
        print(f"      as driven: off centre by {np.median(np.abs(out['offset'][m])) * 100:.0f} cm typically, {np.percentile(np.abs(out['offset'][m]), 95) * 100:.0f} cm at most (95%)")
        path = os.path.join(r.dir, f"replay_{r.name}.png")
        plot_replay(r, p, out, path)
        print(f"      {path}")

    if args.sims:
        bank = noise_bank(runs, outs, p)
        if not bank[0]:
            sys.exit("no driving with pictures to take measurement errors from: cannot simulate")
        path = os.path.join(os.path.dirname(runs[0].dir), "steering_sim.png")
        run_sims(p, bank, args.sims, path)
        print(f"   {path}")


if __name__ == "__main__":
    main()
