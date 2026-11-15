#!/usr/bin/env python3
"""
Read the telemetry (everything except the pictures) out of recorded runs.

  .venv/bin/python extract_telemetry.py                 summarise every run in ~/ramp-runs
  .venv/bin/python extract_telemetry.py RUN.rrd ...     summarise these runs

A bare file name means one in ~/ramp-runs. The summary is a quick health check
of a run: how long it drove, how much steering it holds, how often the
controller dropped out.

From Python:

    from extract_telemetry import load
    run = load("~/ramp-runs/2026-11-11_16-58-31.rrd")
    t, gyro = run.scalars["/imu/gyro"]      # t: int64 ns since the epoch, gyro: (n, 3)

Needs rerun-sdk (the same version camera.py records with) and numpy.
export_run.py uses this to write the training export.
"""
import glob
import json
import os
import sys
from dataclasses import dataclass, field

import numpy as np
import pyarrow as pa
import rerun.chunk as rrc

RUN_DIR = os.path.expanduser("~/ramp-runs")
CAMERAS = ("color", "depth", "ir", "rear")
CAMERA_PATHS = [f"/camera/{c}" for c in CAMERAS]


@dataclass
class Run:
    path: str
    name: str                                       # file name without .rrd
    scalars: dict = field(default_factory=dict)     # entity path -> (t_ns int64 (n,), values float64 (n, k))
    texts: dict = field(default_factory=dict)       # entity path -> [(t_ns, text), ...]
    calibration: dict | None = None                 # what camera.py read from the RealSense
    motors: tuple = ("back", "front")               # column order of the per-motor values


def find_run(arg):
    """A path, or a bare file name in ~/ramp-runs."""
    for p in (os.path.expanduser(arg), os.path.join(RUN_DIR, arg), os.path.join(RUN_DIR, arg + ".rrd")):
        if os.path.isfile(p):
            return p
    sys.exit(f"no such run: {arg}")


def chunk_times(rb):
    """The 'time' column of a chunk as int64 nanoseconds since the epoch."""
    return rb.column("time").cast(pa.int64()).to_numpy()


def load(path):
    """Telemetry of one run. The camera pictures are skipped, so this is quick even for a big file."""
    path = find_run(path)
    run = Run(path, os.path.basename(path).removesuffix(".rrd"))
    parts = {}
    for ch in rrc.RrdReader(path).stream().drop(content=CAMERA_PATHS):
        rb, entity, names = ch.to_record_batch(), ch.entity_path, ch.to_record_batch().schema.names
        if ch.is_static:
            if entity == "/camera/calibration" and "TextDocument:text" in names:
                docs = [d for d in rb.column("TextDocument:text").to_pylist() if d]
                if docs:
                    run.calibration = json.loads(docs[0][0])
            elif entity == "/vesc/erpm" and "SeriesLines:names" in names:
                found = [n for n in rb.column("SeriesLines:names").to_pylist() if n]
                if found:
                    run.motors = tuple(found[0])
            continue
        if "time" not in names or rb.num_rows == 0:
            continue
        t = chunk_times(rb)
        if "Scalars:scalars" in names:
            v = rb.column("Scalars:scalars").flatten().to_numpy(zero_copy_only=False)
            parts.setdefault(entity, []).append((t, v.reshape(len(t), -1)))
        elif "TextLog:text" in names:
            for ti, text in zip(t.tolist(), rb.column("TextLog:text").to_pylist()):
                if text:
                    run.texts.setdefault(entity, []).append((ti, text[0]))
    for entity, ps in parts.items():
        t = np.concatenate([p[0] for p in ps])
        v = np.concatenate([p[1] for p in ps])
        order = np.argsort(t, kind="stable")
        run.scalars[entity] = (t[order], v[order].astype(np.float64))
    for rows in run.texts.values():
        rows.sort()
    return run


# ---------- putting streams on a common clock ----------

def latest_at(t_src, v_src, t_query):
    """For each query time, the most recent source row at or before it (NaN before the first)."""
    v_src = np.asarray(v_src, dtype=np.float64)
    out = np.full((len(t_query),) + v_src.shape[1:], np.nan)
    if len(t_src):
        i = np.searchsorted(t_src, t_query, side="right") - 1
        out[i >= 0] = v_src[i[i >= 0]]
    return out


def window_mean(t_src, v_src, t_query, window_ns):
    """For each query time, the mean of the source rows in (t - window, t] (NaN if there are none)."""
    v_src = np.asarray(v_src, dtype=np.float64)
    out = np.full((len(t_query),) + v_src.shape[1:], np.nan)
    if len(t_src):
        csum = np.concatenate([np.zeros((1,) + v_src.shape[1:]), np.cumsum(v_src, axis=0)])
        i0 = np.searchsorted(t_src, t_query - window_ns, side="right")
        i1 = np.searchsorted(t_src, t_query, side="right")
        n = i1 - i0
        ok = n > 0
        out[ok] = (csum[i1[ok]] - csum[i0[ok]]) / n[ok].reshape((-1,) + (1,) * (v_src.ndim - 1))
    return out


def latest_text(rows, t_query):
    """Like latest_at, for [(t_ns, text)] rows. None before the first."""
    if not rows:
        return [None] * len(t_query)
    ts = np.array([r[0] for r in rows])
    i = np.searchsorted(ts, t_query, side="right") - 1
    return [rows[j][1] if j >= 0 else None for j in i]


def control_table(run):
    """
    The control loop's state as columns on its own 50 Hz clock: {name: array}.
    Per-motor values get one column per motor, e.g. vesc_erpm_back. VESC and
    Teensy columns are NaN on ticks where the control script had no fresh reply.
    """
    if "/control/r1" not in run.scalars:
        return {}
    t = run.scalars["/control/r1"][0]
    cols = {"t_ns": t, "t": t / 1e9,
            "state": [None if s is None else s.split(":")[0].split()[0] for s in latest_text(run.texts.get("/control/state", []), t)]}

    def add(entity, name):
        if entity not in run.scalars:
            return
        ts, v = run.scalars[entity]
        i = np.clip(np.searchsorted(ts, t), 0, len(ts) - 1)
        same_tick = ts[i] == t
        for k in range(v.shape[1]):
            col = np.where(same_tick, v[i, k], np.nan)
            cols[name if v.shape[1] == 1 else f"{name}_{run.motors[k]}"] = col

    for key in ("r1", "stick", "steer", "jack", "speed", "auto", "erpm_cmd"):
        add(f"/control/{key}", key)
    for entity in sorted(e for e in run.scalars if e.startswith("/vesc/")):
        add(entity, "vesc_" + entity.split("/")[-1])
    add("/teensy/steer", "teensy_steer")
    add("/teensy/jack", "teensy_jack")
    return cols


# ---------- summary ----------

def summarize(run):
    c = control_table(run)
    if not c:
        return f"{run.name}: no control data"
    t = c["t"]
    dur = t[-1] - t[0]
    hz = 1.0 / np.median(np.diff(t)) if len(t) > 1 else 0.0
    tick = 1.0 / hz if hz else 0.0
    cmd = np.nanmean(np.stack([c[k] for k in c if k.startswith("erpm_cmd")]), axis=0)
    steer = np.nan_to_num(c["steer"])
    fwd, back = cmd > 200, cmd < -200
    starts = np.flatnonzero(np.diff(np.concatenate([[0], steer])) != 0)
    starts = starts[steer[starts] != 0]
    ends = np.flatnonzero(np.diff(np.concatenate([steer, [0]])) != 0)
    ends = ends[steer[ends] != 0]
    taps = (ends - starts + 1) * tick if len(starts) == len(ends) else np.zeros(0)
    states = [text for _, text in run.texts.get("/control/state", [])]
    lost = sum("controller signal lost" in s for s in states)
    lines = [f"{run.name}  {dur:6.1f} s   control {hz:.0f} Hz",
             f"  driving   forward {fwd.sum() * tick:5.1f} s   back {back.sum() * tick:5.1f} s",
             f"  steering  {len(taps)} taps" + (f", median {np.median(taps):.2f} s, longest {taps.max():.2f} s" if len(taps) else "")
             + f"   (left {(steer < 0).sum() * tick:.1f} s, right {(steer > 0).sum() * tick:.1f} s)",
             f"  faults    controller signal lost x{lost}   state changes {len(states)}"]
    for name in ("gyro", "accel"):
        if f"/imu/{name}" in run.scalars and dur > 0:
            lines[0] += f"   {name} {len(run.scalars[f'/imu/{name}'][0]) / dur:.0f} Hz"
    erpm = [c[k] for k in c if k.startswith("vesc_erpm")]
    if erpm and np.isfinite(erpm[0]).any():
        v_in = np.concatenate([c[k] for k in c if k.startswith("vesc_v_in")])
        lines.append(f"  vesc      erpm {np.nanmin(erpm):.0f} .. {np.nanmax(erpm):.0f}   battery {np.nanmin(v_in):.1f}-{np.nanmax(v_in):.1f} V")
    return "\n".join(lines)


def main():
    args = sys.argv[1:]
    if args and args[0] in ("-h", "--help"):
        sys.exit(__doc__.strip())
    paths = [find_run(a) for a in args] or sorted(glob.glob(os.path.join(RUN_DIR, "*.rrd")))
    if not paths:
        sys.exit(f"no runs in {RUN_DIR}")
    for p in paths:
        print(summarize(load(p)), flush=True)


if __name__ == "__main__":
    main()
