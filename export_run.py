#!/usr/bin/env python3
"""
Turn recorded runs (.rrd) into plain folders of pictures and tables, for
training on another machine.

  .venv/bin/python export_run.py RUN.rrd ...        export these runs
  .venv/bin/python export_run.py --all              export every run in ~/ramp-runs
        -o DIR          where to write (default ~/ramp-export); each run gets DIR/<run name>/
        --skip CAM ...  leave out some pictures: color, depth, ir, rear
        --force         replace an export that is already there

A bare file name means one in ~/ramp-runs. What gets written is described in
SETUP.md ("Exporting runs for training"). The pictures are copied out as
recorded (JPEG, and 16-bit PNG for depth), not re-encoded, so an export is
about the size of the .rrd it came from.
"""
import argparse
import glob
import json
import os
import shutil
import sys
import time

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import rerun.chunk as rrc

from extract_telemetry import (CAMERAS, RUN_DIR, chunk_times, control_table, find_run, latest_at,
                               latest_text, load, window_mean)

EXPORT_DIR = os.path.expanduser("~/ramp-export")
FORMAT_VERSION = 1
FRONT = ("color", "depth", "ir")        # logged together, with one timestamp per frame
EXT = {"color": "jpg", "ir": "jpg", "rear": "jpg", "depth": "png"}
CHECK_EVERY = 60                        # look at one picture in this many for the health check


def blobs(rb):
    """The encoded pictures of a chunk, one bytes object per row."""
    col = rb.column([n for n in rb.schema.names if n.endswith(":blob")][0])
    inner = col.flatten()                               # one list<uint8> per row
    if len(inner) != rb.num_rows:                       # a row with no picture: take the slow, careful path
        return [None if b is None else bytes(b[0]) for b in col.to_pylist()]
    data = inner.values.to_numpy(zero_copy_only=False)
    off = inner.offsets.to_numpy()
    return [data[off[i]:off[i + 1]].tobytes() for i in range(rb.num_rows)]


def health(cam, blob, cv2):
    """One number per checked picture: % of depth pixels with a reading, or % of pixels blown out."""
    img = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    if cam == "depth":
        return float((img > 0).mean() * 100)
    return float(((img if img.ndim == 2 else img.max(axis=2)) >= 250).mean() * 100)


def write_pictures(path, cam, out):
    """Write every picture of one camera under a temporary name. Returns (times ns, temp paths, health numbers)."""
    try:
        import cv2
    except ImportError:
        cv2 = None
    os.makedirs(os.path.join(out, cam))
    times, files, checks = [], [], []
    stream = rrc.RrdReader(path).stream().filter(content=f"/camera/{cam}", is_static=False)
    for ch in stream:
        rb = ch.to_record_batch()
        if "time" not in rb.schema.names:
            continue
        for t, blob in zip(chunk_times(rb).tolist(), blobs(rb)):
            if blob is None:
                continue
            tmp = os.path.join(out, cam, f"tmp_{len(files)}")
            with open(tmp, "wb") as f:
                f.write(blob)
            if cv2 is not None and len(files) % CHECK_EVERY == 0:
                h = health(cam, blob, cv2)
                if h is not None:
                    checks.append(h)
            times.append(t)
            files.append(tmp)
    return np.array(times, dtype=np.int64), files, checks


def name_pictures(cam, times, files, index_of):
    """Rename temp files to <frame index>.<ext>; returns {frame index: relative path}. Repeated timestamps are dropped."""
    named = {}
    for t, tmp in zip(times.tolist(), files):
        i = index_of[t]
        if i in named:
            os.remove(tmp)
            continue
        rel = f"{cam}/{i:06d}.{EXT[cam]}"
        os.replace(tmp, os.path.join(os.path.dirname(os.path.dirname(tmp)), rel))
        named[i] = rel
    return named


def write_table(path, cols):
    pq.write_table(pa.table({k: pa.array(v) for k, v in cols.items()}), path)


def export(path, out_root, skip=(), force=False):
    run = load(path)
    final = os.path.join(out_root, run.name)
    out = final + ".partial"            # renamed when complete, so a folder without that suffix is a whole export
    if os.path.exists(final):
        if not force:
            print(f"{run.name}: already exported to {final} (--force to replace)")
            return
        shutil.rmtree(final)
    if os.path.exists(out):
        shutil.rmtree(out)
    need = os.path.getsize(path) * 1.1
    os.makedirs(out_root, exist_ok=True)
    if shutil.disk_usage(out_root).free < need:
        sys.exit(f"{run.name}: needs about {need / 1e9:.1f} GB free in {out_root}")
    os.makedirs(out)
    started = time.monotonic()

    # pictures
    cams = [c for c in CAMERAS if c not in skip]
    raw = {}
    for cam in cams:
        raw[cam] = write_pictures(path, cam, out)
        print(f"  {run.name}: {cam} {len(raw[cam][1])} pictures", flush=True)
    front_t = np.unique(np.concatenate([raw[c][0] for c in FRONT if c in raw] or [np.zeros(0, np.int64)]))
    rear_t = np.unique(raw["rear"][0]) if "rear" in raw else np.zeros(0, np.int64)
    named = {}
    for cam in cams:
        clock = rear_t if cam == "rear" else front_t
        named[cam] = name_pictures(cam, raw[cam][0], raw[cam][1], {t: i for i, t in enumerate(clock.tolist())})

    # the control loop and the IMU, as recorded
    control = control_table(run)
    if control:
        write_table(os.path.join(out, "control.parquet"), control)
    imu = {}
    for name in ("gyro", "accel"):
        if f"/imu/{name}" in run.scalars:
            t, v = imu[name] = run.scalars[f"/imu/{name}"]
            write_table(os.path.join(out, f"imu_{name}.parquet"),
                        {"t_ns": t, "t": t / 1e9, "x": v[:, 0], "y": v[:, 1], "z": v[:, 2]})

    # one row per front frame, with the rear picture nearest in time and the telemetry as of that frame
    def frame_rows(clock, own):
        cols = {"frame": np.arange(len(clock)), "t_ns": clock, "t": clock / 1e9}
        for cam in own:
            if cam in named:
                cols[cam] = [named[cam].get(i) for i in range(len(clock))]
        return cols

    frames = frame_rows(front_t, FRONT)
    if len(front_t) and len(rear_t):
        j = np.clip(np.searchsorted(rear_t, front_t), 1, len(rear_t) - 1) if len(rear_t) > 1 else np.zeros(len(front_t), int)
        j = np.where(np.abs(rear_t[j - 1] - front_t) <= np.abs(rear_t[j] - front_t), j - 1, j) if len(rear_t) > 1 else j
        frames["rear"] = [named["rear"].get(int(i)) for i in j]
        frames["rear_dt"] = (rear_t[j] - front_t) / 1e9          # rear picture time minus this frame's time
    if control and len(front_t):
        tc = control["t_ns"]
        frames["state"] = latest_text(list(zip(tc.tolist(), control["state"])), front_t)
        for k, v in control.items():
            if k not in ("t_ns", "t", "state"):
                frames[k] = latest_at(tc, v, front_t)
        frames["control_age"] = (front_t - latest_at(tc, tc, front_t)) / 1e9
    period = int(np.median(np.diff(front_t))) if len(front_t) > 1 else 33_333_333
    for name, (t, v) in imu.items():
        mean = window_mean(t, v, front_t, period)                # averaged over the frame interval ending at t
        for k, axis in enumerate("xyz"):
            frames[f"{name}_{axis}"] = mean[:, k]
    if len(front_t):
        write_table(os.path.join(out, "frames.parquet"), frames)
    if len(rear_t):
        write_table(os.path.join(out, "rear_frames.parquet"), frame_rows(rear_t, ("rear",)))

    # the rest
    if run.calibration:
        with open(os.path.join(out, "calibration.json"), "w") as f:
            json.dump(run.calibration, f, indent=1)
    with open(os.path.join(out, "events.json"), "w") as f:
        json.dump([{"t": t / 1e9, "source": entity.strip("/"), "text": text}
                   for entity, rows in run.texts.items() for t, text in rows], f, indent=1)

    def stats(vals):
        return None if not vals else {"median": round(float(np.median(vals)), 1), "min": round(min(vals), 1), "max": round(max(vals), 1)}

    checks = {"depth_valid_pct": stats(raw["depth"][2]) if "depth" in raw else None,
              **{f"{cam}_blown_out_pct": stats(raw[cam][2]) for cam in ("ir", "color", "rear") if cam in raw}}
    span = (front_t[-1] - front_t[0]) / 1e9 if len(front_t) > 1 else 0.0
    meta = {"format_version": FORMAT_VERSION, "run": run.name, "source": os.path.basename(path),
            "start_t": float(front_t[0] / 1e9) if len(front_t) else None, "duration_s": round(span, 2),
            "front_frames": len(front_t), "rear_frames": len(rear_t),
            "front_fps": round((len(front_t) - 1) / span, 2) if span else None,
            "pictures": {cam: len(named[cam]) for cam in cams}, "skipped": sorted(skip),
            "motors": list(run.motors),
            "depth_unit_m": run.calibration.get("depth_scale") if run.calibration else None,
            "checks": checks}
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)

    os.rename(out, final)
    out = final
    size = sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(out) for f in fs)
    print(f"{run.name}: {len(front_t)} front frames, {len(rear_t)} rear, {size / 1e9:.2f} GB "
          f"in {time.monotonic() - started:.0f} s -> {out}")
    dv, ir = checks.get("depth_valid_pct"), checks.get("ir_blown_out_pct")
    if dv and dv["median"] < 30:
        print(f"  WARNING: depth is mostly empty (median {dv['median']}% of pixels have a reading)")
    if ir and ir["median"] > 50:
        print(f"  WARNING: infrared is blown out (median {ir['median']}% of pixels saturated)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help=".rrd files (a bare name means one in ~/ramp-runs)")
    ap.add_argument("--all", action="store_true", help="every run in ~/ramp-runs")
    ap.add_argument("-o", "--out", default=EXPORT_DIR, help="output directory (default ~/ramp-export)")
    ap.add_argument("--skip", nargs="+", choices=CAMERAS, default=[], help="pictures to leave out")
    ap.add_argument("--force", action="store_true", help="replace an existing export")
    args = ap.parse_args()
    paths = sorted(glob.glob(os.path.join(RUN_DIR, "*.rrd"))) if args.all else [find_run(r) for r in args.runs]
    if not paths:
        ap.error("name some runs, or pass --all")
    for p in paths:
        export(p, os.path.expanduser(args.out), skip=set(args.skip), force=args.force)


if __name__ == "__main__":
    main()
