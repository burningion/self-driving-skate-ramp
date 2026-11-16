#!/usr/bin/env -S uv run --script
# /// script
# requires-python = "==3.10.*"
# dependencies = [
#     "pycolmap-cuda12==4.2.1",
#     "numpy",
#     "pandas",
#     "pyarrow",
#     "scipy",
# ]
# ///
"""
Work out where the ramp was during a run, inside a map made by build_map.py.

  ./localize_run.py RUN --map NAME      RUN is a folder in ~/ramp-export (from export_run.py)
        --step-m N        look at a front picture every N metres driven (default 0.3)
        --neighbors N     map frames each picture is matched against (default 20)
        --min-inliers N   matched points needed to trust a position (default 30)
        --calibration F   calibration.json to use if the run has none
        --maps-dir DIR    where maps live (default /scratch/ramp-maps)
        --export-dir DIR  where exported runs live (default ~/ramp-export, then ~/Development/ramp-export)
        --keep            keep the working database (in the map's runs/ folder)
        --verbose         show COLMAP's own log

Writes RUN/trajectory.parquet, one row per front frame: the camera's position
(x y z) and orientation (qx qy qz qw, camera to world, camera looking along +z
with +y down) in the map's sfm/ coordinates, and where it came from:
  fix     a picture matched against the map (`inliers` says how well)
  interp  between two fixes: dead-reckoned from each by how far the wheels turned, and blended
  odom    before the first or after the last fix, dead-reckoned from it
  none    more than 5 m of driving from any fix (outside the map, most likely): no position
RUN/trajectory.json has the summary, with the map's size in map units per metre.

How: the front pictures are matched against the map's frames on the GPU (SIFT)
in two rounds. First a dozen pictures spread over the run against a thinned-out
map, to find roughly where the run is; then every picture against only the map
frames near where the wheels say it should be. Each picture's position comes
from its matched points' known 3D positions (PnP with RANSAC).
"""
import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

os.environ.setdefault("SSL_CERT_FILE", "/etc/ssl/certs/ca-certificates.crt")

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp

MAPS_DIR = Path("/scratch/ramp-maps")
EXPORT_DIRS = [Path(p).expanduser() for p in ("~/ramp-export", "~/Development/ramp-export")]   # first one that has the run
M_PER_ERPM = 3.9e-4             # m/s per unit of VESC ERPM; the same number as autosteer.py's m_per_erpm
ANCHORS = 12                    # pictures matched against the whole (thinned) map in the first round
COARSE_MAP = 300                # map frames in the thinned map
MAX_GAP_S = 3.0                 # look at a picture at least this often, even when standing still
MAX_RECKON_M = 5.0              # further than this (by the wheels) from any fix, a frame gets no position
OUTLIER_M = 0.5                 # a weak fix this far from where its neighbours and the wheels put it is dropped
STRONG_INLIERS = 100            # a fix with this many matched points is trusted over the wheels, whatever they say


def log(msg):
    print(msg, flush=True)


# ---------- the run ----------

def opencv_camera(calibration):
    """The front colour camera as COLMAP OPENCV parameters. The RealSense's model (inverse Brown-Conrady) maps
    distorted to undistorted points; COLMAP's goes the other way, so fit its coefficients to match (linear)."""
    c = calibration["color"]
    fx, fy, cx, cy = c["fx"], c["fy"], c["ppx"], c["ppy"]
    k1, k2, p1, p2, k3 = c["coeffs"]
    u, v = np.meshgrid(np.linspace(0, c["width"], 25), np.linspace(0, c["height"], 19))
    xd, yd = ((u - cx) / fx).ravel(), ((v - cy) / fy).ravel()
    r2 = xd * xd + yd * yd
    f = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    x = xd * f + 2 * p1 * xd * yd + p2 * (r2 + 2 * xd * xd)        # undistorted
    y = yd * f + 2 * p2 * xd * yd + p1 * (r2 + 2 * yd * yd)
    r2 = x * x + y * y                                             # now solve for the forward coefficients
    A = np.r_[np.c_[x * r2, x * r2 ** 2, 2 * x * y, r2 + 2 * x * x], np.c_[y * r2, y * r2 ** 2, r2 + 2 * y * y, 2 * x * y]]
    coef = np.linalg.lstsq(A, np.r_[xd - x, yd - y], rcond=None)[0]
    return c["width"], c["height"], [fx, fy, cx, cy, *coef]


def load_run(run_dir, calibration_path):
    frames = pd.read_parquet(run_dir / "frames.parquet")
    cal = calibration_path or run_dir / "calibration.json"
    if not Path(cal).is_file():
        sys.exit(f"{run_dir.name} has no calibration.json: pass --calibration with one from another run of the same camera")
    t = frames.t.to_numpy()
    erpm = frames[["vesc_erpm_back", "vesc_erpm_front"]].mean(axis=1).fillna(0).to_numpy()
    odom = np.r_[0, np.cumsum(erpm[1:] * np.diff(t) * M_PER_ERPM)]     # metres along the path, signed
    return frames, t, odom, opencv_camera(json.load(open(cal)))


def pick_frames(t, odom, step_m):
    """Rows to look at: one every step_m metres driven, and at least one every MAX_GAP_S."""
    picks = [0]
    for i in range(1, len(t)):
        if abs(odom[i] - odom[picks[-1]]) >= step_m or t[i] - t[picks[-1]] >= MAX_GAP_S:
            picks.append(i)
    return picks


# ---------- from fixes to a pose for every frame ----------

def motion_model(rows, pos, fwd, odom, t):
    """How the wheels' metres map onto the map: units per metre, and whether driving forward (positive ERPM)
    moves the camera the way it looks (+1) or the other way (-1). From pairs of fixes close together in time."""
    r = np.asarray(rows)
    dd, dp, dt = np.diff(odom[r]), np.diff(pos, axis=0), np.diff(t[r])
    ok = np.abs(dd) > 0.25
    if (ok & (dt < 5)).sum() >= 3:
        ok &= dt < 5
    if not ok.any():
        return float("nan"), 1.0
    scale = float(np.median(np.linalg.norm(dp[ok], axis=1) / np.abs(dd[ok])))
    along = np.einsum("ij,ij->i", dp[ok], fwd[:-1][ok]) / dd[ok]
    return scale, 1.0 if np.median(along) >= 0 else -1.0


def reckon(k, at, rows, pos, step, odom):
    """Where fix k and the wheels put the rows `at`: from the fix, along the way its camera looked."""
    return pos[k] + (odom[at] - odom[np.asarray(rows)[k]])[..., None] * step[k]


def drop_outliers(rows, pos, fwd, inliers, odom, t):
    """Drop weak fixes that are not where their neighbours and the wheels say they should be."""
    keep = np.ones(len(rows), bool)
    for _ in range(max(1, len(rows) // 10)):
        idx = np.flatnonzero(keep)
        r = [rows[i] for i in idx]
        scale, sign = motion_model(r, pos[idx], fwd[idx], odom, t)
        if len(idx) < 4 or not np.isfinite(scale):
            break
        step = sign * scale * fwd[idx]
        res = np.zeros(len(idx))
        for j in np.flatnonzero(inliers[idx] < STRONG_INLIERS):
            near = [k for k in (j - 1, j + 1) if 0 <= k < len(idx)]
            k = min(near, key=lambda k: abs(odom[r[k]] - odom[r[j]]))          # the neighbour fewer metres away
            slack = scale * (OUTLIER_M + 0.1 * abs(odom[r[k]] - odom[r[j]]))   # the wheels drift a little
            res[j] = np.linalg.norm(pos[idx[j]] - reckon(k, r[j], r, pos[idx], step, odom)) / slack
        if res.max() <= 1:
            break
        keep[idx[np.argmax(res)]] = False                # one at a time: a bad fix makes its neighbours look bad
    return keep


def fuse(t, odom, fixes):
    """fixes: {row: (centre, camera-to-world Rotation, matched points)} -> positions, rotations, source for every row; the map
    units per metre; and the rows of the fixes kept. Between two fixes a row is dead-reckoned from each (the
    wheels' metres along the way the camera looked) and the two are blended by time."""
    rows = sorted(fixes)
    pos = np.array([fixes[r][0] for r in rows])
    rot = Rotation.concatenate([fixes[r][1] for r in rows])
    keep = drop_outliers(rows, pos, rot.apply([0, 0, 1]), np.array([fixes[r][2] for r in rows]), odom, t)
    rows, pos, rot = [r for r, k in zip(rows, keep) if k], pos[keep], rot[np.flatnonzero(keep)]
    fwd = rot.apply([0, 0, 1])
    scale, sign = motion_model(rows, pos, fwd, odom, t)
    step = sign * scale * fwd if np.isfinite(scale) else np.zeros_like(fwd)    # map units per metre driven
    n, r_arr = len(t), np.asarray(rows)
    every = np.arange(n)
    source = np.full(n, "interp", dtype=object)
    source[(every < r_arr[0]) | (every > r_arr[-1])] = "odom"
    source[r_arr] = "fix"
    if len(rows) == 1:
        P = reckon(0, every, rows, pos, step, odom)
        far = np.abs(odom - odom[r_arr[0]]) > MAX_RECKON_M
        P[far], source[far] = np.nan, "none"
        return P, Rotation.concatenate([rot] * n), source, scale, rows
    a = np.clip(np.searchsorted(r_arr, every, side="right") - 1, 0, len(rows) - 2)
    w = np.clip((t - t[r_arr[a]]) / np.maximum(t[r_arr[a + 1]] - t[r_arr[a]], 1e-9), 0, 1)[:, None]
    w = np.where((every < r_arr[0])[:, None], 0.0, np.where((every > r_arr[-1])[:, None], 1.0, w))
    P = (1 - w) * reckon(a, every, rows, pos, step, odom) + w * reckon(a + 1, every, rows, pos, step, odom)
    P[r_arr] = pos
    nearest = np.clip(np.searchsorted(r_arr, every), 1, len(rows) - 1)         # the fix on either side
    far = np.minimum(np.abs(odom - odom[r_arr[nearest - 1]]), np.abs(odom - odom[r_arr[nearest]])) > MAX_RECKON_M
    P[far], source[far] = np.nan, "none"
    R = Slerp(t[r_arr], rot)(np.clip(t, t[r_arr[0]], t[r_arr[-1]]))            # outside the fixes: hold the nearest one's
    return P, R, source, scale, rows


# ---------- matching against the map ----------

class Matcher:
    def __init__(self, map_dir, run_dir, work, camera, verbose):
        import pycolmap
        self.pc = pycolmap
        if not pycolmap.has_cuda:
            sys.exit("this pycolmap was built without CUDA")
        if not verbose:
            pycolmap.logging.minloglevel = 2
        self.rec = pycolmap.Reconstruction(map_dir / "sfm" / "0")
        self.map_ids = sorted(self.rec.reg_image_ids(), key=lambda i: self.rec.image(i).name)
        self.map_centres = np.array([self.rec.image(i).projection_center() for i in self.map_ids])
        self.map_forward = np.array([self.rec.image(i).cam_from_world().rotation.matrix()[2] for i in self.map_ids])
        self.width, self.height, params = camera
        self.camera = pycolmap.Camera(model="OPENCV", width=self.width, height=self.height, params=params)
        self.params = ",".join(f"{p:.9g}" for p in params)
        self.work, self.run, self.run_dir = work, run_dir.name, run_dir
        self.db_path = work / "database.db"
        shutil.copy(map_dir / "sfm" / "database.db", self.db_path)
        (work / "query").mkdir()
        (work / "query" / self.run).symlink_to(run_dir / "color")
        self.ids = {}                                   # frames row -> image id in the database
        self.pairs = {}                                 # frames row -> set of map image ids it was matched against

    def name(self, frame):
        return f"{self.run}/{frame:06d}.jpg"

    def extract(self, rows, frame_numbers):
        pc = self.pc
        names = [self.name(frame_numbers[r]) for r in rows]
        reader = pc.ImageReaderOptions(camera_model="OPENCV", camera_params=self.params)
        pc.extract_features(self.db_path, self.work / "query", image_names=names, camera_mode=pc.CameraMode.SINGLE,
                            reader_options=reader, device=pc.Device.cuda)
        with pc.Database.open(self.db_path) as db:
            for r, n in zip(rows, names):
                self.ids[r] = db.read_image_with_name(n).image_id

    def match(self, wanted, frame_numbers):
        """wanted: {row: iterable of map image ids}. Matches the pairs not matched yet."""
        pc = self.pc
        lines = []
        for r, ids in wanted.items():
            new = set(ids) - self.pairs.setdefault(r, set())
            lines += [f"{self.name(frame_numbers[r])} {self.rec.image(i).name}" for i in sorted(new)]
            self.pairs[r] |= new
        if not lines:
            return 0
        listing = self.work / "pairs.txt"
        listing.write_text("\n".join(lines) + "\n")
        pairing = pc.ImportedPairingOptions()
        pairing.match_list_path = listing
        pc.match_image_pairs(self.db_path, pairing_options=pairing, device=pc.Device.cuda)
        return len(lines)

    def locate(self, rows, min_inliers):
        """PnP for each row from its matches to map points -> {row: (centre, Rotation camera-to-world, inliers)}."""
        pc, out = self.pc, {}
        with pc.Database.open(self.db_path) as db:
            for r in rows:
                q = self.ids[r]
                xy = db.read_keypoints(q)[:, :2]
                seen = {}                               # query keypoint -> map point id
                for m in self.pairs.get(r, ()):
                    if not db.exists_two_view_geometry(q, m):
                        continue
                    points2D = self.rec.image(m).points2D
                    for kq, km in db.read_two_view_geometry(q, m).inlier_matches:
                        p = points2D[int(km)]
                        if p.has_point3D():
                            seen.setdefault(int(kq), p.point3D_id)
                if len(seen) < min_inliers:
                    continue
                pts2 = xy[list(seen)].astype(np.float64)
                pts3 = np.array([self.rec.point3D(i).xyz for i in seen.values()])
                est = pc.estimate_and_refine_absolute_pose(pts2, pts3, self.camera)
                if est is None or est["num_inliers"] < min_inliers:
                    continue
                cfw = est["cam_from_world"]
                Rcw = cfw.rotation.matrix()
                out[r] = (-Rcw.T @ cfw.translation, Rotation.from_matrix(Rcw.T), int(est["num_inliers"]))
        return out

    def near(self, centre, forward, k):
        """The k map frames nearest to a position, preferring ones that look the same way."""
        if not np.isfinite(centre).all():               # too far from any fix to guess where it is
            return []
        same_way = self.map_forward @ forward > 0.3
        dist = np.linalg.norm(self.map_centres - centre, axis=1)
        pool = np.flatnonzero(same_way) if same_way.sum() >= k else np.arange(len(dist))
        return [self.map_ids[i] for i in pool[np.argsort(dist[pool])[:k]]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--map", required=True)
    ap.add_argument("--step-m", type=float, default=0.3)
    ap.add_argument("--neighbors", type=int, default=20)
    ap.add_argument("--min-inliers", type=int, default=30)
    ap.add_argument("--calibration", type=Path)
    ap.add_argument("--maps-dir", type=Path, default=MAPS_DIR)
    ap.add_argument("--export-dir", type=Path)
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    places = [Path(args.run)] + [d / args.run for d in ([args.export_dir] if args.export_dir else EXPORT_DIRS)]
    run_dir = next((p for p in places if (p / "frames.parquet").is_file()), places[-1])
    map_dir = args.maps_dir / args.map
    if not (run_dir / "frames.parquet").is_file():
        sys.exit(f"no exported run {args.run} in {', '.join(str(p.parent) for p in places[1:])}: export it first with export_run.py")
    if not (map_dir / "sfm" / "0" / "images.bin").is_file():
        sys.exit(f"no map at {map_dir}: build it first with build_map.py {args.map}")
    run_dir = run_dir.resolve()

    frames, t, odom, camera = load_run(run_dir, args.calibration)
    numbers = frames.frame.to_numpy()
    rows = [r for r in pick_frames(t, odom, args.step_m) if (run_dir / "color" / f"{numbers[r]:06d}.jpg").is_file()]
    work = map_dir / "runs" / run_dir.name
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    log(f"{run_dir.name}: {t[-1] - t[0]:.0f} s, {np.abs(np.diff(odom)).sum():.0f} m driven, {len(rows)} pictures to place in map '{args.map}'")

    start = time.time()
    m = Matcher(map_dir, run_dir, work, camera, args.verbose)
    m.extract(rows, numbers)

    # round 1: a few pictures against a thinned-out map, to find roughly where the run is
    anchors = [rows[i] for i in np.unique(np.linspace(0, len(rows) - 1, min(ANCHORS, len(rows))).round().astype(int))]
    coarse = m.map_ids[::max(1, len(m.map_ids) // COARSE_MAP)]
    n = m.match({r: coarse for r in anchors}, numbers)
    found = m.locate(anchors, args.min_inliers)
    log(f"   round 1: {len(found)} of {len(anchors)} spread-out pictures found in the map ({n} pairs, {time.time() - start:.0f} s)")
    if not found:
        sys.exit("   none of them matched the map. Is this the right map? Does the map have a pass at the ramp's height?")

    # round 2: every picture, against the map frames near where round 1 and the wheels put it
    P, R, _, _, _ = fuse(t, odom, found)
    forward = R.apply([0, 0, 1])
    n = m.match({r: m.near(P[r], forward[r], args.neighbors) for r in rows}, numbers)
    found = m.locate(rows, args.min_inliers)
    log(f"   round 2: {len(found)} of {len(rows)} pictures placed ({n} pairs, {time.time() - start:.0f} s)")

    # further looks at the ones that failed, now that their neighbours are known
    for _ in range(3):
        missing = [r for r in rows if r not in found]
        if not missing or not found:
            break
        P, R, _, _, _ = fuse(t, odom, found)
        forward = R.apply([0, 0, 1])
        before = len(found)
        if m.match({r: m.near(P[r], forward[r], args.neighbors) for r in missing}, numbers):
            found.update(m.locate(missing, args.min_inliers))
        if len(found) == before:
            break
        log(f"   retry: {len(found)} of {len(rows)} placed ({time.time() - start:.0f} s)")

    P, R, source, scale, kept = fuse(t, odom, found)
    inliers = np.zeros(len(t), int)
    for r in kept:
        inliers[r] = found[r][2]
    q = R.as_quat()
    out = pd.DataFrame({"frame": numbers, "t_ns": frames.t_ns, "t": t, "x": P[:, 0], "y": P[:, 1], "z": P[:, 2],
                        "qx": q[:, 0], "qy": q[:, 1], "qz": q[:, 2], "qw": q[:, 3],
                        "source": source, "inliers": inliers, "odom_m": odom})
    out.to_parquet(run_dir / "trajectory.parquet", index=False)

    gaps = np.diff(t[kept]) if len(kept) > 1 else np.array([np.inf])
    summary = {"run": run_dir.name, "map": args.map, "map_dir": str(map_dir), "units_per_m": scale,
               "pictures": len(rows), "fixes": len(kept), "dropped_as_outliers": len(found) - len(kept),
               "median_inliers": float(np.median([found[r][2] for r in kept])), "longest_gap_s": float(gaps.max()),
               "frames": {k: int(v) for k, v in pd.Series(source).value_counts().items()}}
    json.dump(summary, open(run_dir / "trajectory.json", "w"), indent=1)
    if not args.keep:
        shutil.rmtree(work)
    log(f"   {len(kept)} fixes ({len(found) - len(kept)} dropped as outliers), median {summary['median_inliers']:.0f} matched points,"
        f" longest stretch without one {gaps.max():.1f} s")
    log(f"   map scale: {scale:.3f} map units per metre" if np.isfinite(scale) else "   map scale: unknown (the ramp barely moved)")
    log(f"-> {run_dir / 'trajectory.parquet'}")
    lost = int((source == "none").sum())
    if lost:
        log(f"   NOTE: {lost / len(t):.0%} of the run ({lost / 30:.0f} s) has no position: no picture there matched the map,"
            " so the ramp was outside it or the map looks too different there.")


if __name__ == "__main__":
    main()
