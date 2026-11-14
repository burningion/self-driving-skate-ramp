#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "opencv-python-headless", "pandas", "pyarrow"]
# ///
"""
Turn sidewalk labels into where the ramp is on the sidewalk: how far off the
centre line, and how far it is pointed away from the sidewalk's direction.

  uv run sidewalk_geometry.py RUN_DIR ...
        --pose FILE        use this camera_pose.json (default: RUN_DIR's, else work it out)
        --height M         front camera height above the ground (default: measured from depth)
        --rear-height M    rear camera height (default: same as the front)
        --rear-fx PX       rear camera focal length in pixels (default 615, a Logitech C920 at 640x480)
        --width M          sidewalk width (default: measured from the pictures that show both edges)
        --paved WORD ...   class-name words that count as sidewalk (default: see PAVED_WORDS)

RUN_DIR is a folder written by export_run.py that label_sidewalk.py has been
run on. Writes into it:

  camera_pose.json              each camera's height and tilt, as used
  geometry.parquet              one row per labelled front frame
  geometry_rear.parquet         one row per labelled rear frame
  geometry_preview_<cam>.jpg    pictures with the fitted edges drawn on, to check by eye

Columns, the same for both cameras and both in the ramp's own terms:
  offset_m      how far the camera is from the sidewalk's centre line; + = right of centre, facing forwards
  heading_rad   which way the ramp points relative to the sidewalk; + = turned clockwise seen from above
  width_m       sidewalk width (should come out the same all along a sidewalk: a check on the rest)
  edges         how many edges were found: 2, 1 (the other side filled in from the usual width) or 0 (no result)

How it works: follow the sidewalk up the picture from the bottom row, fit a
straight line to each edge, and put those lines on the ground using the
camera's height and tilt. The front camera's height and tilt come from fitting
a plane to its depth pictures. The rear camera has no depth, so its tilt comes
from where the two edges meet in the picture (the horizon), and its height
from making the sidewalk as wide as the front camera measured it.
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd

# A class counts as sidewalk if its name contains one of these. Road is included because from
# a few inches off the ground a segmentation model often calls the sidewalk "road".
PAVED_WORDS = ("sidewalk", "road", "pedestrian area", "bike lane", "crosswalk", "curb cut",
               "service lane", "parking", "lane marking", "manhole", "catch basin")

Z_FAR = 4.0             # ignore sidewalk farther ahead than this (m): too few pixels per metre
MIN_WIDTH_PX = 30       # stop following the sidewalk up the picture when it gets narrower than this
BORDER_PX = 3           # an edge this close to the side of the picture is the picture's edge, not the sidewalk's
MIN_EDGE_ROWS = 12      # need this many rows to fit an edge...
MIN_EDGE_SPAN = 25      # ...covering at least this many rows of the picture
ROW_STEP = 2
MAX_EDGE_RMS_PX = 6.0   # an edge that fits a straight line worse than this is not used
WIDTH_TOLERANCE = 0.25  # a two-edge width more than this fraction off the usual one: don't trust the frame


def norm_name(s):
    return s.lower().replace("-", " ").replace("_", " ")


def paved_ids(id2label, words=PAVED_WORDS):
    """Class ids whose name contains one of the words."""
    return sorted(int(i) for i, name in id2label.items() if any(w in norm_name(name) for w in words))


# ---------- camera on the ground ----------

class Ground:
    """A camera at a known height and tilt above flat ground: picture positions <-> ground positions."""

    def __init__(self, fx, fy, cx, cy, down, height_m, **_):
        self.fx, self.fy, self.cx, self.cy, self.h = fx, fy, cx, cy, height_m
        g = np.asarray(down, float) / np.linalg.norm(down)      # "down" in camera axes (x right, y down, z forward)
        fwd = np.array([0, 0, 1.0]) - g[2] * g                  # the camera's forward, flattened onto the ground
        fwd /= np.linalg.norm(fwd)
        self.g, self.R = g, np.stack([np.cross(g, fwd), g, fwd])

    def to_ground(self, u, v):
        """Pixel -> (X to the right, Z ahead) on the ground, in metres. NaN above the horizon."""
        ray = np.stack([(np.asarray(u, float) - self.cx) / self.fx, (np.asarray(v, float) - self.cy) / self.fy,
                        np.ones(np.shape(u))], axis=-1) @ self.R.T
        with np.errstate(divide="ignore", invalid="ignore"):
            s = np.where(ray[..., 1] > 1e-4, self.h / ray[..., 1], np.nan)
        return s * ray[..., 0], s * ray[..., 2]

    def horizon_row(self, u=None):
        u = self.cx if u is None else u
        return self.cy - self.fy * (self.g[0] * (u - self.cx) / self.fx + self.g[2]) / self.g[1]

    def top_row(self):
        """The highest picture row worth using: where the ground straight ahead is Z_FAR away."""
        return self.horizon_row() + self.h * self.fy / Z_FAR


def describe(pose):
    g = np.asarray(pose["down"], float)
    g = g / np.linalg.norm(g)
    return (f"height {pose['height_m'] * 100:.1f} cm, tilted {np.degrees(np.arcsin(-g[2])):+.1f} deg up, "
            f"rolled {np.degrees(np.arctan2(g[0], g[1])):+.1f} deg  ({pose['source']})")


def front_pose_from_depth(run_dir, cal):
    """Fit a plane to the ground in the depth pictures. Returns a pose in the colour camera's axes, or None."""
    k = cal["depth"]
    files = sorted(glob.glob(os.path.join(run_dir, "depth", "*.png")))
    files = files[::max(1, len(files) // 150)]
    vv, uu = np.mgrid[0:k["height"]:4, 0:k["width"]:4]
    near_middle = (vv > k["ppy"] + 20) & (np.abs(uu - k["ppx"]) < 150)      # below the horizon, in front of the ramp
    fits = []
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        z = img[::4, ::4] * cal["depth_scale"]
        m = near_middle & (z > 0.25) & (z < 3.0)
        if m.sum() < 400:
            continue
        P = np.stack([(uu[m] - k["ppx"]) / k["fx"] * z[m], (vv[m] - k["ppy"]) / k["fy"] * z[m], z[m]], axis=1)
        keep = np.ones(len(P), bool)
        for _ in range(4):                                                  # refit on the points close to the plane
            c = P[keep].mean(axis=0)
            n = np.linalg.eigh(np.cov((P[keep] - c).T))[1][:, 0]             # the direction the points are thinnest in
            n = n * np.sign(n[1])
            r = (P - c) @ n
            keep = np.abs(r) < max(0.01, 1.5 * np.median(np.abs(r[keep])))
        if keep.mean() > 0.6 and n[1] > 0.9:
            fits.append([*n, c @ n])
    if len(fits) < 10:
        return None
    fits = np.array(fits)
    n, d = np.median(fits[:, :3], axis=0), float(np.median(fits[:, 3]))
    n /= np.linalg.norm(n)
    ext = cal.get("depth_to_color")
    if ext:                                                                 # librealsense rotations are column-major
        R, t = np.array(ext["rotation"]).reshape(3, 3).T, np.array(ext["translation"])
        n = R @ n
        d = d + float(n @ t)
    c = cal["color"]
    return {"fx": c["fx"], "fy": c["fy"], "cx": c["ppx"], "cy": c["ppy"], "down": n.tolist(), "height_m": d,
            "source": f"plane fitted to {len(fits)} depth pictures, height spread {fits[:, 3].std() * 1000:.1f} mm"}


def pose_from_edges(lines, fx, fy, cx, cy, height_m, what):
    """Tilt from where the two edges meet (the horizon), for a camera with no depth. No roll."""
    both = lines.dropna(subset=["l_a", "r_a"])
    both = both[(both.l_b - both.r_b).abs() > 0.2]
    if len(both) < 20:
        return None
    v_meet = np.median((both.r_a - both.l_a) / (both.l_b - both.r_b))      # u = a + b v, for each edge
    down = np.array([0.0, 1.0, (cy - v_meet) / fy])
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "down": (down / np.linalg.norm(down)).tolist(),
            "height_m": height_m, "source": f"horizon from the sidewalk edges in {len(both)} pictures; {what}"}


# ---------- edges in the picture ----------

def fit_line(v, u):
    """u = a + b v, refitted without the points that are far off. Returns (a, b, rms px, points used)."""
    keep = np.ones(len(v), bool)
    for _ in range(3):
        b, a = np.polyfit(v[keep], u[keep], 1)
        r = u - (a + b * v)
        keep = np.abs(r) < max(2.0, 2.5 * 1.4826 * np.median(np.abs(r[keep])))
        if keep.sum() < MIN_EDGE_ROWS:
            return None
    b, a = np.polyfit(v[keep], u[keep], 1)
    return a, b, float(np.sqrt(np.mean((u[keep] - (a + b * v[keep])) ** 2))), int(keep.sum())


def find_edges(paved, top_row):
    """
    Follow the sidewalk up the picture from the bottom row and fit a line to each edge.
    paved: bool (H, W). Returns {"l_a", "l_b", "l_rms", "l_n", "r_a", ...} with NaN for an edge not found.
    """
    H, W = paved.shape
    lo, hi = W / 2 - 60, W / 2 + 60                     # the sidewalk under the camera, to start from
    pts = {"l": [], "r": []}
    for v in range(H - 1, max(int(np.ceil(top_row)), 0) - 1, -ROW_STEP):
        d = np.diff(np.concatenate([[0], paved[v].view(np.int8), [0]]))
        starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1      # runs of sidewalk in this row
        if not len(starts):
            continue
        overlap = np.minimum(ends, hi) - np.maximum(starts, lo)
        i = int(np.argmax(overlap))
        if overlap[i] < 0:
            continue
        lo, hi = starts[i], ends[i]
        if hi - lo < MIN_WIDTH_PX:
            break
        if lo > BORDER_PX:
            pts["l"].append((v, lo))
        if hi < W - 1 - BORDER_PX:
            pts["r"].append((v, hi))
    out = {}
    for side, p in pts.items():
        fit, why = None, "out of view" if not p else "too short"
        if len(p) >= MIN_EDGE_ROWS and p[0][0] - p[-1][0] >= MIN_EDGE_SPAN:
            p = np.array(p, float)
            fit, why = fit_line(p[:, 0], p[:, 1]), "not straight"
            if fit and fit[2] > MAX_EDGE_RMS_PX:
                fit = None
            if fit:
                fit, why = fit + (float(p[:, 0].min()), float(p[:, 0].max())), ""
        out[f"{side}_why"] = why
        for name, val in zip(("a", "b", "rms", "n", "v_far", "v_near"), fit or [np.nan] * 6):
            out[f"{side}_{name}"] = val
    return out


def edge_lines(run_dir, cam, paved, top_row):
    """find_edges for every labelled picture of a camera: a table indexed by frame number."""
    rows = []
    for f in sorted(glob.glob(os.path.join(run_dir, "labels", cam, "*.png"))):
        lab = cv2.imread(f, cv2.IMREAD_UNCHANGED)
        if lab is None:
            continue
        mask = np.isin(lab, paved)
        rows.append({"frame": int(os.path.basename(f)[:-4]), "paved_frac": float(mask.mean()), **find_edges(mask, top_row)})
    return pd.DataFrame(rows)


# ---------- edges on the ground ----------

def to_geometry(lines, ground, rear, known_width=None):
    # returns (table, the width used, the width these pictures themselves show, how many pictures show both edges)
    """Put each picture's edge lines on the ground and work out offset, heading and width."""
    g = lines[["frame", "paved_frac"]].copy()
    side = {}
    for s in ("l", "r"):
        a, b = lines[f"{s}_a"].to_numpy(), lines[f"{s}_b"].to_numpy()
        v1 = np.maximum(lines[f"{s}_v_far"].to_numpy(), ground.top_row())
        v2 = lines[f"{s}_v_near"].to_numpy()
        X1, Z1 = ground.to_ground(a + b * v1, v1)
        X2, Z2 = ground.to_ground(a + b * v2, v2)
        with np.errstate(divide="ignore", invalid="ignore"):
            slope = np.where(v2 - v1 >= MIN_EDGE_SPAN - 1, (X1 - X2) / (Z1 - Z2), np.nan)     # the edge: X = x0 + slope * Z
        side[s] = (X2 - slope * Z2, slope)
    (xl, sl), (xr, sr) = side["l"], side["r"]
    has_l, has_r = np.isfinite(xl), np.isfinite(xr)
    both = has_l & has_r
    cos = lambda s: 1 / np.sqrt(1 + s ** 2)
    width = np.where(both, (xr - xl) * cos((sl + sr) / 2), np.nan)
    own, n_both = (float(np.nanmedian(width)) if both.any() else np.nan), int(both.sum())
    usual = known_width or own
    both &= np.abs(width - usual) < WIDTH_TOLERANCE * usual        # two edges that aren't a sidewalk apart: not this sidewalk
    slope = np.where(both, (sl + sr) / 2, np.where(has_l, sl, sr))
    half = usual / 2 / cos(slope)
    centre = np.where(both, (xl + xr) / 2, np.where(has_l, xl + half, xr - half))   # the centre line's X beside the camera
    edges = np.where(both, 2, np.where(np.isfinite(centre), 1, 0))
    offset, heading = -centre * cos(slope), -np.arctan(slope)       # as this camera sees it: + = right, + = clockwise
    g["edges"] = edges
    g["offset_m"] = -offset if rear else offset                     # the rear camera looks backwards: its right is the ramp's left
    g["heading_rad"] = heading                                      # clockwise is clockwise whichever way you face
    g["width_m"] = np.where(both, width, np.nan)
    g["left_edge_m"], g["right_edge_m"] = (-xr, -xl) if rear else (xl, xr)
    g["edge_rms_px"] = np.fmax(lines.l_rms.to_numpy(), lines.r_rms.to_numpy())
    lw, rw = lines.l_why.to_numpy(), lines.r_why.to_numpy()
    worst = np.where((lw == "not straight") | (rw == "not straight"), "an edge isn't straight",
                     np.where((lw == "too short") | (rw == "too short"), "too little edge in view", "no edge in view"))
    g["why"] = np.where(edges > 0, "", np.where(has_l | has_r, "edges not a sidewalk's width apart", worst))
    return g, usual, own, n_both


def preview(run_dir, cam, lines, geom, ground, paved, path, n=12):
    """A sheet of pictures with the labels, fitted edges and result drawn on."""
    pick = lines.iloc[np.linspace(0, len(lines) - 1, min(n, len(lines))).astype(int)]
    tiles = []
    for _, ln in pick.iterrows():
        name = f"{int(ln.frame):06d}"
        img = cv2.imread(os.path.join(run_dir, cam, name + ".jpg"))
        lab = cv2.imread(os.path.join(run_dir, "labels", cam, name + ".png"), cv2.IMREAD_UNCHANGED)
        if img is None or lab is None:
            continue
        m = np.isin(lab, paved)
        img[m] = (0.65 * img[m] + 0.35 * np.array([0, 200, 0])).astype(np.uint8)
        hz = int(round(ground.horizon_row()))
        cv2.line(img, (0, hz), (img.shape[1], hz), (255, 255, 0), 1)
        for s in ("l", "r"):
            if np.isfinite(ln[f"{s}_a"]):
                p = [(int(round(ln[f"{s}_a"] + ln[f"{s}_b"] * v)), int(v)) for v in (ln[f"{s}_v_far"], ln[f"{s}_v_near"])]
                cv2.line(img, p[0], p[1], (0, 0, 255), 2)
        r = geom[geom.frame == ln.frame].iloc[0]
        text = "no result" if r.edges == 0 else f"off {r.offset_m * 100:+.0f} cm  head {np.degrees(r.heading_rad):+.1f} deg  edges {int(r.edges)}"
        cv2.putText(img, f"{name}  {text}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, f"{name}  {text}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        tiles.append(img)
    if tiles:
        tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 3)
        cv2.imwrite(path, np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]), [cv2.IMWRITE_JPEG_QUALITY, 85])


def report(cam, g, usual):
    ok = g[g.edges > 0]
    print(f"  {cam}: {len(g)} pictures: both edges {(g.edges == 2).mean() * 100:.0f}%, one {(g.edges == 1).mean() * 100:.0f}%, "
          f"no result {(g.edges == 0).mean() * 100:.0f}%")
    if len(ok):
        w = g.width_m.dropna()
        spread = f" (middle half of pictures within {w.quantile(.25):.2f}-{w.quantile(.75):.2f})" if len(w) else ""
        print(f"      sidewalk width {usual:.2f} m{spread}   "
              f"offset {ok.offset_m.quantile(.05) * 100:+.0f}..{ok.offset_m.quantile(.95) * 100:+.0f} cm   "
              f"heading {np.degrees(ok.heading_rad.quantile(.05)):+.1f}..{np.degrees(ok.heading_rad.quantile(.95)):+.1f} deg")
    if (g.edges == 0).any():
        print("      no result because: " + ", ".join(f"{why} {n / len(g) * 100:.0f}%" for why, n in g.why[g.edges == 0].value_counts().items()))


def process(run_dir, args):
    run_dir = run_dir.rstrip("/")
    name = os.path.basename(run_dir)
    classes_file = os.path.join(run_dir, "labels", "classes.json")
    if not os.path.isfile(classes_file):
        print(f"{name}: no labels (run label_sidewalk.py on it first)")
        return
    paved = paved_ids(json.load(open(classes_file))["id2label"], tuple(norm_name(w) for w in args.paved))
    if not paved:
        sys.exit(f"{name}: none of the label classes match {args.paved}")
    cal_file = os.path.join(run_dir, "calibration.json")
    cal = json.load(open(cal_file)) if os.path.isfile(cal_file) else None
    pose_file = args.pose or os.path.join(run_dir, "camera_pose.json")
    pose = json.load(open(pose_file)) if os.path.isfile(pose_file) else {}
    print(f"{name}:")
    geoms, front_width = {}, None
    for cam, out_name in (("color", "geometry.parquet"), ("rear", "geometry_rear.parquet")):
        if not glob.glob(os.path.join(run_dir, "labels", cam, "*.png")):
            continue
        key = "front" if cam == "color" else "rear"
        p = pose.get(key)
        if p is None and cam == "color" and cal:
            p = front_pose_from_depth(run_dir, cal)
        lines = None
        if p is None:                                   # no depth to go on: use the horizon the edges point at
            if cam == "color" and not cal:
                sys.exit(f"{name}: no calibration.json for the front camera")
            height = (args.height if cam == "color" else args.rear_height or (pose.get("front") or {}).get("height_m"))
            if not height:
                sys.exit(f"{name}: no usable depth, so the {key} camera's height can't be measured. Pass --height"
                         f"{'' if cam == 'color' else ' or --rear-height'}, or --pose with the camera_pose.json of a run that has depth.")
            lines = edge_lines(run_dir, cam, paved, top_row=0)
            if cam == "color":
                k = (cal["color"]["fx"], cal["color"]["fy"], cal["color"]["ppx"], cal["color"]["ppy"])
                what = "height as given"
            else:
                sample = cv2.imread(sorted(glob.glob(os.path.join(run_dir, cam, "*.jpg")))[0])
                k = (args.rear_fx, args.rear_fx, sample.shape[1] / 2, sample.shape[0] / 2)
                what = f"focal length assumed {args.rear_fx:.0f} px, height {'as given' if args.rear_height else 'taken from the front camera'}"
            p = pose_from_edges(lines, *k, height, what)
            if p is None:
                print(f"  {cam}: too few pictures show both edges to find the horizon; skipped")
                continue
        if cam == "color" and args.height:
            p = {**p, "height_m": args.height, "source": p["source"].split("; height")[0] + "; height as given"}
        if cam == "rear" and args.rear_height:
            p = {**p, "height_m": args.rear_height, "height_from": "given"}
        pose[key] = p
        ground = Ground(**p)
        print(f"  {key} camera: {describe(p)}")
        lines = edge_lines(run_dir, cam, paved, ground.top_row())       # again, now knowing where the horizon is
        # It is one sidewalk: the rear camera uses the width the front camera measured, which is the better measurement.
        width = args.width or (front_width if cam == "rear" else None)
        g, usual, own, n_both = to_geometry(lines, ground, rear=cam == "rear", known_width=width)
        if cam == "rear" and width and n_both >= 20 and p.get("height_from") != "given" and abs(own / width - 1) > 0.02:
            # Everything on the ground scales with the camera's height, so the height that makes the
            # sidewalk as wide here as the front camera sees it is the rear camera's real height.
            p["height_m"] *= width / own
            p["height_from"] = "width"
            p["source"] = p["source"].split(", height")[0] + f", height set so the sidewalk comes out {width:.2f} m wide like in the front camera"
            ground = Ground(**p)
            print(f"  {key} camera, corrected: {describe(p)}")
            lines = edge_lines(run_dir, cam, paved, ground.top_row())
            g, usual, own, n_both = to_geometry(lines, ground, rear=True, known_width=width)
        if cam == "color":
            front_width = usual if np.isfinite(usual) else None
        times = os.path.join(run_dir, "frames.parquet" if cam == "color" else "rear_frames.parquet")
        if os.path.isfile(times):
            g = g.merge(pd.read_parquet(times, columns=["frame", "t_ns", "t"]), on="frame", how="left")
        g.to_parquet(os.path.join(run_dir, out_name), index=False)
        preview(run_dir, cam, lines, g, ground, paved, os.path.join(run_dir, f"geometry_preview_{cam}.jpg"))
        report(cam, g, usual)
        geoms[cam] = g
    with open(os.path.join(run_dir, "camera_pose.json"), "w") as f:
        json.dump(pose, f, indent=1)
    if len(geoms) == 2 and "t_ns" in geoms["color"] and "t_ns" in geoms["rear"]:      # the two cameras should agree
        a = geoms["color"][geoms["color"].edges > 0].sort_values("t_ns")
        b = geoms["rear"][geoms["rear"].edges > 0].sort_values("t_ns")
        if len(a) > 20 and len(b) > 20:
            m = pd.merge_asof(a, b, on="t_ns", tolerance=40_000_000, direction="nearest", suffixes=("", "_rear")).dropna(subset=["offset_m_rear"])
            if len(m) > 20:
                print(f"  front vs rear over {len(m)} moments: offset differs by {np.median(np.abs(m.offset_m - m.offset_m_rear)) * 100:.0f} cm (median), "
                      f"heading by {np.degrees(np.median(np.abs(m.heading_rad - m.heading_rad_rear))):.1f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="exported run folders that label_sidewalk.py has been run on")
    ap.add_argument("--pose", help="a camera_pose.json to use instead of working the pose out")
    ap.add_argument("--height", type=float, help="front camera height above the ground, metres")
    ap.add_argument("--rear-height", type=float, help="rear camera height above the ground, metres")
    ap.add_argument("--rear-fx", type=float, default=615.0, help="rear camera focal length, pixels")
    ap.add_argument("--width", type=float, help="sidewalk width, metres, instead of measuring it from the pictures")
    ap.add_argument("--paved", nargs="+", default=list(PAVED_WORDS), help="class-name words that count as sidewalk")
    args = ap.parse_args()
    for run in args.runs:
        process(os.path.expanduser(run), args)


if __name__ == "__main__":
    main()
