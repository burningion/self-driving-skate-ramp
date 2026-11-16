#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "viser>=1.1,<2",
#     "numpy",
#     "pandas",
#     "pyarrow",
#     "scipy",
#     "opencv-python-headless",
# ]
# ///
"""
Watch a run in 3D: the map's Gaussian splat, with the ramp's camera and a model
of its board moving through it along the path localize_run.py found. Opens a web page (Chrome works
best); it is reachable over Tailscale like view_run.sh's.

  ./view_run_3d.py RUN                RUN is a folder in ~/ramp-export with a trajectory.parquet
        --map NAME        map to show (default: the one the run was localized in)
        --port N          port for the page (default 8080)
        --max-splats N    show at most this many Gaussians, the most opaque first (default 1500000)
        --maps-dir DIR    where maps live (default /scratch/ramp-maps)
        --export-dir DIR  where exported runs live (default ~/ramp-export, then ~/Development/ramp-export)

On the page: drag to orbit, scroll to zoom, right-drag to pan. The panel has
the time slider, play and speed, "follow" to ride behind the ramp, and
"board" to show or hide the model of the plywood board it rides on. The model is drawn from the
sizes in BOARD below, hung under the camera at the height and tilt in the run's
camera_pose.json (from sidewalk_geometry.py). The camera outline shows what the front camera saw at that moment. The path is
cyan where the pictures matched the map and orange where it is dead-reckoned
from the wheels; stretches with no position (outside the map) are not drawn.
"""
import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import viser
from scipy.spatial.transform import Rotation

MAPS_DIR = Path("/scratch/ramp-maps")
EXPORT_DIRS = [Path(p).expanduser() for p in ("~/ramp-export", "~/Development/ramp-export")]
SH_C0 = 0.28209479177387814
M_PER_ERPM = 3.9e-4             # the same number as autosteer.py's m_per_erpm
FPS = 15                        # how often the page is updated while playing
BOARD = dict(                   # the plywood board the ramp rides on, in metres (sizes by eye, but for the wheelbase)
    length=1.05, width=0.45,    # the bottom sheet of plywood
    deck_top=0.13, sheet=0.018, # its top, off the ground, and how thick each sheet is
    wheelbase=0.84,             # axle to axle (33"), as in autosteer.py; the front camera is over the front axle
    wheel_d=0.09, wheel_w=0.05,
    track=0.40,                 # between the middles of the left and right wheels
    camera_m=0.19,              # the front camera's height, when the run has no camera_pose.json
)


def load_splat(path, limit):
    """A nerfstudio / 3DGS .ply -> centres, covariances, colours, opacities."""
    with open(path, "rb") as f:
        names, count = [], 0
        while (line := f.readline().decode().strip()) != "end_header":
            if line.startswith("element vertex"):
                count = int(line.split()[-1])
            elif line.startswith("property"):
                if line.split()[1] != "float":
                    sys.exit(f"{path}: expected only float properties, got '{line}'")
                names.append(line.split()[-1])
        data = np.frombuffer(f.read(count * 4 * len(names)), np.float32).reshape(count, len(names))
    col = {n: i for i, n in enumerate(names)}
    get = lambda *keys: data[:, [col[k] for k in keys]]
    opacity = 1 / (1 + np.exp(-get("opacity")))
    keep = np.flatnonzero((opacity[:, 0] > 0.02) & np.isfinite(data).all(axis=1))
    if len(keep) > limit:
        keep = keep[np.argsort(-opacity[keep, 0])[:limit]]
    R = Rotation.from_quat(get("rot_1", "rot_2", "rot_3", "rot_0")[keep]).as_matrix().astype(np.float32)   # stored w x y z
    RS = R * np.exp(get("scale_0", "scale_1", "scale_2")[keep])[:, None, :]
    rgb = np.clip(0.5 + SH_C0 * get("f_dc_0", "f_dc_1", "f_dc_2")[keep], 0, 1)
    return get("x", "y", "z")[keep], RS @ RS.transpose(0, 2, 1), rgb, opacity[keep]


def box(lo, hi):
    v = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], float)
    f = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
    return v, np.array(f)


def prism(outline, lo, hi, axis):
    """A flat outline (n x 2) pulled out from lo to hi along `axis`, with both ends closed."""
    n = len(outline)
    v = np.concatenate([np.insert(outline, axis, h, axis=1) for h in (lo, hi)] +
                       [np.insert(outline.mean(axis=0)[None], axis, h, axis=1) for h in (lo, hi)])
    i, j = np.arange(n), (np.arange(n) + 1) % n
    f = np.concatenate([np.stack([i, j, j + n], 1), np.stack([i, j + n, i + n], 1),
                        np.stack([i, j, np.full(n, 2 * n)], 1), np.stack([i + n, j + n, np.full(n, 2 * n + 1)], 1)])
    return v, f


def join(parts):
    v, f, n = [], [], 0
    for pv, pf in parts:
        v.append(pv), f.append(pf + n)
        n += len(pv)
    return np.concatenate(v), np.concatenate(f)


def board_parts(camera_m):
    """The ramp's board as (name, colour, vertices, faces), in metres: x right, y up from the ground,
    z forward, with the front camera (and the front axle) above the origin."""
    b = BOARD
    half, top = b["width"] / 2, b["deck_top"]
    front = (b["length"] - b["wheelbase"]) / 2
    back = front - b["length"]
    sheet = box([-half, top - b["sheet"], back], [half, top, front])
    upper = box([-half + 0.05, top, back + 0.10], [half - 0.05, top + b["sheet"], front - 0.12])        # the smaller sheet on top
    a = np.linspace(0, 2 * np.pi, 25)[:-1]
    r = b["wheel_d"] / 2
    tyre = r * np.stack([np.cos(a), np.sin(a)], 1) + [r, 0]                          # (up, forward)
    wheels, trucks = [], []
    for z in (0.0, -b["wheelbase"]):
        for side in (-1, 1):
            x = side * b["track"] / 2
            wheels.append(prism(tyre + [0, z], x - b["wheel_w"] / 2, x + b["wheel_w"] / 2, axis=0))
        trucks.append(box([-b["track"] / 2, r - 0.008, z - 0.008], [b["track"] / 2, r + 0.008, z + 0.008]))    # axle
        trucks.append(box([-0.04, r, z - 0.03], [0.04, top - b["sheet"], z + 0.03]))                           # up to the board
    gear = box([-0.09, top + b["sheet"], back + 0.30], [0.09, top + b["sheet"] + 0.07, -0.22])                 # battery and electronics
    camera = [box([-0.062, camera_m - 0.0145, -0.026], [0.062, camera_m + 0.0145, 0]),                         # a RealSense D455
              box([-0.015, top, -0.024], [0.015, camera_m - 0.0145, -0.006])]
    return [("sheet", (235, 232, 222), *sheet), ("upper", (205, 170, 115), *upper), ("wheels", (35, 35, 38), *join(wheels)),
            ("trucks", (150, 155, 160), *join(trucks)), ("gear", (25, 25, 28), *gear), ("camera", (225, 225, 230), *join(camera))]


def under_camera(points, down, camera_m):
    """Points from board_parts -> the front camera's own axes (x right, y down, z forward)."""
    down = np.asarray(down, float) / np.linalg.norm(down)
    ahead = np.array([0, 0, 1.0]) - down[2] * down
    ahead /= np.linalg.norm(ahead)
    return np.outer(points[:, 0], np.cross(down, ahead)) + np.outer(camera_m - points[:, 1], down) + np.outer(points[:, 2], ahead)


def addresses(port):
    hosts = ["localhost"]
    try:
        hosts.insert(0, subprocess.run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=3).stdout.split()[0])
    except Exception:
        hosts.insert(0, socket.gethostname())
    return [f"http://{h}:{port}" for h in hosts]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--map")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--max-splats", type=int, default=1_500_000)
    ap.add_argument("--maps-dir", type=Path, default=MAPS_DIR)
    ap.add_argument("--export-dir", type=Path)
    args = ap.parse_args()

    places = [Path(args.run)] + [d / args.run for d in ([args.export_dir] if args.export_dir else EXPORT_DIRS)]
    run_dir = next((p for p in places if (p / "trajectory.parquet").is_file()), None)
    if run_dir is None:
        sys.exit(f"no trajectory.parquet for {args.run}: run localize_run.py {args.run} --map NAME first")
    info = json.load(open(run_dir / "trajectory.json"))
    splat_dir = args.maps_dir / (args.map or info["map"]) / "splat"
    if not (splat_dir / "splat.ply").is_file():
        sys.exit(f"no splat at {splat_dir}: run build_map.py {args.map or info['map']} --steps splat")

    # the path, taken from the map's sfm coordinates into the splat's
    tf = json.load(open(splat_dir / "dataparser_transforms.json"))
    T, zoom = np.array(tf["transform"]), tf["scale"]
    tr = pd.read_parquet(run_dir / "trajectory.parquet")
    frames = pd.read_parquet(run_dir / "frames.parquet")
    t = tr.t.to_numpy() - tr.t.iloc[0]
    pos = zoom * (tr[["x", "y", "z"]].to_numpy() @ T[:, :3].T + T[:, 3])
    rot = Rotation.from_matrix(T[:, :3]) * Rotation.from_quat(tr[["qx", "qy", "qz", "qw"]].to_numpy())
    wxyz = rot.as_quat()[:, [3, 0, 1, 2]]
    forward = rot.apply([0, 0, 1])
    known = np.isfinite(pos).all(axis=1)
    if not known.any():
        sys.exit("the trajectory has no positions at all: nothing in this run matched the map")
    metre = zoom * info["units_per_m"] if np.isfinite(info.get("units_per_m") or np.nan) else np.ptp(pos[known], axis=0).max() / 20
    speed = frames[["vesc_erpm_back", "vesc_erpm_front"]].mean(axis=1).fillna(0).to_numpy() * M_PER_ERPM
    cal = run_dir / "calibration.json"
    c = json.load(open(cal))["color"] if cal.is_file() else {"width": 640, "height": 480, "fy": 383.0}
    mount = json.load(open(run_dir / "camera_pose.json"))["front"] if (run_dir / "camera_pose.json").is_file() else {}
    down, camera_m = mount.get("down", [0, 1, 0]), mount.get("height_m", BOARD["camera_m"])

    print(f"loading {splat_dir / 'splat.ply'} ...", flush=True)
    centres, covariances, rgbs, opacities = load_splat(splat_dir / "splat.ply", args.max_splats)

    server = viser.ViserServer(port=args.port, verbose=False)
    server.scene.set_up_direction("+z")
    server.scene.add_gaussian_splats("/map", centres, covariances, rgbs, opacities)
    seg = np.flatnonzero(known[:-1] & known[1:])
    dead = (tr.source.to_numpy()[seg] == "odom")[:, None, None]
    server.scene.add_line_segments("/path", np.stack([pos[seg], pos[seg + 1]], axis=1),
                                   np.where(dead, [255, 150, 0], [0, 220, 255]).repeat(2, axis=1).astype(np.uint8),
                                   thickness=3, thickness_units="screen")

    def picture(i):
        img = cv2.imread(str(run_dir / "color" / f"{tr.frame.iloc[i]:06d}.jpg"))
        return None if img is None else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    first = int(np.flatnonzero(known)[0])
    ramp = server.scene.add_camera_frustum("/ramp", fov=2 * np.arctan(c["height"] / 2 / c["fy"]), aspect=c["width"] / c["height"],
                                           scale=0.4 * metre, thickness=3, thickness_units="screen", color=(255, 60, 60),
                                           image=picture(first), format="jpeg", wxyz=wxyz[first], position=pos[first])

    board = server.scene.add_frame("/board", show_axes=False, wxyz=wxyz[first], position=pos[first])
    for name, colour, points, faces in board_parts(camera_m):
        server.scene.add_mesh_simple(f"/board/{name}", metre * under_camera(points, down, camera_m), faces, color=colour, side="double")

    with server.gui.add_folder(run_dir.name):
        clock = server.gui.add_slider("time (s)", 0.0, float(t[-1]), 0.1, float(t[first]))
        playing = server.gui.add_checkbox("play", True)
        rate = server.gui.add_slider("speed", 0.25, 4.0, 0.25, 1.0)
        follow = server.gui.add_checkbox("follow", False)
        show_board = server.gui.add_checkbox("board", True)
        readout = server.gui.add_markdown("")

    def chase(client, i):
        client.camera.position = pos[i] - 2.5 * metre * forward[i] + np.array([0, 0, 1.2 * metre])
        client.camera.look_at = pos[i] + 1.0 * metre * forward[i]

    @server.on_client_connect
    def _(client):
        chase(client, shown if shown is not None else first)

    for url in addresses(args.port):
        print(f"open:  {url}")
    print("Ctrl-C to stop", flush=True)

    @show_board.on_update
    def _(_event):
        board.visible = show_board.value and bool(known[shown if shown is not None else first])

    shown = None
    while True:
        tick = time.time()
        if playing.value:
            clock.value = (clock.value + rate.value / FPS) % float(t[-1])
        i = int(np.clip(np.searchsorted(t, clock.value), 0, len(t) - 1))
        if i != shown:
            shown = i
            row = frames.iloc[i]
            ramp.visible = bool(known[i])
            board.visible = show_board.value and bool(known[i])
            if known[i]:
                ramp.position, ramp.wxyz = pos[i], wxyz[i]
                board.position, board.wxyz = pos[i], wxyz[i]
                if (img := picture(i)) is not None:
                    ramp.image = img
                if follow.value:
                    for client in server.get_clients().values():
                        chase(client, i)
            where = {"fix": "matched to the map", "interp": "between matches", "odom": "dead-reckoned from the wheels",
                     "none": "**no position** (outside the map)"}[tr.source.iloc[i]]
            readout.content = (f"frame {tr.frame.iloc[i]} &nbsp; {row.get('state', '')}\n\n"
                               f"{speed[i]:+.2f} m/s &nbsp; steer {row.get('steer', float('nan')):+.2f}\n\n{where}")
        time.sleep(max(0.0, 1 / FPS - (time.time() - tick)))


if __name__ == "__main__":
    main()
