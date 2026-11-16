#!/usr/bin/env -S uv run --script
# /// script
# requires-python = "==3.10.*"
# dependencies = [
#     "pycolmap-cuda12==4.2.1",
#     "numpy",
#     "opencv-python-headless",
# ]
# ///
"""
Build a 3D map (camera poses + a Gaussian splat) of a place from walk-through
video, for localize_run.py and view_run_3d.py to place the ramp's runs in.

  ./build_map.py NAME                 build the map in /scratch/ramp-maps/NAME
        --steps frames,sfm,splat    only these steps (default: all three)
        --force                     redo steps whose output is already there
        --fps N                     frames kept per second of video (default 5)
        --width N                   frame width in pixels (default 1920; never upscaled)
        --focal-mm N                the lens's focal length (full-frame mm), as a starting guess
        --mapper global|incremental how the poses are solved (default global: much faster)
        --iterations N              splat training steps (default 30000)
        --maps-dir DIR              where maps live (default /scratch/ramp-maps)
        --verbose                   show COLMAP's own log

Put the camera's video files in /scratch/ramp-maps/NAME/video/ first. All of
them must come from the same camera at the same zoom, since they share one
camera model. The steps, each writing one folder next to video/:

  frames/   the sharpest frame of every 1/fps second of each video (ffmpeg)
  sfm/      where the camera was for each frame (COLMAP on the GPU: SIFT,
            sequential matching with loop detection, then the mapper)
  splat/    the Gaussian splat (nerfstudio's splatfacto), exported as
            splat.ply with the transform that takes sfm/ coordinates to it

COLMAP comes from the pycolmap-cuda12 wheel, which uv installs into a
throwaway environment; the splat is trained by the system's ns-train.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("SSL_CERT_FILE", "/etc/ssl/certs/ca-certificates.crt")   # for COLMAP's vocabulary tree download

import cv2
import numpy as np

MAPS_DIR = Path("/scratch/ramp-maps")
VIDEO_EXT = {".mp4", ".mov", ".mxf", ".mkv", ".m4v"}
FULL_FRAME_MM = 36.0            # sensor width the --focal-mm guess is relative to


def log(msg):
    print(msg, flush=True)


# ---------- 1. frames ----------

def probe(video):
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "stream=width,height,avg_frame_rate,nb_frames:stream_side_data=rotation", "-of", "json", str(video)],
                         capture_output=True, text=True, check=True).stdout
    s = json.loads(out)["streams"][0]
    num, den = s["avg_frame_rate"].split("/")
    w, h = int(s["width"]), int(s["height"])
    turned = any(abs(int(float(d.get("rotation", 0)))) in (90, 270) for d in s.get("side_data_list", []))
    return (h, w) if turned else (w, h), float(num) / float(den)


def sharp_frames(video, out_dir, width, fps, hwaccel):
    """Keep the sharpest frame of every window of 1/fps seconds. Returns how many were kept, or None if ffmpeg failed."""
    (w, h), src_fps = probe(video)
    width = min(width, w)
    height = int(round(h * width / w / 2)) * 2
    window = max(1, int(round(src_fps / fps)))
    cmd = ["ffmpeg", "-v", "error", "-nostdin", *(["-hwaccel", "cuda"] if hwaccel else []), "-i", str(video),
           "-vf", f"scale={width}:{height}:flags=area", "-pix_fmt", "bgr24", "-f", "rawvideo", "-"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    size, seen, kept, best = width * height * 3, 0, [], None
    def flush():
        if best is not None:
            path = out_dir / f"{video.stem}_{best[1]:06d}.jpg"
            cv2.imwrite(str(path), best[2], [cv2.IMWRITE_JPEG_QUALITY, 95])
            kept.append(path)
    while True:
        buf = proc.stdout.read(size)
        if len(buf) < size:
            break
        img = np.frombuffer(buf, np.uint8).reshape(height, width, 3)
        small = cv2.cvtColor(img[::2, ::2], cv2.COLOR_BGR2GRAY)
        score = cv2.Laplacian(small, cv2.CV_32F).var()
        if best is None or score > best[0]:
            best = (score, seen, img.copy())
        seen += 1
        if seen % window == 0:
            flush()
            best = None
    flush()
    err = proc.stderr.read().decode(errors="replace").strip()
    if proc.wait() != 0 or seen == 0:
        for p in kept:
            p.unlink()
        return None, err
    log(f"   {video.name}: {w}x{h} at {src_fps:.4g} fps -> kept {len(kept)} of {seen} frames at {width}x{height}"
        f"{' (decoded on the GPU)' if hwaccel else ''}")
    return len(kept), err


def step_frames(args, d):
    videos = sorted(p for p in (d / "video").iterdir() if p.suffix.lower() in VIDEO_EXT) if (d / "video").is_dir() else []
    if not videos:
        sys.exit(f"no video files in {d / 'video'}: copy the camera's recordings there first")
    out = d / "frames"
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    log(f"1. frames: the sharpest frame of every 1/{args.fps:g} s, from {len(videos)} video(s)")
    total = 0
    for v in videos:
        n, err = sharp_frames(v, out, args.width, args.fps, hwaccel=True)
        if n is None:                                   # e.g. 4:2:2 video, which the GPU can't decode
            n, err = sharp_frames(v, out, args.width, args.fps, hwaccel=False)
        if n is None:
            sys.exit(f"ffmpeg could not read {v}:\n{err}")
        total += n
    log(f"   {total} frames in {out}")


# ---------- 2. camera poses ----------

def step_sfm(args, d):
    import pycolmap
    if not pycolmap.has_cuda:
        sys.exit("this pycolmap was built without CUDA")
    if not args.verbose:
        pycolmap.logging.minloglevel = 2
    frames = sorted(p.name for p in (d / "frames").glob("*.jpg"))
    if len(frames) < 10:
        sys.exit(f"only {len(frames)} frames in {d / 'frames'}: run the frames step first")
    out = d / "sfm"
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir()
    db = out / "database.db"
    log(f"2. sfm: solving where the camera was for {len(frames)} frames (GPU {os.popen('nvidia-smi --query-gpu=name --format=csv,noheader').read().strip()})")

    reader = pycolmap.ImageReaderOptions(camera_model="OPENCV")
    if args.focal_mm:
        img = cv2.imread(str(d / "frames" / frames[0]))
        h, w = img.shape[:2]
        f = args.focal_mm / FULL_FRAME_MM * w
        reader.camera_params = f"{f},{f},{w / 2},{h / 2},0,0,0,0"
    t = time.time()
    pycolmap.extract_features(db, d / "frames", camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader,
                              device=pycolmap.Device.cuda)
    log(f"   features: {time.time() - t:.0f} s")

    t = time.time()
    pairing = pycolmap.SequentialPairingOptions()
    pairing.overlap = 12
    pairing.loop_detection = True                      # ties the separate passes and videos together
    pairing.loop_detection_period = 5
    pairing.loop_detection_num_images = 40
    pycolmap.match_sequential(db, pairing_options=pairing, device=pycolmap.Device.cuda)
    with pycolmap.Database.open(db) as base:
        log(f"   matching: {time.time() - t:.0f} s, {base.num_verified_image_pairs()} image pairs")

    t = time.time()
    if args.mapper == "global":
        recs = pycolmap.global_mapping(db, d / "frames", out)
    else:
        recs = pycolmap.incremental_mapping(db, d / "frames", out)
    if not recs:
        sys.exit("the mapper could not reconstruct anything: the frames may overlap too little")
    best = max(recs, key=lambda k: recs[k].num_reg_images())
    rec = recs[best]
    for k in recs:                                      # keep only the biggest piece, as sfm/0
        if (out / str(k)).is_dir() and k != best:
            shutil.rmtree(out / str(k))
    if best != 0:
        (out / str(best)).rename(out / "0")
    n = rec.num_reg_images()
    log(f"   mapper ({args.mapper}): {time.time() - t:.0f} s, placed {n} of {len(frames)} frames, "
        f"{rec.num_points3D()} points, reprojection error {rec.compute_mean_reprojection_error():.2f} px")
    cam = next(iter(rec.cameras.values()))
    log(f"   camera: {cam.width}x{cam.height}, focal length {cam.params[0]:.0f} px"
        f" = {cam.params[0] / cam.width * FULL_FRAME_MM:.1f} mm full-frame equivalent")
    if len(recs) > 1 or n < 0.9 * len(frames):
        log(f"   WARNING: the map came out in {len(recs)} piece(s) and {len(frames) - n} frames are not in the one kept."
            " Parts of the walk-through did not connect: film a pass that links them, or try --mapper incremental.")
    json.dump({"frames": len(frames), "placed": n, "points": rec.num_points3D(), "mapper": args.mapper,
               "reprojection_px": rec.compute_mean_reprojection_error()}, open(out / "summary.json", "w"), indent=1)


# ---------- 3. splat ----------

def step_splat(args, d):
    if not (d / "sfm" / "0" / "images.bin").is_file():
        sys.exit(f"no camera poses in {d / 'sfm' / '0'}: run the sfm step first")
    for tool in ("ns-train", "ns-export"):
        if not shutil.which(tool):
            sys.exit(f"{tool} is not on the PATH: the splat step needs nerfstudio")
    out = d / "splat"
    shutil.rmtree(out, ignore_errors=True)
    log(f"3. splat: training for {args.iterations} steps (watch it at http://localhost:7007)")
    env = {k: v for k, v in os.environ.items() if k not in ("VIRTUAL_ENV", "PYTHONPATH", "PYTHONHOME")}   # ns-train is the system's, not uv's
    env["PATH"] = os.pathsep.join(p for p in env["PATH"].split(os.pathsep) if "/uv/" not in p and ".venv" not in p)
    t = time.time()
    subprocess.run(["ns-train", "splatfacto", "--data", str(d), "--output-dir", str(out),
                    "--experiment-name", "map", "--timestamp", "run", "--max-num-iterations", str(args.iterations),
                    "--vis", "viewer", "--viewer.quit-on-train-completion", "True",
                    "--pipeline.datamanager.cache-images", "cpu",
                    "colmap", "--colmap-path", "sfm/0", "--images-path", "frames", "--downscale-factor", "1",
                    "--eval-mode", "all"], check=True, env=env, stdout=None if args.verbose else subprocess.DEVNULL)
    run = out / "map" / "splatfacto" / "run"
    subprocess.run(["ns-export", "gaussian-splat", "--load-config", str(run / "config.yml"), "--output-dir", str(out)],
                   check=True, env=env, stdout=None if args.verbose else subprocess.DEVNULL)
    shutil.copy(run / "dataparser_transforms.json", out / "dataparser_transforms.json")
    log(f"   {time.time() - t:.0f} s -> {out / 'splat.ply'} ({(out / 'splat.ply').stat().st_size / 1e6:.0f} MB)")


STEPS = {"frames": (step_frames, lambda d: any((d / "frames").glob("*.jpg"))),
         "sfm": (step_sfm, lambda d: (d / "sfm" / "0" / "images.bin").is_file()),
         "splat": (step_splat, lambda d: (d / "splat" / "splat.ply").is_file())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--steps", default="frames,sfm,splat")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--focal-mm", type=float)
    ap.add_argument("--mapper", choices=("global", "incremental"), default="global")
    ap.add_argument("--iterations", type=int, default=30000)
    ap.add_argument("--maps-dir", type=Path, default=MAPS_DIR)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    d = args.maps_dir / args.name
    (d / "video").mkdir(parents=True, exist_ok=True)
    steps = args.steps.split(",")
    if bad := [s for s in steps if s not in STEPS]:
        sys.exit(f"unknown step(s) {bad}: choose from {list(STEPS)}")
    redo = args.force
    for name in STEPS:                                  # in order; a redone step makes the later ones stale
        if name not in steps:
            continue
        run, done = STEPS[name]
        if done(d) and not redo:
            log(f"{name}: already there, skipping (--force to redo)")
            continue
        run(args, d)
        redo = True
    log(f"map: {d}")


if __name__ == "__main__":
    main()
