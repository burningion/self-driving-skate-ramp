#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "rerun-sdk",
#     "pyarrow",
#     "numpy",
#     "opencv-python-headless",
# ]
# ///
"""Extract sharp frames from a Rerun .rrd recording for COLMAP / nerfstudio.

Step 1 - see what's in the recording (entity paths, timelines):
    uv run extract_frames_rrd.py run.rrd --list

Step 2 - extract frames from one image entity:
    uv run extract_frames_rrd.py run.rrd frames/ --entity /camera/color --window 10

It splits the image stream into windows of N frames and keeps the sharpest
frame of each window (variance of the Laplacian), which drops most blurry
frames. Handles both raw images (rr.Image) and compressed ones
(rr.EncodedImage, e.g. JPEG/PNG). Video streams (rr.VideoStream /
rr.AssetVideo) are not handled; for those, decode the video with ffmpeg.

Dependencies are declared inline above, so `uv run` installs them into a
throwaway environment automatically; nothing touches your nerfstudio venv.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


# ---------------------------------------------------------------- loading
def load_recording(path: Path):
    """Load an .rrd with whichever API this rerun-sdk version offers."""
    import rerun as rr
    try:  # rerun >= 0.32
        from rerun.experimental import RrdReader  # type: ignore
        return RrdReader(str(path)).store()
    except Exception:
        pass
    if hasattr(rr, "recording") and hasattr(rr.recording, "load_recording"):
        return rr.recording.load_recording(str(path))
    return rr.dataframe.load_recording(str(path))


def col_entity(desc) -> str:
    return str(getattr(desc, "entity_path", ""))


def col_component(desc) -> str:
    for attr in ("component", "component_name", "name"):
        v = getattr(desc, attr, None)
        if v:
            return str(v)
    return str(desc)


def col_name(desc) -> str:
    """Name usable in view.select()."""
    v = getattr(desc, "name", None)
    if v:
        return str(v)
    return f"{col_entity(desc)}:{col_component(desc)}"


def list_contents(rec) -> None:
    schema = rec.schema()
    print("Timelines (use with --timeline):")
    for idx in schema.index_columns():
        print("   ", getattr(idx, "name", idx))
    print("\nComponent columns:")
    for c in schema.component_columns():
        print(f"    {col_entity(c):40s} {col_component(c)}")


# ---------------------------------------------------------------- decoding
COLOR_MODELS = {1: "L", 2: "RGB", 3: "RGBA", 4: "BGR", 5: "BGRA"}


def first_blob(cell) -> np.ndarray | None:
    """A component cell is a list of blobs; return the first as uint8 array."""
    if cell is None or not cell.is_valid:
        return None
    inner = cell.values
    if len(inner) == 0:
        return None
    blob = inner[0]
    if hasattr(blob, "values"):          # list<uint8>
        return blob.values.to_numpy(zero_copy_only=False).astype(np.uint8)
    return np.frombuffer(blob.as_py(), dtype=np.uint8)


def first_value(cell):
    if cell is None or not cell.is_valid or len(cell.values) == 0:
        return None
    return cell.values[0].as_py()


def decode_raw(buf: np.ndarray, fmt: dict) -> np.ndarray:
    w, h = int(fmt["width"]), int(fmt["height"])
    ch = buf.size // (w * h)
    img = buf[: w * h * ch].reshape(h, w, ch) if ch > 1 else buf.reshape(h, w)
    model = COLOR_MODELS.get(fmt.get("color_model") or 0)
    if model is None:  # not stated: guess from channel count, assume RGB order
        model = {1: "L", 3: "RGB", 4: "RGBA"}.get(ch, "L")
    if model == "RGB":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif model == "RGBA":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif model == "BGRA":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def sharpness(img: np.ndarray) -> float:
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ---------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("rrd", type=Path)
    ap.add_argument("out", type=Path, nargs="?")
    ap.add_argument("--list", action="store_true", help="print entities and exit")
    ap.add_argument("--entity", help="image entity path, e.g. /camera/color")
    ap.add_argument("--timeline", default=None,
                    help="timeline to order frames by (default: first listed)")
    ap.add_argument("--window", type=int, default=10,
                    help="keep the sharpest frame out of every N (default 10)")
    args = ap.parse_args()

    rec = load_recording(args.rrd)
    if args.list:
        list_contents(rec)
        return
    if not (args.out and args.entity):
        sys.exit("need an output folder and --entity (run with --list first)")

    schema = rec.schema()
    timeline = args.timeline or getattr(schema.index_columns()[0], "name")
    entity = "/" + args.entity.strip("/")

    cols = {}
    for c in schema.component_columns():
        if col_entity(c).rstrip("/") != entity:
            continue
        comp = col_component(c).lower()
        if comp.endswith("blob"):
            cols["blob"] = col_name(c)
        elif comp.endswith("buffer"):
            cols["buffer"] = col_name(c)
        elif comp.endswith("format") and "image" in comp:
            cols["format"] = col_name(c)
    if "blob" not in cols and not {"buffer", "format"} <= cols.keys():
        sys.exit(f"no image data found at {entity}. Run --list to check the path.")
    mode = "encoded" if "blob" in cols else "raw"
    print(f"reading {mode} images from {entity} along timeline '{timeline}'")

    view = rec.view(index=timeline, contents=entity)
    wanted = [cols["blob"]] if mode == "encoded" else [cols["buffer"], cols["format"]]
    reader = view.select(timeline, *wanted)

    args.out.mkdir(parents=True, exist_ok=True)
    best, best_score, best_t = None, -1.0, None
    seen = kept = 0
    rows = []
    last_fmt = None

    def flush():
        nonlocal best, best_score, kept
        if best is not None:
            cv2.imwrite(str(args.out / f"frame_{kept:05d}.png"), best)
            rows.append((kept, best_t, best_score))
            kept += 1
        best, best_score = None, -1.0

    for batch in reader:
        names = batch.schema.names
        tcol = batch.column(names.index(timeline) if timeline in names else 0)
        data = [batch.column(names.index(n)) for n in wanted]
        for i in range(batch.num_rows):
            if mode == "encoded":
                buf = first_blob(data[0][i])
                if buf is None:
                    continue
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is None:
                    continue
            else:
                buf = first_blob(data[0][i])
                fmt = first_value(data[1][i]) or last_fmt  # format may be logged once
                if buf is None or fmt is None:
                    continue
                last_fmt = fmt
                img = decode_raw(buf, fmt)

            s = sharpness(img)
            if s > best_score:
                best, best_score, best_t = img, s, tcol[i].as_py()
            seen += 1
            if seen % args.window == 0:
                flush()
    flush()

    with open(args.out / "timestamps.csv", "w") as fh:
        fh.write(f"index,{timeline},sharpness\n")
        for i, t, s in rows:
            fh.write(f"{i},{t},{s:.1f}\n")
    print(f"read {seen} frames, kept {kept} -> {args.out}")


if __name__ == "__main__":
    main()
