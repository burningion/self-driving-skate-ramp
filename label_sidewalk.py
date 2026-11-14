#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "torch",
#     "transformers>=4.40",
#     "pillow",
#     "numpy",
#     "opencv-python-headless",
#     "pandas",
#     "pyarrow",
# ]
# ///
"""
Label what is sidewalk in a run's pictures, with a pretrained street-scene
segmentation model. For the training machine (needs a GPU; not for the Jetson).

  uv run label_sidewalk.py RUN_DIR ...
        --cams color rear   which cameras (default: both)
        --every N           label one picture in N (default 3, i.e. 10 a second)
        --batch N           pictures per batch (default 4)
        --half              16-bit floats: faster, and usually the same result
        --model NAME        a Hugging Face semantic segmentation model
        --force             label again what is already labelled

RUN_DIR is a folder written by export_run.py. Writes into it:

  labels/classes.json            the model's class names, by number
  labels/<cam>/000123.png        for picture <cam>/000123.jpg: the class number of every pixel
  labels/preview_<cam>.jpg       a sheet of pictures with the sidewalk tinted green, to check by eye

The whole class map is kept, not just "sidewalk or not", so which classes
count as sidewalk can be changed afterwards without running the model again
(see PAVED_WORDS in sidewalk_geometry.py, which reads these files next).

Stopping and starting again carries on where it left off.
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

from sidewalk_geometry import PAVED_WORDS, paved_ids

# Trained on Mapillary Vistas: street scenes from many viewpoints, with separate
# sidewalk, road, curb, terrain and vegetation classes.
MODEL = "facebook/mask2former-swin-large-mapillary-vistas-semantic"


def todo(run_dir, cam, every, force):
    """(picture path, label path) for the pictures still to label."""
    out = []
    for f in sorted(glob.glob(os.path.join(run_dir, cam, "*.jpg")))[::every]:
        lab = os.path.join(run_dir, "labels", cam, os.path.basename(f)[:-4] + ".png")
        if force or not os.path.isfile(lab):
            out.append((f, lab))
    return out


def preview(run_dir, cam, paved, n=12):
    labs = sorted(glob.glob(os.path.join(run_dir, "labels", cam, "*.png")))
    tiles = []
    for lab in [labs[i] for i in np.linspace(0, len(labs) - 1, min(n, len(labs))).astype(int)]:
        img = cv2.imread(os.path.join(run_dir, cam, os.path.basename(lab)[:-4] + ".jpg"))
        m = np.isin(cv2.imread(lab, cv2.IMREAD_UNCHANGED), paved)
        img[m] = (0.6 * img[m] + 0.4 * np.array([0, 200, 0])).astype(np.uint8)
        cv2.putText(img, os.path.basename(lab)[:-4], (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        tiles.append(img)
    if tiles:
        tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 3)
        cv2.imwrite(os.path.join(run_dir, "labels", f"preview_{cam}.jpg"),
                    np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]), [cv2.IMWRITE_JPEG_QUALITY, 85])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="exported run folders")
    ap.add_argument("--cams", nargs="+", default=["color", "rear"], choices=["color", "rear", "ir"])
    ap.add_argument("--every", type=int, default=3)
    ap.add_argument("--batch", type=int, default=4)
    ap.add_argument("--half", action="store_true")
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        print("WARNING: no GPU found by torch; this will take minutes per picture batch instead of a fraction of a second.\n"
              "         (On Windows, plain `pip install torch` is CPU-only; install the CUDA build from pytorch.org.)")
    print(f"loading {args.model} on {torch.cuda.get_device_name(0) if device == 'cuda' else 'the CPU'}", flush=True)
    processor = AutoImageProcessor.from_pretrained(args.model)
    model = Mask2FormerForUniversalSegmentation.from_pretrained(args.model).to(device).eval()
    id2label = {int(i): name for i, name in model.config.id2label.items()}
    if max(id2label) > 255:
        sys.exit("this model has more than 256 classes; the labels are stored as 8-bit pictures")
    paved = paved_ids(id2label)
    print("counted as sidewalk:", ", ".join(id2label[i] for i in paved) or "NOTHING (check PAVED_WORDS)")

    for run_dir in (os.path.expanduser(r).rstrip("/") for r in args.runs):
        name = os.path.basename(run_dir)
        os.makedirs(os.path.join(run_dir, "labels"), exist_ok=True)
        with open(os.path.join(run_dir, "labels", "classes.json"), "w") as f:
            json.dump({"model": args.model, "id2label": {str(i): n for i, n in sorted(id2label.items())}}, f, indent=1)
        for cam in args.cams:
            work = todo(run_dir, cam, args.every, args.force)
            if not os.path.isdir(os.path.join(run_dir, cam)):
                continue
            os.makedirs(os.path.join(run_dir, "labels", cam), exist_ok=True)
            started, done = time.monotonic(), 0
            for i in range(0, len(work), args.batch):
                batch = work[i:i + args.batch]
                images = [Image.open(f).convert("RGB") for f, _ in batch]
                inputs = processor(images=images, return_tensors="pt").to(device)
                with torch.inference_mode(), torch.autocast(device, dtype=torch.float16, enabled=args.half and device == "cuda"):
                    outputs = model(**inputs)
                outputs.class_queries_logits = outputs.class_queries_logits.float()
                outputs.masks_queries_logits = outputs.masks_queries_logits.float()
                maps = processor.post_process_semantic_segmentation(outputs, target_sizes=[im.size[::-1] for im in images])
                for (_, lab), m in zip(batch, maps):
                    cv2.imwrite(lab + ".tmp.png", m.to(torch.uint8).cpu().numpy())
                    os.replace(lab + ".tmp.png", lab)             # so a half-written file is never taken for a label
                done += len(batch)
                if (i // args.batch) % 25 == 0 or done == len(work):
                    rate = done / (time.monotonic() - started)
                    print(f"  {name} {cam}: {done}/{len(work)}  {rate:.1f} pictures/s  "
                          f"{(len(work) - done) / max(rate, 1e-9) / 60:.0f} min left", flush=True)
            preview(run_dir, cam, paved)
            total = len(glob.glob(os.path.join(run_dir, "labels", cam, "*.png")))
            print(f"{name} {cam}: {total} pictures labelled; check {os.path.join(run_dir, 'labels', f'preview_{cam}.jpg')}")


if __name__ == "__main__":
    main()
