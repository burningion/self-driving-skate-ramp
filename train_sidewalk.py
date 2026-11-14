#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "torch",
#     "torchvision",
#     "onnx",
#     "numpy",
#     "opencv-python-headless",
#     "pandas",
#     "pyarrow",
# ]
# ///
"""
Train a small network to find the sidewalk, from the labels the big model made
(label_sidewalk.py), and write it out in a form the Jetson can run.
For the training machine (needs a GPU; a few minutes on a 4090).

  uv run train_sidewalk.py RUN_DIR ...
        --out DIR          where to write (default: sidewalk_net/ beside the runs)
        --cams color rear  which cameras' pictures to learn from (default: both)
        --val-run NAME     keep this whole run aside for checking, instead of a fifth of every run
        --epochs N         passes over the pictures (default 40)
        --batch N          pictures per step (default 16)
        --width N          size of the network's decoder (default 64)
        --workers N        processes reading pictures (default 8)

RUN_DIR is a folder written by export_run.py that label_sidewalk.py has been
run on. Writes into DIR:

  sidewalk_net.onnx     the network, for the Jetson: a 480x640 RGB picture (values 0..1,
                        shape 1x3x480x640) in, the chance that each pixel is sidewalk out (1x1x480x640)
  sidewalk_net.pt       the same as PyTorch weights, to carry on from or export again
  training.json         the settings and how each pass went
  preview.jpg           pictures it did not learn from: its answer tinted green, the big model's outlined in red

A fifth of each run, in 10 s stretches, is kept aside and never learned from.
The scores at the end are on those pictures: how much of the sidewalk it
agrees on with the big model, and, what the steering actually uses, how far
the offset and heading worked out from its answer are from those worked out
from the big model's (needs camera_pose.json from sidewalk_geometry.py).
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

from sidewalk_geometry import Ground, find_edges, paved_ids, to_geometry

H, W = 480, 640
BLOCK_FRAMES = 300          # 10 s: pictures closer together than this are near copies, so a stretch is kept aside whole
MEAN, STD = (0.485, 0.456, 0.406), (0.229, 0.224, 0.225)


# ---------- the pictures ----------

def find_pictures(run_dirs, cams, val_run):
    """[(picture, label, run dir, cam, frame, for checking?)] and, per run, the class numbers that count as sidewalk."""
    items, paved = [], {}
    for d in run_dirs:
        cf = os.path.join(d, "labels", "classes.json")
        if not os.path.isfile(cf):
            print(f"{os.path.basename(d)}: no labels (run label_sidewalk.py on it first); skipped")
            continue
        paved[d] = paved_ids(json.load(open(cf))["id2label"])
        for cam in cams:
            for lab in sorted(glob.glob(os.path.join(d, "labels", cam, "*.png"))):
                frame = int(os.path.basename(lab)[:-4])
                pic = os.path.join(d, cam, f"{frame:06d}.jpg")
                if not os.path.isfile(pic):
                    continue
                held = os.path.basename(d) == val_run if val_run else (frame // BLOCK_FRAMES) % 5 == 4
                items.append((pic, lab, d, cam, frame, held))
    return items, paved


class Pictures(torch.utils.data.Dataset):
    def __init__(self, items, paved):
        self.items = items
        self.lut = {}
        for d, ids in paved.items():
            lut = np.zeros(256, np.uint8)
            lut[ids] = 1
            self.lut[d] = lut

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        pic, lab, d, *_ = self.items[i]
        img = cv2.cvtColor(cv2.imread(pic), cv2.COLOR_BGR2RGB)
        mask = self.lut[d][cv2.imread(lab, cv2.IMREAD_UNCHANGED)]
        if img.shape[:2] != (H, W):
            img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
        if mask.shape != (H, W):
            mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
        return torch.from_numpy(img).permute(2, 0, 1), torch.from_numpy(mask)[None], i


def vary(img, mask):
    """
    Make every picture a bit different each time it is seen, so the network learns the
    sidewalk and not these particular pictures: mirrored, tilted, nearer or further,
    brighter or darker, tinted, in shadow, noisy. img: float 0..1 (B,3,H,W); mask: float (B,1,H,W).
    """
    B, dev = img.shape[0], img.device
    r = lambda lo, hi, shape=(B, 1, 1, 1): torch.empty(shape, device=dev).uniform_(lo, hi)
    # shape: mirror, turn, zoom, shift
    ang = r(-6, 6, (B,)) * math.pi / 180
    zoom = 1 / r(0.85, 1.25, (B,))
    flip = torch.where(torch.rand(B, device=dev) < 0.5, -1.0, 1.0)
    cos, sin = torch.cos(ang) * zoom, torch.sin(ang) * zoom
    theta = torch.stack([torch.stack([cos * flip, -sin * H / W, r(-0.15, 0.15, (B,))], 1),
                         torch.stack([sin * flip * W / H, cos, r(-0.10, 0.10, (B,))], 1)], 1)
    grid = F.affine_grid(theta, img.shape, align_corners=False)
    img = F.grid_sample(img, grid, mode="bilinear", padding_mode="border", align_corners=False)
    mask = F.grid_sample(mask, grid, mode="nearest", padding_mode="border", align_corners=False)
    # light and colour
    grey = img.mean(1, keepdim=True)
    img = grey + (img - grey) * torch.where(torch.rand(B, 1, 1, 1, device=dev) < 0.1, 0.0, 1.0) * r(0.5, 1.5)
    img = img * r(0.85, 1.15, (B, 3, 1, 1))                                     # a colour cast
    mean = img.mean((1, 2, 3), keepdim=True)
    img = (mean + (img - mean) * r(0.7, 1.4)) * r(0.6, 1.5)
    img = img.clamp(1e-4, 1) ** r(0.7, 1.5)
    # a shadow across part of the picture
    yy, xx = torch.meshgrid(torch.linspace(-1, 1, H, device=dev), torch.linspace(-1, 1, W, device=dev), indexing="ij")
    a = r(0, 2 * math.pi)
    side = torch.sigmoid((torch.cos(a) * xx + torch.sin(a) * yy - r(-0.7, 0.7)) * r(8, 60))
    dark = torch.where(torch.rand(B, 1, 1, 1, device=dev) < 0.35, r(0.35, 0.8), torch.ones(B, 1, 1, 1, device=dev))
    img = img * (1 - side * (1 - dark))
    # focus and grain
    soft = torch.rand(B, 1, 1, 1, device=dev) < 0.2
    img = torch.where(soft, F.avg_pool2d(img, 3, 1, 1), img)
    img = img + torch.randn_like(img) * r(0, 0.03)
    return img.clamp(0, 1), mask


# ---------- the network ----------

class SidewalkNet(nn.Module):
    """
    A MobileNetV3 that has already learned to see from ImageNet, with a small decoder
    that brings its coarse answer back up to half the picture's size, where the
    sidewalk's edges are still sharp. Picture in (RGB, 0..1), one number per pixel out.
    """
    TAPS = (1, 3, 6, 12, 15)            # layers of the MobileNet whose output is used: 1/2, 1/4, 1/8, 1/16, 1/32 size

    def __init__(self, width=64, pretrained=True):
        super().__init__()
        weights = torchvision.models.MobileNet_V3_Large_Weights.IMAGENET1K_V2 if pretrained else None
        self.body = torchvision.models.mobilenet_v3_large(weights=weights).features[:self.TAPS[-1] + 1]
        self.register_buffer("mean", torch.tensor(MEAN).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(STD).view(1, 3, 1, 1))
        with torch.no_grad():
            chans = [f.shape[1] for f in self._features(torch.zeros(1, 3, 64, 64))]
        self.side = nn.ModuleList(nn.Conv2d(c, width, 1) for c in chans)
        block = lambda: nn.Sequential(nn.Conv2d(width, width, 3, padding=1, bias=False), nn.BatchNorm2d(width), nn.ReLU(inplace=True))
        self.mix = nn.ModuleList(block() for _ in chans[:-1])
        self.head = nn.Conv2d(width, 1, 1)

    def _features(self, x):
        out = []
        for i, layer in enumerate(self.body):
            x = layer(x)
            if i in self.TAPS:
                out.append(x)
        return out

    def forward(self, x):
        feats = self._features((x - self.mean) / self.std)
        y = self.side[-1](feats[-1])
        for i in range(len(feats) - 2, -1, -1):
            y = self.mix[i](self.side[i](feats[i]) + F.interpolate(y, size=feats[i].shape[2:], mode="bilinear", align_corners=False))
        return F.interpolate(self.head(y), size=x.shape[2:], mode="bilinear", align_corners=False)


class Exported(nn.Module):
    def __init__(self, net):
        super().__init__()
        self.net = net

    def forward(self, x):
        return torch.sigmoid(self.net(x))


def loss_fn(logits, mask):
    p = torch.sigmoid(logits)
    inter = (p * mask).sum((1, 2, 3))
    dice = 1 - (2 * inter + 1) / (p.sum((1, 2, 3)) + mask.sum((1, 2, 3)) + 1)
    return F.binary_cross_entropy_with_logits(logits, mask) + dice.mean()


# ---------- checking ----------

@torch.inference_mode()
def predict(net, loader, device, half):
    """Its answer for every picture of a loader: {index: bool (H, W)}, and the agreement with the big model per camera."""
    net.eval()
    masks, inter, union = {}, {}, {}
    for img, mask, idx in loader:
        img = img.to(device, non_blocking=True).float() / 255
        with torch.autocast(device, dtype=torch.float16, enabled=half):
            out = net(img.contiguous(memory_format=torch.channels_last)) > 0
        out, mask = out.cpu().numpy()[:, 0], mask.numpy()[:, 0].astype(bool)
        for k, i in enumerate(idx.tolist()):
            masks[i] = out[k]
            cam = loader.dataset.items[i][3]
            inter[cam] = inter.get(cam, 0) + int((out[k] & mask[k]).sum())
            union[cam] = union.get(cam, 0) + int((out[k] | mask[k]).sum())
    return masks, {cam: inter[cam] / max(union[cam], 1) for cam in union}


def geometry_check(dataset, masks):
    """Offset and heading from the small network's answer against those from the big model's, per run."""
    by_run = {}
    for i, (pic, lab, d, cam, frame, held) in enumerate(dataset.items):
        if cam == "color" and i in masks:
            by_run.setdefault(d, []).append(i)
    lines = []
    for d, idx in sorted(by_run.items()):
        name = os.path.basename(d)
        pose_file = os.path.join(d, "camera_pose.json")
        if not os.path.isfile(pose_file) or "front" not in json.load(open(pose_file)):
            lines.append(f"   {name}: no camera_pose.json (run sidewalk_geometry.py on it), so no offset and heading to compare")
            continue
        ground = Ground(**json.load(open(pose_file))["front"])
        gp = os.path.join(d, "geometry.parquet")
        width = float(pd.read_parquet(gp).width_m.median()) if os.path.isfile(gp) else None
        rows = {"big": [], "small": []}
        for i in idx:
            frame = dataset.items[i][4]
            teacher = dataset.lut[d][cv2.imread(dataset.items[i][1], cv2.IMREAD_UNCHANGED)].astype(bool)
            for who, m in (("big", teacher), ("small", masks[i])):
                rows[who].append({"frame": frame, "paved_frac": float(m.mean()), **find_edges(m, ground.top_row())})
        big, usual, _, _ = to_geometry(pd.DataFrame(rows["big"]), ground, rear=False, known_width=width)
        if not np.isfinite(usual):
            lines.append(f"   {name}: the big model's labels give no sidewalk width here; skipped")
            continue
        small = to_geometry(pd.DataFrame(rows["small"]), ground, rear=False, known_width=usual)[0]
        b, s = big.edges.to_numpy() > 0, small.edges.to_numpy() > 0
        both = b & s
        if both.sum() < 5:
            lines.append(f"   {name}: {len(idx)} pictures, too few with a result from both to compare")
            continue
        do = np.abs(big.offset_m.to_numpy() - small.offset_m.to_numpy())[both] * 100
        dh = np.degrees(np.abs(big.heading_rad.to_numpy() - small.heading_rad.to_numpy()))[both]
        lines.append(f"   {name}: {len(idx)} pictures. Result from the big model's labels on {b.mean() * 100:.0f}%, from the small network's on {s.mean() * 100:.0f}%"
                     f" (only the big: {(b & ~s).sum()}, only the small: {(s & ~b).sum()}).")
        lines.append(f"      where both: offset differs by {np.median(do):.1f} cm typically, {np.percentile(do, 90):.1f} cm at the worst tenth;"
                     f" heading by {np.median(dh):.1f} deg, {np.percentile(dh, 90):.1f} deg")
    return lines


def preview(dataset, masks, path, n=12):
    idx = sorted(masks)
    idx = [idx[i] for i in np.linspace(0, len(idx) - 1, min(n, len(idx))).astype(int)]
    tiles = []
    for i in idx:
        pic, lab, d, cam, frame, _ = dataset.items[i]
        img = cv2.resize(cv2.imread(pic), (W, H))
        m = masks[i]
        img[m] = (0.6 * img[m] + 0.4 * np.array([0, 200, 0])).astype(np.uint8)
        teacher = cv2.resize(dataset.lut[d][cv2.imread(lab, cv2.IMREAD_UNCHANGED)], (W, H), interpolation=cv2.INTER_NEAREST)
        cv2.drawContours(img, cv2.findContours(teacher, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1, (0, 0, 255), 1)
        cv2.putText(img, f"{os.path.basename(d)[-8:]} {cam} {frame}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        tiles.append(img)
    if tiles:
        tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 3)
        cv2.imwrite(path, np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]), [cv2.IMWRITE_JPEG_QUALITY, 85])


def export_onnx(net, path):
    model = Exported(net).cpu().float().eval()
    example = torch.zeros(1, 3, H, W)
    kw = dict(input_names=["image"], output_names=["sidewalk"], opset_version=17)
    try:
        torch.onnx.export(model, example, path, dynamo=False, **kw)
    except TypeError:                   # an older torch, where this is the only exporter
        torch.onnx.export(model, example, path, **kw)


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="exported run folders that label_sidewalk.py has been run on")
    ap.add_argument("--out")
    ap.add_argument("--cams", nargs="+", default=["color", "rear"], choices=["color", "rear"])
    ap.add_argument("--val-run")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--width", type=int, default=64)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--no-pretrained", action="store_true", help="start from nothing instead of ImageNet weights (worse; for testing offline)")
    args = ap.parse_args()

    run_dirs = [os.path.expanduser(r).rstrip("/") for r in args.runs]
    out = os.path.expanduser(args.out) if args.out else os.path.join(os.path.dirname(run_dirs[0]), "sidewalk_net")
    os.makedirs(out, exist_ok=True)
    items, paved = find_pictures(run_dirs, args.cams, args.val_run)
    learn = Pictures([it for it in items if not it[5]], paved)
    check = Pictures([it for it in items if it[5]], paved)
    if not len(learn) or not len(check):
        sys.exit(f"{len(learn)} pictures to learn from and {len(check)} to check with: need some of both")
    for d in run_dirs:
        n = [sum(it[2] == d and it[3] == cam for it in items) for cam in args.cams]
        print(f"{os.path.basename(d)}: " + ", ".join(f"{k} {cam}" for k, cam in zip(n, args.cams)) + " labelled pictures")
    print(f"learning from {len(learn)} pictures, checking with {len(check)} kept aside")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    half = device == "cuda"
    if device == "cpu":
        print("WARNING: no GPU found by torch; this will take hours instead of minutes.")
    print(f"training on {torch.cuda.get_device_name(0) if half else 'the CPU'}", flush=True)
    net = SidewalkNet(args.width, pretrained=not args.no_pretrained).to(device).to(memory_format=torch.channels_last)
    print(f"the network has {sum(p.numel() for p in net.parameters()) / 1e6:.1f} million weights")

    loader_kw = dict(num_workers=args.workers, pin_memory=half, persistent_workers=args.workers > 0)
    learn_loader = torch.utils.data.DataLoader(learn, batch_size=args.batch, shuffle=True, drop_last=len(learn) > args.batch, **loader_kw)
    check_loader = torch.utils.data.DataLoader(check, batch_size=args.batch, shuffle=False, **loader_kw)
    body = [p for n, p in net.named_parameters() if n.startswith("body.")]
    rest = [p for n, p in net.named_parameters() if not n.startswith("body.")]
    opt = torch.optim.AdamW([{"params": body, "lr": args.lr * 0.3}, {"params": rest, "lr": args.lr}], weight_decay=1e-4)
    steps = args.epochs * len(learn_loader)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=[args.lr * 0.3, args.lr], total_steps=max(steps, 1), pct_start=0.1)
    scaler = torch.amp.GradScaler(enabled=half)

    weights = os.path.join(out, "sidewalk_net.pt")
    history, best = [], -1.0
    started = time.monotonic()
    for epoch in range(args.epochs):
        net.train()
        total = 0.0
        for img, mask, _ in learn_loader:
            img = img.to(device, non_blocking=True).float() / 255
            mask = mask.to(device, non_blocking=True).float()
            img, mask = vary(img, mask)
            with torch.autocast(device, dtype=torch.float16, enabled=half):
                logits = net(img.contiguous(memory_format=torch.channels_last))
            loss = loss_fn(logits.float(), mask)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            total += loss.item()
        _, agree = predict(net, check_loader, device, half)
        score = float(np.mean(list(agree.values())))
        history.append({"epoch": epoch + 1, "loss": total / len(learn_loader), **{f"agree_{c}": a for c, a in agree.items()}})
        if score > best:
            best = score
            torch.save({"weights": net.state_dict(), "width": args.width}, weights)
        left = (time.monotonic() - started) / (epoch + 1) * (args.epochs - epoch - 1)
        print(f"  pass {epoch + 1}/{args.epochs}: loss {total / len(learn_loader):.3f}   agrees with the big model on "
              + ", ".join(f"{a * 100:.1f}% ({c})" for c, a in agree.items()) + f"   {left / 60:.0f} min left", flush=True)

    if os.path.isfile(weights):
        net.load_state_dict(torch.load(weights, map_location=device)["weights"])
    masks, agree = predict(net, check_loader, device, half)
    print("on the pictures kept aside, with the best weights:")
    print("   sidewalk it and the big model both mark, out of what either marks: " + ", ".join(f"{a * 100:.1f}% ({c})" for c, a in agree.items()))
    geo = geometry_check(check, masks)
    print("\n".join(geo))
    preview(check, masks, os.path.join(out, "preview.jpg"))
    onnx_path = os.path.join(out, "sidewalk_net.onnx")
    export_onnx(net, onnx_path)
    with open(os.path.join(out, "training.json"), "w") as f:
        json.dump({"runs": [os.path.basename(d) for d in run_dirs], "settings": vars(args), "learned_from": len(learn),
                   "checked_with": len(check), "input": "1x3x480x640 RGB, 0..1", "output": "1x1x480x640, chance of sidewalk",
                   "agreement": agree, "geometry": geo, "history": history}, f, indent=1)
    print(f"written: {onnx_path} ({os.path.getsize(onnx_path) / 1e6:.0f} MB), and preview.jpg, sidewalk_net.pt, training.json beside it")


if __name__ == "__main__":
    main()
