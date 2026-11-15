"""
The self-steering, live: front colour pictures -> the small sidewalk network (on the
GPU, through TensorRT) -> the sidewalk's edges -> where the ramp is on it
(autosteer.Estimator) -> which way the steering should be tapped (autosteer.Controller).

camera.py runs one Autosteer and feeds it pictures, the gyro and the control loop's
state. In "shadow" mode it only watches: what it would do is shown on the web page
and written into the recording, and nothing is sent to the actuator.

It needs a folder (camera.py: AUTOSTEER_DIR) holding
    sidewalk_net.onnx       from train_sidewalk.py
    camera_pose.json        from sidewalk_geometry.py (any recent run's)
    steering_model.json     from steering_replay.py
The first start after a new sidewalk_net.onnx takes a few minutes: the network is
converted for this machine's GPU (sidewalk_net.engine, kept beside it).

Try it on an exported run, against what sidewalk_geometry.py found from the big model's labels:
    python sidewalk_live.py FOLDER RUN_DIR
"""
import ctypes
import json
import os
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np
import pandas as pd

from autosteer import DEG, Controller, Estimator, Params
from sidewalk_geometry import Ground, find_edges, to_geometry

TRTEXEC = "/usr/src/tensorrt/bin/trtexec"
VISION_HZ = 10              # pictures looked at per second (the camera makes 30)
MIN_WIDTH_M, MAX_WIDTH_M = 0.7, 2.5     # two edges further apart or closer than this are not a sidewalk
WIDTHS_KEPT, WIDTHS_NEEDED = 150, 15    # the usual width is the median of the last so many two-edge pictures
CONTROL_GAP_S = 0.5         # the control loop silent for this long: start the estimate afresh


class Net:
    """sidewalk_net.onnx on the GPU. mask(bgr picture) -> bool array, True where it sees sidewalk."""

    def __init__(self, onnx, say=print):
        import tensorrt as trt
        engine = os.path.splitext(onnx)[0] + ".engine"
        if not os.path.isfile(engine) or os.path.getmtime(engine) < os.path.getmtime(onnx):
            say("converting the sidewalk network for this GPU (a few minutes, once)")
            tmp = engine + ".partial"
            r = subprocess.run([TRTEXEC, f"--onnx={onnx}", "--fp16", f"--saveEngine={tmp}", "--skipInference"],
                               capture_output=True, text=True)
            if r.returncode or not os.path.isfile(tmp):
                raise RuntimeError("trtexec failed: " + (r.stdout + r.stderr)[-400:])
            os.replace(tmp, engine)
        self._logger = trt.Logger(trt.Logger.WARNING)
        with open(engine, "rb") as f:
            self._engine = trt.Runtime(self._logger).deserialize_cuda_engine(f.read())
        if self._engine is None:
            raise RuntimeError(f"could not load {engine} (delete it to have it converted again)")
        self._ctx = self._engine.create_execution_context()
        names = [self._engine.get_tensor_name(i) for i in range(self._engine.num_io_tensors)]
        shapes = [tuple(self._engine.get_tensor_shape(n)) for n in names]
        if len(names) != 2 or len(shapes[0]) != 4 or shapes[0][:2] != (1, 3):
            raise RuntimeError(f"unexpected network: {dict(zip(names, shapes))}")
        self.h, self.w = shapes[0][2:]
        cuda = self._cuda = ctypes.CDLL("libcudart.so")
        cuda.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        cuda.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self._host = [np.zeros(s, np.float32) for s in shapes]
        self._dev = []
        for a in self._host:
            p = ctypes.c_void_p()
            if cuda.cudaMalloc(ctypes.byref(p), a.nbytes):
                raise RuntimeError("no GPU memory for the sidewalk network")
            self._dev.append(p)

    def mask(self, bgr):
        if bgr.shape[:2] != (self.h, self.w):
            import cv2
            bgr = cv2.resize(bgr, (self.w, self.h), interpolation=cv2.INTER_AREA)
        inp, out = self._host
        inp[0] = bgr[:, :, ::-1].transpose(2, 0, 1)                 # BGR -> RGB, channels first
        inp *= 1 / 255.0
        cuda, (d_in, d_out) = self._cuda, self._dev
        if (cuda.cudaMemcpy(d_in, inp.ctypes.data, inp.nbytes, 1)                    # 1 = host to device
                or not self._ctx.execute_v2([d_in.value, d_out.value])
                or cuda.cudaMemcpy(out.ctypes.data, d_out, out.nbytes, 2)):          # 2 = device to host
            raise RuntimeError("the sidewalk network failed to run")
        return out[0, 0] > 0.5


class Geometry:
    """A sidewalk mask -> offset from the centre line and heading, for the front camera."""

    def __init__(self, pose):
        self.ground = Ground(**pose)
        self.top = self.ground.top_row()
        self.widths = deque(maxlen=WIDTHS_KEPT)

    def measure(self, mask):
        """{"edges": 0, 1 or 2, "offset": m, "heading": rad, "width": m or None, "why": why not, if edges is 0}"""
        usual = float(np.median(self.widths)) if len(self.widths) >= WIDTHS_NEEDED else None
        lines = pd.DataFrame([{"frame": 0, "paved_frac": float(mask.mean()), **find_edges(mask, self.top)}])
        with np.errstate(invalid="ignore"):
            g = to_geometry(lines, self.ground, False, usual)[0].iloc[0]
        edges, why = int(g.edges), str(g.why)
        if edges == 2 and not MIN_WIDTH_M < g.width_m < MAX_WIDTH_M:
            edges, why = 0, "edges not a sidewalk's width apart"
        elif edges == 2:
            self.widths.append(float(g.width_m))
        elif edges == 1 and usual is None:
            edges, why = 0, "one edge, and the sidewalk's width isn't known yet"
        if not edges:
            return {"edges": 0, "offset": np.nan, "heading": np.nan, "width": usual, "why": why}
        return {"edges": edges, "offset": float(g.offset_m), "heading": float(g.heading_rad),
                "width": float(g.width_m) if edges == 2 else usual, "why": ""}


def load(folder):
    """(onnx path, front camera pose, Params) from the folder, or raises FileNotFoundError naming what is missing."""
    folder = os.path.expanduser(folder)
    paths = [os.path.join(folder, n) for n in ("sidewalk_net.onnx", "camera_pose.json", "steering_model.json")]
    for p in paths:
        if not os.path.isfile(p):
            raise FileNotFoundError(f"no {p}")
    with open(paths[1]) as f:
        pose = json.load(f)["front"]
    params = Params()
    with open(paths[2]) as f:
        for k, v in json.load(f).items():
            if hasattr(params, k):
                setattr(params, k, v)
    return paths[0], pose, params


class Autosteer(threading.Thread):
    """
    picture(), gyro() and control() are called from camera.py's threads as things arrive;
    none of them waits. The network runs in this thread; the estimate advances in
    control(), once per message from the control loop (50 a second).
    """

    def __init__(self, folder, mode="shadow", say=print, log=None, stop=None):
        super().__init__(name="autosteer", daemon=True)
        self.folder, self.say, self.log = folder, say, log or (lambda t, values: None)
        self.stop = stop or threading.Event()
        self.mode = mode                    # "off" or "shadow"
        self.problem = "starting"           # why it can't work, or None once it can
        self.mask = None                    # the newest mask, for drawing on the live view
        self._lock = threading.Lock()
        self._new = threading.Condition(self._lock)
        self._picture = None                # (time, bgr) waiting to be looked at
        self._last_picture = 0.0
        self._seen = []                     # measurements waiting for control() to use them
        self._gyro = [0.0, 0]               # sum and count since the last control message
        self._p = None
        self._est = self._ctl = None
        self._last = {"t": 0.0, "steer": 0}
        self._view = {}                     # for the web page
        self._rate = 0.0

    # ---- called from camera.py ----
    def set_mode(self, mode):
        if mode not in ("off", "shadow"):
            return False
        with self._lock:
            self.mode = mode
            self._est = None                # start afresh when switched on
            self._view, self.mask = {}, None
        return True

    def picture(self, t, bgr):
        """A front colour picture. Takes a copy of one in three; the rest cost nothing."""
        if self.mode == "off" or self.problem or t - self._last_picture < 1.0 / VISION_HZ - 0.01:
            return
        self._last_picture = t
        with self._new:
            self._picture = (t, bgr.copy())
            self._new.notify()

    def gyro(self, yaw_rate):
        """rad/s about the camera's y axis (+ = turning right)."""
        g = self._gyro
        g[0] += yaw_rate
        g[1] += 1

    def control(self, msg):
        """One state message from control_steering.py."""
        p = self._p
        if p is None or self.mode == "off":
            return
        t = msg.get("t", time.time())
        vesc = [v["erpm"] for v in (msg.get("vesc") or {}).values() if v]
        erpm = float(np.mean(vesc)) if vesc else 0.0
        with self._lock:
            seen, self._seen = self._seen, []
            (total, n), self._gyro = self._gyro, [0.0, 0]
            dt = t - self._last["t"]
            if self._est is None or not 0 < dt < CONTROL_GAP_S:
                self._est, self._ctl, dt = Estimator(p), Controller(p), 0.02
            est, ctl = self._est, self._ctl
            est.predict(dt, erpm, self._last["steer"])
            if n:
                est.gyro(total / n)
            for z in seen:
                z["used"] = est.vision(z["offset"], z["heading"], z["edges"])
            ctl.step(est, dt)
            self._last = {"t": t, "steer": int(msg.get("steer") or 0)}
            now = {"offset": float(est.x[0]), "heading": float(est.x[1]), "steer": float(est.x[2]),
                   "sure": bool(est.sure), "want": int(ctl.want(est)), "target": float(ctl.aim(est)),
                   "speed": float(est.v), "blind_s": float(est.blind_s)}
            self._view.update(now, at=time.monotonic())
        for z in seen:                      # into the recording, at the picture's own time
            self.log(z["t"], {"seen_edges": z["edges"], "seen_offset": z["offset"], "seen_heading": z["heading"],
                              "seen_used": float(z["used"]), **{k: float(v) for k, v in now.items()}})

    def status(self):
        """For the web page."""
        with self._lock:
            v, mode, problem = dict(self._view), self.mode, self.problem
        out = {"mode": mode, "available": problem is None}
        if problem:
            out["text"] = f"autosteer: {problem}"
        elif mode == "off":
            out["text"] = "autosteer off"
        else:
            side = lambda x, unit, n: f"{abs(x):.{n}f} {unit} {'right' if x > 0 else 'left'}"
            seen = (f"camera: {side(v['seen_offset'] * 100, 'cm', 0)}, {side(v['seen_heading'] / DEG, 'deg', 1)}"
                    f" ({v['seen_edges']} edge{'s' * (v['seen_edges'] != 1)})" if v.get("seen_edges")
                    else f"camera: no result ({v.get('seen_why') or 'no pictures yet'})")
            lines = [f"autosteer ({mode}, {self._rate:.0f} pictures/s)", seen]
            if time.monotonic() - v.get("at", -1e9) > 1.0:
                lines.append("estimate: waiting for control_steering.py")
            elif not v["sure"]:
                lines.append("estimate: not sure where the sidewalk is; would steer straight")
            else:
                lines.append(f"estimate: {side(v['offset'] * 100, 'cm', 0)} of centre, pointing {side(v['heading'] / DEG, 'deg', 1)},"
                             f" steering {side(v['steer'] / DEG, 'deg', 1)}")
            if time.monotonic() - v.get("at", -1e9) <= 1.0:
                lines.append("would tap: " + {-1: "◀ left", 0: "nothing", 1: "right ▶"}[v["want"]]
                             if abs(v["speed"]) >= self._p.min_speed else "would tap: nothing (not moving)")
            out["text"] = "\n".join(lines)
            out.update({k: v[k] for k in ("offset", "heading", "steer", "sure", "want") if k in v})
        return out

    # ---- this thread ----
    def run(self):
        try:
            onnx, pose, self._p = load(self.folder)
            net, geometry = Net(onnx, self.say), Geometry(pose)
        except Exception as e:
            self.problem = str(e)
            self.say(f"autosteer not available: {e}")
            return
        self.problem = None
        self.say(f"autosteer ready ({self.mode}): {onnx}")
        times = deque(maxlen=20)
        while not self.stop.is_set():
            with self._new:
                if self._picture is None:
                    self._new.wait(0.5)
                pic, self._picture = self._picture, None
            if pic is None or self.mode == "off":
                continue
            t, bgr = pic
            try:
                mask = net.mask(bgr)
                z = geometry.measure(mask)
            except Exception as e:
                self.problem = f"stopped: {e!r}"
                self.say(f"autosteer stopped: {e!r}")
                return
            times.append(time.monotonic())
            with self._lock:
                if self.mode == "off":
                    continue
                self.mask = mask
                if len(times) > 1:
                    self._rate = (len(times) - 1) / (times[-1] - times[0])
                self._view.update(seen_edges=z["edges"], seen_offset=z["offset"], seen_heading=z["heading"], seen_why=z["why"])
                if z["edges"]:
                    self._seen.append({"t": t, **z})
                    del self._seen[:-5]     # control_steering.py not running: don't pile them up


def main():
    """Run the network and the geometry over an exported run's colour pictures and compare with geometry.parquet."""
    import cv2
    folder, run = sys.argv[1], os.path.expanduser(sys.argv[2]).rstrip("/")
    onnx, pose, _ = load(folder)
    net, geometry = Net(onnx), Geometry(pose)
    ref = pd.read_parquet(os.path.join(run, "geometry.parquet")).set_index("frame")
    rows, spent = [], []
    for frame in ref.index[::2]:
        img = cv2.imread(os.path.join(run, "color", f"{frame:06d}.jpg"))
        if img is None:
            continue
        t0 = time.perf_counter()
        z = geometry.measure(net.mask(img))
        spent.append(time.perf_counter() - t0)
        rows.append((int(ref.edges[frame]), z["edges"], z["offset"] - ref.offset_m[frame], z["heading"] - ref.heading_rad[frame]))
    r = np.array(rows, float)
    both = (r[:, 0] > 0) & (r[:, 1] > 0)
    print(f"{len(r)} pictures, {np.median(spent) * 1000:.0f} ms each ({np.percentile(spent, 95) * 1000:.0f} ms at worst twentieth)")
    print(f"a result from the big model's labels on {100 * (r[:, 0] > 0).mean():.0f}%, live on {100 * (r[:, 1] > 0).mean():.0f}%"
          f" (only the big: {int(((r[:, 0] > 0) & ~both).sum())}, only live: {int(((r[:, 1] > 0) & ~both).sum())})")
    d = np.abs(r[both])
    print(f"where both: offset differs by {np.median(d[:, 2]) * 100:.1f} cm typically, {np.percentile(d[:, 2], 90) * 100:.1f} cm at the worst tenth;"
          f" heading by {np.median(d[:, 3]) / DEG:.1f} deg, {np.percentile(d[:, 3], 90) / DEG:.1f} deg")


if __name__ == "__main__":
    main()
