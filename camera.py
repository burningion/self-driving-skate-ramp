#!/usr/bin/env python3
"""
RealSense D455 -> H.264 (WebRTC via MediaMTX) + MJPEG fallback, with IMU overlay, over Tailscale.

Also records runs. While control_steering.py is running, everything is written
at full rate to one Rerun file per session in RUN_DIR: colour (JPEG), depth
(16-bit PNG, lossless), raw IMU, and the control loop's state. The recording
starts when control_steering.py starts and ends a few seconds after it stops.

Run with the project venv (.venv/bin/python camera.py): it has rerun-sdk, and
falls through to the system pyrealsense2.
"""
import json, math, os, queue, shutil, socket, threading, time, signal, subprocess
import cv2, numpy as np, pyrealsense2 as rs
import rerun as rr
import rerun.blueprint as rrb
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------- settings ----------
W, H, CAM_FPS = 640, 480, 30   # capture mode (must be supported by the camera)
OUT_SCALE  = 0.5               # output size (0.5 -> 640x240 with depth); keep dimensions even
OUT_FPS    = 10
SHOW_DEPTH = True
DEPTH_MAX_MM = 6000
USE_IMU    = True
IMU_HZ     = 200
H264_KBPS  = 400               # cellular-friendly; raise for more detail
RTSP_URL   = "rtsp://127.0.0.1:8554/cam"
JPEG_Q     = 60                # MJPEG fallback only
PORT       = 8080
STATE_ADDR = ("127.0.0.1", 47800)   # control_steering.py sends its state here (UDP JSON)
RUN_DIR    = os.path.expanduser("~/ramp-runs")
REC_IDLE_S = 5.0               # close the recording this long after control_steering.py goes quiet
REC_MIN_FREE_GB = 20           # don't start a recording with less disk than this (a run is roughly 25 GB per hour)
REC_JPEG_Q = 90

# ---------- shared state ----------
stop = threading.Event()
cond = threading.Condition()
latest_jpg, frame_id = None, 0
mjpeg_clients = 0
mjpeg_lock = threading.Lock()

imu_lock = threading.Lock()
imu = {"available": False, "accel": [0, 0, 0], "gyro": [0, 0, 0], "roll": 0.0, "pitch": 0.0}

control_lock = threading.Lock()
control = {"msg": None, "at": 0.0}    # newest state packet from control_steering.py

stats_lock = threading.Lock()
stats = {"video": 0, "imu": 0, "h264": 0, "mjpeg": 0, "rec": 0, "rec_drop": 0}

def bump(key, n=1):
    with stats_lock:
        stats[key] += n

# ---------- H.264 output (ffmpeg -> MediaMTX) ----------
class H264Out:
    def __init__(self, w, h, fps):
        self.w, self.h, self.fps = w, h, fps
        self.frame, self.proc = None, None
        self.cv = threading.Condition()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def push(self, img):              # never blocks the camera loop; drops old frames
        with self.cv:
            self.frame = img
            self.cv.notify()

    def _start(self):
        cmd = ["/home/stankley/ffmpeg-static/ffmpeg-7.0.2-arm64-static/ffmpeg", "-loglevel", "error",
               "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{self.w}x{self.h}",
               "-r", str(self.fps), "-i", "-",
               "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
               "-profile:v", "baseline", "-pix_fmt", "yuv420p",
               "-g", str(self.fps * 2),
               "-b:v", f"{H264_KBPS}k", "-maxrate", f"{int(H264_KBPS * 1.5)}k",
               "-bufsize", f"{int(H264_KBPS * 1.5)}k",
               "-f", "rtsp", "-rtsp_transport", "tcp", RTSP_URL]
        print("starting ffmpeg ->", RTSP_URL, flush=True)
        return subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def _run(self):
        while not stop.is_set():
            if self.proc is None or self.proc.poll() is not None:
                if self.proc is not None:
                    print("ffmpeg exited with", self.proc.returncode, "- is mediamtx running? retrying", flush=True)
                    stop.wait(2)
                    if stop.is_set():
                        break
                self.proc = self._start()
            with self.cv:
                self.cv.wait_for(lambda: self.frame is not None or stop.is_set(), timeout=1)
                img, self.frame = self.frame, None
            if img is None:
                continue
            try:
                self.proc.stdin.write(img.tobytes())
                self.proc.stdin.flush()
                bump("h264")
            except (BrokenPipeError, OSError):
                self.proc.kill()
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            self.proc.terminate()

# ---------- run recorder (Rerun) ----------
VESC_FIELDS = ("erpm", "current_motor", "current_in", "v_in", "duty", "tach", "temp_fet")
IMU_AXES = ("x", "y", "z")

class Recorder(threading.Thread):
    """
    One thread owns the Rerun file. Everything else hands it (kind, time, data)
    through a queue and never waits: if the disk can't keep up, camera frames
    are dropped and counted rather than stalling capture.

    Every item is stamped with wall-clock time (seconds since the epoch) on the
    "time" timeline. The camera and IMU stamps come from the RealSense itself,
    which keeps them aligned to this machine's clock.
    """

    MAX_QUEUED_FRAMES = 30

    def __init__(self):
        super().__init__(name="recorder")
        self.q = queue.Queue()
        self.rec = None
        self.name = None                 # file name of the open recording, for the web page
        self.calibration = None          # set by capture() once the camera is open
        self._frames_queued = 0
        self._last_control = -1e9
        self._motors = ()
        self._last_state = None

    # ---- called from other threads ----
    def frame(self, t, color, depth):
        if self.rec is None:
            return
        if self._frames_queued >= self.MAX_QUEUED_FRAMES:
            bump("rec_drop")
            return
        self._frames_queued += 1
        self.q.put(("frame", t, (color.copy(), None if depth is None else depth.copy())))

    def imu(self, kind, t, xyz):
        if self.rec is not None:
            self.q.put((kind, t, xyz))

    def control(self, msg):
        self.q.put(("control", msg.get("t", time.time()), msg))

    # ---- recorder thread ----
    def run(self):
        while not stop.is_set():
            try:
                kind, t, data = self.q.get(timeout=0.5)
            except queue.Empty:
                kind = None
            now = time.monotonic()
            if kind == "control":
                self._last_control = now
            active = now - self._last_control < REC_IDLE_S
            if active and self.rec is None:
                self._open(data if kind == "control" else {})
            elif not active and self.rec is not None:
                self._close()
            if kind == "frame":
                self._frames_queued -= 1
            if self.rec is None or kind is None:
                continue
            try:
                self.rec.set_time("time", timestamp=t)
                if kind == "frame":
                    self._log_frame(*data)
                elif kind == "control":
                    self._log_control(data)
                else:
                    self.rec.log(f"imu/{kind}", rr.Scalars(data))
            except Exception as e:
                print("recorder error:", repr(e), flush=True)
        self._close()

    def _open(self, msg):
        free_gb = shutil.disk_usage(os.path.dirname(RUN_DIR)).free / 1e9
        if free_gb < REC_MIN_FREE_GB:
            if self._last_state != "full":
                self._last_state = "full"
                print(f"NOT recording: only {free_gb:.0f} GB free", flush=True)
            return
        os.makedirs(RUN_DIR, exist_ok=True)
        run = time.strftime("%Y-%m-%d_%H-%M-%S")
        rec = rr.RecordingStream("skate_ramp", recording_id=run)
        rec.save(os.path.join(RUN_DIR, run + ".rrd"))
        self._motors = tuple(msg.get("vesc") or ("back", "front"))
        self._last_state = None

        # things that don't change during a run
        cal = self.calibration
        if cal:
            rec.log("camera/calibration", rr.TextDocument(json.dumps(cal, indent=1), media_type="text/plain"), static=True)
            for name in ("color", "depth"):
                k = cal[name]
                rec.log(f"camera/{name}", rr.Pinhole(focal_length=[k["fx"], k["fy"]], principal_point=[k["ppx"], k["ppy"]],
                                                     resolution=[k["width"], k["height"]]), static=True)
        for path in ("imu/accel", "imu/gyro"):
            rec.log(path, rr.SeriesLines(names=IMU_AXES), static=True)
        for path in ["control/erpm_cmd"] + [f"vesc/{f}" for f in VESC_FIELDS]:
            rec.log(path, rr.SeriesLines(names=self._motors), static=True)
        rec.send_blueprint(rrb.Blueprint(
            rrb.Horizontal(
                rrb.Vertical(
                    rrb.Spatial2DView(origin="camera/color", name="color"),
                    rrb.Spatial2DView(origin="camera/depth", name="depth"),
                    rrb.TextLogView(origin="control/state", name="state"),
                ),
                rrb.Vertical(
                    rrb.TimeSeriesView(name="controller", contents=["control/r1", "control/stick", "control/steer", "control/jack"]),
                    rrb.TimeSeriesView(name="speed (ERPM)", contents=["control/erpm_cmd", "vesc/erpm"]),
                    rrb.TimeSeriesView(name="motor current (A)", contents=["vesc/current_motor"]),
                    rrb.TimeSeriesView(name="gyro (rad/s)", contents=["imu/gyro"]),
                    rrb.TimeSeriesView(name="accel (m/s2)", contents=["imu/accel"]),
                ),
            ),
            collapse_panels=True))
        self.rec, self.name = rec, run + ".rrd"
        print("recording ->", os.path.join(RUN_DIR, self.name), flush=True)

    def _close(self):
        if self.rec is None:
            return
        rec, self.rec = self.rec, None
        rec.flush()
        rec.disconnect()
        print("recording closed:", self.name, flush=True)
        self.name = None

    def _log_frame(self, color, depth):
        ok, jpg = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, REC_JPEG_Q])
        if ok:
            self.rec.log("camera/color", rr.EncodedImage(contents=jpg.tobytes(), media_type="image/jpeg"))
        if depth is not None:
            # RLE strategy: lossless, ~5 ms a frame (default PNG settings take ~35 ms)
            ok, png = cv2.imencode(".png", depth, [cv2.IMWRITE_PNG_STRATEGY, cv2.IMWRITE_PNG_STRATEGY_RLE])
            if ok:
                meter = 1.0 / self.calibration["depth_scale"] if self.calibration else 1000.0
                self.rec.log("camera/depth", rr.EncodedDepthImage(blob=png.tobytes(), media_type="image/png", meter=meter))
        bump("rec")

    def _log_control(self, m):
        log = self.rec.log
        log("control/r1", rr.Scalars(float(m.get("r1", False))))
        for key in ("stick", "steer", "jack", "speed"):
            log(f"control/{key}", rr.Scalars(float(m.get(key, 0))))
        cmd = m.get("erpm_cmd") or {}
        log("control/erpm_cmd", rr.Scalars([float(cmd.get(mo, 0)) for mo in self._motors]))
        vesc = m.get("vesc") or {}
        if all(vesc.get(mo) for mo in self._motors):
            for f in VESC_FIELDS:
                log(f"vesc/{f}", rr.Scalars([float(vesc[mo][f]) for mo in self._motors]))
        teensy = m.get("teensy")
        if teensy:
            log("teensy/steer", rr.Scalars(float(teensy["steer"])))
            log("teensy/jack", rr.Scalars(float(teensy["jack"])))
        state = (m.get("state"), tuple(m.get("faults") or ()), tuple(m.get("held") or ()))
        if state != self._last_state:
            self._last_state = state
            text = state[0] + (": " + "; ".join(state[1]) if state[1] else "") + (f"  [{' '.join(state[2])}]" if state[2] else "")
            log("control/state", rr.TextLog(text, level="WARN" if state[0] == "FAULT" else "INFO"))

recorder = Recorder()

def frame_time(fr):
    """Seconds since the epoch. The RealSense aligns its stamps to this machine's clock ("global time")."""
    if fr.get_frame_timestamp_domain() == rs.timestamp_domain.global_time:
        return fr.get_timestamp() / 1000.0
    return time.time()

# ---------- control loop state (from control_steering.py) ----------
def listen_control():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(STATE_ADDR)
    sock.settimeout(0.5)
    while not stop.is_set():
        try:
            data = sock.recv(65535)
            msg = json.loads(data)
        except (socket.timeout, ValueError):
            continue
        with control_lock:
            control["msg"], control["at"] = msg, time.monotonic()
        recorder.control(msg)

def control_snapshot():
    with control_lock:
        msg, age = control["msg"], time.monotonic() - control["at"]
    if msg is None or age > 1.0:
        return {"state": "OFFLINE", "faults": ["control_steering.py is not running"], "recording": recorder.name}
    return dict(msg, recording=recorder.name)

# ---------- IMU (opened directly on the motion sensor) ----------
def on_motion(fr):
    try:
        m = fr.as_motion_frame()
        d = m.get_motion_data()
        v = [d.x, d.y, d.z]
        recorder.imu("accel" if m.get_profile().stream_type() == rs.stream.accel else "gyro", frame_time(m), v)
        with imu_lock:
            if m.get_profile().stream_type() == rs.stream.accel:
                a = imu["accel"] = [0.9 * o + 0.1 * n for o, n in zip(imu["accel"], v)]
                ax, ay, az = a  # RealSense frame: x right, y down, z forward
                imu["roll"]  = math.degrees(math.atan2(-ax, -ay))
                imu["pitch"] = math.degrees(math.atan2(az, math.hypot(ax, ay)))
            else:
                imu["gyro"] = [math.degrees(x) for x in v]
        bump("imu")
    except Exception as e:
        print("imu callback error:", e)

def start_imu(dev):
    for s in dev.query_sensors():
        profs = s.get_stream_profiles()
        accel = [p for p in profs if p.stream_type() == rs.stream.accel and p.format() == rs.format.motion_xyz32f]
        gyro  = [p for p in profs if p.stream_type() == rs.stream.gyro  and p.format() == rs.format.motion_xyz32f]
        if accel and gyro:
            a = min(accel, key=lambda p: abs(p.fps() - IMU_HZ))
            g = min(gyro,  key=lambda p: abs(p.fps() - IMU_HZ))
            s.open([a, g])
            s.start(on_motion)
            print(f"IMU started: accel {a.fps()} Hz, gyro {g.fps()} Hz")
            with imu_lock:
                imu["available"] = True
            return s
    print("no motion sensor found")
    return None

def stop_imu(s):
    try:
        s.stop(); s.close()
    except Exception as e:
        print("imu stop error:", e)

# ---------- camera ----------
def colorize_depth(depth_mm):
    d8 = cv2.convertScaleAbs(depth_mm, alpha=255.0 / DEPTH_MAX_MM)
    img = cv2.applyColorMap(255 - d8, cv2.COLORMAP_JET)  # near = red, far = blue
    img[depth_mm == 0] = 0
    return img

def read_calibration(profile):
    def intrinsics(stream):
        k = profile.get_stream(stream).as_video_stream_profile().get_intrinsics()
        return {"width": k.width, "height": k.height, "fx": k.fx, "fy": k.fy, "ppx": k.ppx, "ppy": k.ppy,
                "model": str(k.model), "coeffs": list(k.coeffs)}
    ext = profile.get_stream(rs.stream.depth).get_extrinsics_to(profile.get_stream(rs.stream.color))
    dev = profile.get_device()
    return {"camera": dev.get_info(rs.camera_info.name), "serial": dev.get_info(rs.camera_info.serial_number),
            "color": intrinsics(rs.stream.color), "depth": intrinsics(rs.stream.depth),
            "depth_to_color": {"rotation": list(ext.rotation), "translation": list(ext.translation)},
            "depth_scale": dev.first_depth_sensor().get_depth_scale()}

def capture():
    global latest_jpg, frame_id
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, CAM_FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, CAM_FPS)
    profile = pipe.start(cfg)
    dev = profile.get_device()
    print("camera:", dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number))
    recorder.calibration = read_calibration(profile)
    motion = start_imu(dev) if USE_IMU else None

    encoder = None
    period, last_out = 1.0 / OUT_FPS, 0.0
    try:
        while not stop.is_set():
            ok, f = pipe.try_wait_for_frames(1000)   # keep draining at camera rate
            if not ok:
                continue
            bump("video")
            color_f, depth_f = f.get_color_frame(), f.get_depth_frame()
            if not color_f:
                continue
            img = np.asanyarray(color_f.get_data())
            recorder.frame(frame_time(color_f), img, np.asanyarray(depth_f.get_data()) if depth_f else None)
            now = time.monotonic()
            if now - last_out < period:
                continue
            last_out = now
            if SHOW_DEPTH:
                d = colorize_depth(np.asanyarray(depth_f.get_data())) if depth_f else np.zeros_like(img)
                if d.shape[:2] != img.shape[:2]:
                    d = cv2.resize(d, (img.shape[1], img.shape[0]))
                img = np.hstack((img, d))
            img = cv2.resize(img, None, fx=OUT_SCALE, fy=OUT_SCALE, interpolation=cv2.INTER_AREA)
            img = np.ascontiguousarray(img[: img.shape[0] // 2 * 2, : img.shape[1] // 2 * 2])  # even dims for x264

            if encoder is None:
                encoder = H264Out(img.shape[1], img.shape[0], OUT_FPS)
            encoder.push(img)

            with mjpeg_lock:
                want_jpeg = mjpeg_clients > 0
            if want_jpeg:
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                if ok:
                    with cond:
                        latest_jpg, frame_id = jpg.tobytes(), frame_id + 1
                        cond.notify_all()
    finally:
        if motion:
            stop_imu(motion)
        pipe.stop()
        print("camera stopped")

def report():
    last = time.monotonic()
    while not stop.wait(2.0):
        now = time.monotonic()
        with stats_lock:
            snap = dict(stats)
            for k in stats:
                stats[k] = 0
        dt, last = now - last, now
        print("  ".join(f"{k} {v / dt:5.1f}/s" for k, v in snap.items()), flush=True)

# ---------- web page ----------
PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>skate cam</title>
<style>
html,body{margin:0;height:100%;background:#111;color:#eee;font:14px ui-monospace,Menlo,monospace;overflow:hidden}
#v{position:fixed;inset:0;width:100vw;height:100vh;border:0;object-fit:contain;display:block}
#hud{position:fixed;top:10px;left:10px;background:rgba(0,0,0,.6);padding:8px 12px;border-radius:8px;white-space:pre;line-height:1.4;pointer-events:none;z-index:2}
#hz{position:fixed;bottom:12px;right:12px;border-radius:50%;background:rgba(0,0,0,.4);pointer-events:none;z-index:2}
#mode{position:fixed;top:10px;right:10px;z-index:2;color:#aaa;font-size:12px}
#mode a{color:#8cf}
#state{position:fixed;bottom:12px;left:10px;background:rgba(0,0,0,.6);padding:8px 12px;border-radius:8px;white-space:pre;line-height:1.4;pointer-events:none;z-index:2;font-weight:bold}
#state small{font-weight:normal;color:#ccc}
</style></head><body>
<div id="mode"></div><div id="hud">connecting...</div><div id="state"></div><canvas id="hz" width="140" height="140"></canvas>
<script>
const mjpeg = new URLSearchParams(location.search).has('mjpeg');
let v;
if (mjpeg) {
  v = document.createElement('img'); v.src = '/stream';
  v.onerror = () => setTimeout(() => { v.src = '/stream?' + Date.now(); }, 1000);
  document.getElementById('mode').innerHTML = 'MJPEG &middot; <a href="/">switch to H.264</a>';
} else {
  v = document.createElement('iframe');
  v.src = `http://${location.hostname}:8889/cam/?controls=false&muted=true&autoplay=true&playsinline=true`;
  v.allow = 'autoplay';
  document.getElementById('mode').innerHTML = 'H.264 / WebRTC &middot; <a href="/?mjpeg">switch to MJPEG</a>';
}
v.id = 'v'; document.body.prepend(v);

const hud=document.getElementById('hud'), c=document.getElementById('hz').getContext('2d');
function horizon(roll,pitch){
  const r=70; c.clearRect(0,0,140,140); c.save();
  c.beginPath(); c.arc(r,r,r-2,0,2*Math.PI); c.clip(); c.translate(r,r); c.rotate(-roll*Math.PI/180);
  const off=Math.max(-r,Math.min(r,pitch*1.5));
  c.fillStyle='#3a6ea5'; c.fillRect(-2*r,-2*r,4*r,2*r+off);
  c.fillStyle='#7a5a32'; c.fillRect(-2*r,off,4*r,2*r);
  c.strokeStyle='#fff'; c.beginPath(); c.moveTo(-2*r,off); c.lineTo(2*r,off); c.stroke(); c.restore();
  c.strokeStyle='#ff0'; c.lineWidth=2; c.beginPath(); c.moveTo(r-30,r); c.lineTo(r+30,r); c.stroke(); c.lineWidth=1;
}
const f=(x,n=1)=>(x>=0?' ':'')+x.toFixed(n);
async function poll(){
  try{
    const d=await (await fetch('/imu')).json();
    if(!d.available){hud.textContent='no IMU';}
    else{
      const [ax,ay,az]=d.accel, [gx,gy,gz]=d.gyro, g=Math.hypot(ax,ay,az)/9.81;
      hud.textContent=`roll  ${f(d.roll)} deg\\npitch ${f(d.pitch)} deg\\n\\ngyro deg/s\\n x ${f(gx)}  y ${f(gy)}  z ${f(gz)}\\n\\naccel m/s2  (${g.toFixed(2)} g)\\n x ${f(ax,2)}  y ${f(ay,2)}  z ${f(az,2)}`;
      horizon(d.roll,d.pitch);
    }
  }catch(e){hud.textContent='disconnected';}
  setTimeout(poll,200);
}
poll();
const st=document.getElementById('state'), colors={ARMED:'#f55',READY:'#5d5',REARM:'#fc3',FAULT:'#fc3',OFFLINE:'#999'};
async function pollState(){
  let d;
  try{ d=await (await fetch('/state')).json(); }catch(e){ d={state:'OFFLINE',faults:['no connection']}; }
  st.style.color=colors[d.state]||'#eee';
  st.textContent=d.state+(d.recording?'  \\u25cf REC':'');
  const lines=[...(d.faults||[])];
  if(d.state==='REARM') lines.push('release R1');
  for(const [m,v] of Object.entries(d.vesc||{})) if(v) lines.push(`${m} ${v.erpm} erpm  ${v.v_in.toFixed(1)} V  ${v.current_in.toFixed(1)} A`);
  if(lines.length){ const s=document.createElement('small'); s.textContent='\\n'+lines.join('\\n'); st.append(s); }
  setTimeout(pollState,250);
}
pollState();
</script></body></html>"""

# ---------- HTTP server ----------
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?")[0]
        if path not in ("/imu", "/state"):
            print(f"request {path} from {self.client_address[0]}", flush=True)
        if path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE)
        elif path == "/imu":
            with imu_lock:
                body = json.dumps(imu).encode()
            self._send(200, "application/json", body)
        elif path == "/state":
            self._send(200, "application/json", json.dumps(control_snapshot()).encode())
        elif path == "/stream":
            self._mjpeg()
        else:
            self._send(404, "text/plain", b"not found")

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _mjpeg(self):
        global mjpeg_clients
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with mjpeg_lock:
            mjpeg_clients += 1
        last_id = -1
        try:
            while not stop.is_set():
                with cond:
                    cond.wait_for(lambda: frame_id != last_id or stop.is_set(), timeout=1)
                    jpg, last_id = latest_jpg, frame_id
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    bump("mjpeg")
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with mjpeg_lock:
                mjpeg_clients -= 1

    def log_message(self, *a): pass

# ---------- main ----------
def main():
    ts_ip = subprocess.check_output(["tailscale", "ip", "-4"]).decode().split()[0]
    server = ThreadingHTTPServer((ts_ip, PORT), Handler)
    server.daemon_threads = True
    cam = threading.Thread(target=capture)
    cam.start()
    recorder.start()
    threading.Thread(target=report, daemon=True).start()
    threading.Thread(target=listen_control, daemon=True).start()

    def shutdown(*_):
        stop.set()
        threading.Thread(target=server.shutdown).start()
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"open http://{ts_ip}:{PORT}/   (MJPEG fallback: /?mjpeg)")
    server.serve_forever()
    cam.join()
    recorder.join()

if __name__ == "__main__":
    main()
