# Setup

How to get the ramp's software running on a fresh Jetson. Written against the
machine it was developed on: Jetson Orin Nano (JetPack 7, Ubuntu 24.04,
Python 3.12), run headless over Tailscale.

## What runs where

| Piece | What it does | How it starts |
|---|---|---|
| `mediamtx` | Serves the live video over WebRTC / RTSP | systemd user service, at boot |
| `camera.py` | Reads the RealSense (front) and the USB webcam (rear), feeds the live view, serves the phone page, records runs | systemd user service (`skate-camera`), at boot |
| `control_steering.py` | PS4 controller -> VESCs + Teensy. The only thing that moves the ramp | by hand, each session |
| Teensy firmware | Drives the steering and jack relays, with its own watchdog | flashed once |

Recording is tied to the control script: a run file is opened when
`control_steering.py` starts and closed a few seconds after it stops.
The phone page shows the name of the file being written (bottom left, under
`REC`) and, once it is closed, `saved <file>`. **stop recording** (right
side) closes the file while `control_steering.py` keeps running; the button
then becomes **start recording**, which opens a new file. Stopping by hand
only lasts for that session: the next start of `control_steering.py` records
again.

## Hardware assumed

- RealSense D455 on a USB 3 port, facing forwards.
- A USB webcam facing backwards (a Logitech C920 here). Any UVC webcam works;
  `camera.py` picks the first USB camera that is not the RealSense.
- Two VESCs. The back one is on USB, the front one is reached through it over CAN.
- Teensy 3.2 on USB. Relay pins: A9 steer right, A8 steer left, A6 jack up, A7 jack down.
- PS4 controller (DualShock 4), over USB or Bluetooth. Pairing notes are in
  [`ps4-controller.md`](ps4-controller.md).

## 1. Prerequisites

Install these first. The repo is expected at
`~/Development/self-driving-skate-ramp`; the service file uses that path.

- **librealsense with Python bindings**, installed system-wide so that
  `python3 -c "import pyrealsense2"` works. This project used a source build
  (2.59.0).
- **Tailscale**, logged in. `camera.py` binds its web page to the Tailscale
  address and exits if there isn't one.
- **uv** (`~/.local/bin/uv`), for the Python environments.
- **MediaMTX** unpacked in `~/mediamtx` (binary and `mediamtx.yml` side by
  side; v1.21.3 was used). `camera.py` publishes to
  `rtsp://127.0.0.1:8554/cam` and the phone page plays WebRTC from port 8889.
- **A static ffmpeg build** at
  `~/ffmpeg-static/ffmpeg-7.0.2-arm64-static/ffmpeg`. That path is hard-coded
  in `camera.py` (`H264Out._start`); change it there if yours differs.
- **teensy-loader-cli**: `sudo apt install teensy-loader-cli`
- **arduino-cli** with the Teensy core, for building the firmware:

  ```
  # put the arduino-cli binary from its GitHub releases page in ~/.local/bin, then:
  arduino-cli config init
  arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
  arduino-cli core update-index
  arduino-cli core install teensy:avr
  ```

## 2. Device permissions

One udev file covers flashing the Teensy without root, keeping ModemManager
away from the Teensy and VESC serial ports, and letting the control script
read the controller's raw HID reports. That is the only way it reads the
controller (buttons, stick, and the "still in range" check), so without this
rule it reports "controller not connected".

```
sudo cp udev/99-skate-ramp.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

Your user also needs to be in the `dialout` group for the VESC serial port:
`sudo usermod -aG dialout $USER`, then log out and back in.

## 3. Python environment for the camera

`camera.py` needs `rerun-sdk`, which needs numpy 2; the system OpenCV does
not work with numpy 2. So it runs from a venv that has its own numpy and
OpenCV and falls through to the system `pyrealsense2`:

```
uv venv --python /usr/bin/python3 --system-site-packages .venv
uv pip install --python .venv/bin/python "rerun-sdk==0.43.0" opencv-python-headless pandas
```

`pandas` is there for `export_run.py` (section 10): the system pandas does not
load with numpy 2 either, and pyarrow fails on it when writing tables.

Keep the Rerun version the same on any machine that opens the recordings.

`control_steering.py` needs no setup: it declares its own dependencies and
`uv run` installs them on first use.

## 4. Teensy firmware

With the Teensy plugged in and `control_steering.py` not running:

```
./control_steering/flash.sh
```

This builds `control_steering/control_steering.ino` and flashes it. Keep
actuator power off, or stay clear of the steering, while flashing. The
firmware and `control_steering.py` share a protocol version; the control
script checks it at startup and refuses to run against the wrong one.

## 5. VESC settings

Set these in VESC Tool, on both VESCs:

- **Controller IDs** must differ (here: back = 1, front = 0) and both must use
  the same CAN baud rate.
- **App Settings > General > Timeout**: 300-500 ms, with a non-zero timeout
  brake current. This is what stops the motors if the Jetson dies. With the
  brake current at 0 the motors coast instead of braking.

`uv run control_steering.py --detect` prints which VESCs, Teensy and
controller it finds, then exits without moving anything.

## 6. Services

Install the two user services and let them start at boot without a login:

```
mkdir -p ~/.config/systemd/user
cp systemd/*.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now mediamtx skate-camera
sudo loginctl enable-linger $USER
```

`skate-camera` restarts itself until Tailscale and the camera are up.

Day to day:

```
systemctl --user status skate-camera      # running? plus the last log lines
systemctl --user restart skate-camera     # after editing camera.py
systemctl --user stop skate-camera        # frees the camera for other tools
```

A healthy log line looks like
`video 30.0/s  rear 30.0/s  imu 450/s  h264 9.0/s ...` (`video` is the front
camera, `rear` the webcam). While recording, `rec` should
read 30 and `rec_drop` 0.

After changing a file in `systemd/`, copy it again and run
`systemctl --user daemon-reload`.

## 7. Driving

```
uv run control_steering.py --telemetry
```

At startup it finds the hardware and tests the Teensy's watchdog. Then:

- **READY**: hold R1, with nothing else pressed, to arm.
- **ARMED**: stick or face buttons drive, D-pad left/right steers, D-pad
  up/down moves the jack, L1/L2 change the speed scale. Release R1 to stop.
- **FAULT**: the log says why (controller lost, VESC link or telemetry lost,
  VESC fault, Teensy silent). Fix it, release R1, and arm again.

One continuous steering push is cut off after 3 seconds, because the
actuator has no limit switches. Release and press again to keep going.

`--no-checks` skips the Teensy self-test and the telemetry-based faults;
`--dry-run` runs with no hardware at all.

## 8. Live view

With the services running, from any device on the tailnet (find the address
with `tailscale ip -4`):

| URL | What |
|---|---|
| `http://<jetson>:8080/` | Phone page: video, IMU, control state, REC indicator |
| `http://<jetson>:8080/?mjpeg` | Same page with MJPEG video, if WebRTC won't connect |
| `http://<jetson>:8889/cam/` | Video only (WebRTC) |
| `rtsp://<jetson>:8554/cam` | Video only (VLC, ffplay) |

The live video is a downscaled, low-bitrate view for a phone hotspot: the
front camera on the left and the rear camera on the right, each labelled.
Depth is not shown live, but is still recorded. Settings are at the top of
`camera.py`, including `LIVE_VIEW` (front camera: `"ir"` or `"color"`),
`REAR_CAM` (`"auto"`, a device path, or `None`) and `EMITTER` (the IR dot
projector). If the rear camera is unplugged its half goes black and reads
`REAR (no camera)`; it comes back by itself when plugged in again.

### Exposure

While nothing is being recorded, exposure adjusts itself: `camera.py` sets the
rear camera and the front colour camera from their own pictures (aiming for
`AE_TARGET` brightness, but darker if more than `AE_HIGHLIGHT` of the picture
would be blown out), and the front infrared/depth pair uses the RealSense's
own auto-exposure. When a recording starts, all three are frozen where they
are, and released when it ends. The log and the recording (`camera/exposure`)
say what each was locked at.

So start `control_steering.py` with the ramp in the light it will be driven
in: if it starts in the garage and drives out into the sun, the run will be
overexposed. If the light does change mid-run, tap **reset exposure** on the
phone page (top right; greyed out when nothing is locked): exposure adjusts to the current light for
`AE_RESET_S` (3 seconds), then locks again, and the new values are logged.
Restarting `control_steering.py` does the same thing, as a new recording.
`LOCK_EXPOSURE = False` keeps exposure adjusting during runs.

If the RealSense stops sending pictures for 10 seconds, `camera.py` resets it
and exits, and systemd starts it again.

## 9. Recordings

Runs are written to `~/ramp-runs/<date>_<time>.rrd` at roughly 35 GB per
hour, and recording will not start with less than 20 GB free. Each file
holds front colour, infrared and depth and the rear camera (`camera/rear`),
all at 30 fps, raw IMU, the control loop's
state at 50 Hz (inputs, commanded speed, VESC telemetry, Teensy outputs),
state changes and faults, and the front camera's calibration.

To look at one:

- **In a browser:** `./view_run.sh` on the Jetson. It lists the runs, newest
  first; type a number (Enter takes the newest), or pass a file name to skip
  the list. Then open the URL it prints. Chrome works best.
- **On another machine:** copy the file over, then
  `pip install rerun-sdk==0.43.0 && rerun FILE.rrd`. Better for long runs.

## 10. Exporting runs for training

A `.rrd` is good for looking at a run, but awkward to train from. Two scripts
turn runs into plain files. Both run on the Jetson from the camera's venv, and
take run names the same way `view_run.sh` does (a bare name means a file in
`~/ramp-runs`).

**Check what a run holds** (a few seconds per run; it skips the pictures):

```
.venv/bin/python extract_telemetry.py                  # every run
.venv/bin/python extract_telemetry.py 2026-11-11_16-58-31
```

For each run it prints how long it drove forwards and backwards, how many
steering taps it holds, and how often the controller dropped out. Runs with
no driving or no steering are not worth exporting.

**Export:**

```
.venv/bin/python export_run.py 2026-11-11_16-58-31 2026-11-11_09-38-03
.venv/bin/python export_run.py --all                   # everything in ~/ramp-runs
```

Each run becomes a folder in `~/ramp-export` (`-o DIR` to write elsewhere),
about the size of its `.rrd`, at roughly 10 seconds per minute of recording.
`--skip ir depth` leaves those pictures out. A run that is already exported
is left alone unless you pass `--force`. A folder ending in `.partial` is an
export that was interrupted; it is replaced on the next try.

The export ends with a warning if the depth pictures are mostly empty or the
infrared is blown out, which is what happens when the RealSense's exposure was
locked wrong for the light. The colour and rear pictures of such a run are
still usable.

**Copy to the training machine**, for example:

```
rsync -a --info=progress2 ~/ramp-export/ <training-machine>:ramp-export/
```

### What is in an export

```
2026-11-11_16-58-31/
  color/000000.jpg ...     front colour, 640x480
  ir/000000.jpg ...        front left infrared, 640x480, greyscale
  depth/000000.png ...     front depth, 16-bit; value x depth_unit_m (in meta.json) = metres, 0 = no reading
  rear/000000.jpg ...      rear camera, 640x480
  frames.parquet           one row per front frame: the table to train from
  rear_frames.parquet      frame, t, file for every rear picture
  control.parquet          the control loop's state at 50 Hz, as recorded
  imu_gyro.parquet         raw gyro (rad/s), about 200 Hz
  imu_accel.parquet        raw accelerometer (m/s2), about 250 Hz
  calibration.json         the front camera's intrinsics and depth-to-colour transform
  events.json              state changes, faults, and the exposure each camera was locked at
  meta.json                counts, frame rate, motor order, and the picture health check
```

The pictures are the recorded bytes, not re-encoded. Colour, infrared and
depth share a frame number: `color/000123.jpg`, `ir/000123.jpg` and
`depth/000123.png` were taken together. The rear camera has its own clock and
its own numbering.

Every table has `t` (seconds since the epoch) and `t_ns` (the same in
nanoseconds, exact), on one clock across all of them.

`frames.parquet` columns:

| Column | Meaning |
|---|---|
| `frame`, `t`, `t_ns` | Front frame number and time |
| `color`, `depth`, `ir` | Picture paths, relative to the run folder |
| `rear`, `rear_dt` | The rear picture nearest in time, and how far off it is in seconds (under 0.035) |
| `state` | `READY`, `ARMED`, `REARM` or `FAULT`. Only `ARMED` rows can move the ramp |
| `r1`, `stick`, `speed` | Deadman held (0/1), stick position (-1..1), speed scale |
| `steer` | Steering command: -1 left, 0 none, +1 right |
| `jack` | Jack command: -1 down, 0 none, +1 up |
| `erpm_cmd_back`, `erpm_cmd_front` | Speed sent to each VESC (positive = forwards) |
| `vesc_<field>_<motor>` | VESC replies: `erpm`, `current_motor`, `current_in`, `v_in`, `duty`, `tach`, `temp_fet` |
| `teensy_steer`, `teensy_jack` | What the Teensy says it is doing; trails the command by a tick or two |
| `control_age` | How old the control values on this row are, in seconds (under 0.025) |
| `gyro_x/y/z`, `accel_x/y/z` | IMU averaged over the frame interval ending at `t` |

The control values on a row are the most recent ones at or before the frame,
never later ones. The first frame or two of a run can come before the first
control message, and those rows are empty. `control.parquet` has the same
control columns at the full 50 Hz, with empty VESC values on the rare tick
where no fresh reply had arrived.

The IMU is in the RealSense's own axes: x to the right, y down, z forwards. So
`gyro_y` is the turning rate, positive when turning right, and `accel_z` is
forward acceleration. The accelerometer is uncalibrated (gravity reads about
9.33).

### Reading an export

Reading needs only `pandas`, `pyarrow` and something to open pictures, not
Rerun:

```python
import cv2, pandas as pd

run = "ramp-export/2026-11-11_16-58-31"
frames = pd.read_parquet(f"{run}/frames.parquet")
driving = frames[(frames.state == "ARMED") & (frames.erpm_cmd_back.abs() > 200)]
row = driving.iloc[0]
color = cv2.imread(f"{run}/{row.color}")
depth_m = cv2.imread(f"{run}/{row.depth}", cv2.IMREAD_UNCHANGED) * 0.001    # depth_unit_m from meta.json
```

To run the export scripts themselves on another machine (from copied `.rrd`
files), it needs `rerun-sdk==0.43.0`, `pandas` and `opencv-python-headless`.

## 11. Finding the sidewalk in the pictures (training machine)

This turns exported runs into the numbers a steering controller needs: how
far the ramp is from the middle of the sidewalk and how far it is pointed off
the sidewalk's direction, for every labelled picture. It runs on the machine
with the GPU, from a clone of this repo, on the folders copied over in
section 10. Both scripts declare their own dependencies, so `uv run` sets
them up on first use.

**Step 1: label the sidewalk.** Needs the GPU.

```
uv run label_sidewalk.py ~/ramp-export/2026-11-11_16-58-31
uv run label_sidewalk.py ~/ramp-export/*/              # every run
```

This runs a pretrained street-scene segmentation model (Mask2Former trained
on Mapillary Vistas, about 900 MB, downloaded on first use) over one picture
in three from the front colour and rear cameras, and writes the result into
the run's `labels/` folder. `--every 1` labels every picture; `--half` is
faster. Stopping it and running it again carries on where it left off.

Then **look at `labels/preview_color.jpg` and `labels/preview_rear.jpg`**:
the area counted as sidewalk is tinted green. The camera sits much lower than
anything the model was trained on, so this is the step most likely to need
attention. The model's "Road" class is counted as sidewalk on purpose, since
that is the usual mistake from down there. If grass is being tinted, or the
sidewalk isn't, that has to be fixed (a different model, or other class
words) before the next step means anything.

**Step 2: turn labels into positions.** No GPU needed.

```
uv run sidewalk_geometry.py ~/ramp-export/*/
```

For each run it prints the camera pose it used, how many pictures gave a
result, and the sidewalk width it measured, and writes:

| File | What |
|---|---|
| `geometry.parquet` | One row per labelled front picture |
| `geometry_rear.parquet` | One row per labelled rear picture |
| `camera_pose.json` | Each camera's height and tilt, as used |
| `geometry_preview_color.jpg`, `geometry_preview_rear.jpg` | Pictures with the fitted edges (red) and horizon drawn on |

Columns, the same for both cameras and both in the ramp's own terms:

| Column | Meaning |
|---|---|
| `frame`, `t`, `t_ns` | Which picture (matches `frames.parquet` / `rear_frames.parquet`) |
| `offset_m` | Camera's distance from the sidewalk's centre line; positive = right of centre, facing forwards |
| `heading_rad` | Ramp's direction relative to the sidewalk; positive = turned clockwise seen from above |
| `width_m` | Sidewalk width from this picture (only when both edges were found) |
| `edges` | Edges found: 2, 1 (other side filled in from the usual width), or 0 (no result; the other columns are empty) |
| `left_edge_m`, `right_edge_m` | Where each edge is beside the camera |
| `edge_rms_px` | How well the edges fitted a straight line, in pixels |
| `why` | For pictures with no result, the reason (also totalled in what the script prints) |

**How to tell if it worked:**

- The sidewalk width should be the real width (measure it once) and nearly
  the same in every picture. A wide spread means bad labels or a wrong pose.
- When a run has both cameras, the last line compares them. They look at
  different ground, so agreement within a few centimetres and a degree or two
  is the two halves checking each other.
- The red lines in the preview should lie on the sidewalk's edges.

**Camera height and tilt.** For the front camera these are measured, by
fitting a plane to the ground in the depth pictures (19.4 cm and about 1
degree up, on the runs of 2026-11-11). A run with no usable depth can't do
that, so give it the pose from one that can:

```
uv run sidewalk_geometry.py ~/ramp-export/2026-11-11_09-38-03 --pose ~/ramp-export/2026-11-11_16-58-31/camera_pose.json
```

The rear camera has no depth and no calibration. Its tilt is worked out from
where the sidewalk's edges meet in its pictures, and its focal length is
assumed (`--rear-fx`). Its height is set so that the sidewalk comes out as
wide as the front camera measured it, since both look at the same sidewalk;
that needs at least 20 rear pictures showing both edges, and otherwise the
front camera's height is used. `--rear-height` (metres, to the middle of the
lens) overrides this. The offset depends only on the height; a wrong focal
length only skews the heading. The rear camera also uses the front camera's
sidewalk width to fill in the side it can't see. `--height` overrides the front camera's measured height, and
`--width` sets the sidewalk width instead of measuring it.

**Limits to know about:** the sidewalk is treated as straight and the ground
as flat, edges farther than 4 m ahead are ignored, and pictures where the
sidewalk meets a driveway or another path usually give no result
(`edges` = 0) rather than a wrong one.


## 12. Trying the self-steering on recorded runs

`autosteer.py` holds the self-steering: an estimate of where the ramp is on
the sidewalk (from the pictures' geometry and the gyro) and the decision to
tap the steering left or right. It does not touch hardware and nothing calls
it while driving yet. `steering_replay.py` tries it out on runs that have a
`geometry.parquet` (section 11). Only the tables are needed, so this runs on
the Jetson too:

```bash
uv run steering_replay.py ~/ramp-export/2026-11-11_16-58-31 ~/ramp-export/2026-11-11_09-44-21
```

It prints four things:

1. **fit**: how the ramp moves, worked out from the runs: speed per ERPM, and
   how fast the actuator changes the steer angle. Written to
   `steering_model.json` beside the runs (`--model FILE` reuses one instead).
2. **check**: the model's prediction 2 s ahead from the recorded taps alone,
   against what the camera then saw.
3. **replay**: the estimator run over each recording, and whether the taps the
   controller wanted match the driver's. `replay_<run>.png` in the run folder
   shows every drive: camera against estimate, steer angle, and both sets of taps.
4. **simulate**: the controller driving the fitted model from starts up to
   30 cm off centre and 8° off straight, with measurement errors and dropouts
   taken from the real pictures. `steering_sim.png` beside the runs shows them.
   `--sims 0` skips this.

Settings are the fields of `Params` in `autosteer.py`; try one without editing
with `--set settle_m=1.5`. The measured lengths are there too: the wheelbase
(33", axle to axle), with the front camera taken to be over the front, steered
axle.

**Limits to know about:** the simulation drives the same kind of model the
controller assumes, so it shows how the controller copes with noise, dropouts
and an actuator that is faster or slower than thought, not whether the model is
right; step 2 is the check on that. The sidewalk is straight in all of it. The
estimate uses the front camera in both directions.
