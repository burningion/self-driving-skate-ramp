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
read the controller's raw report stream (its "still in range" check).

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
uv pip install --python .venv/bin/python "rerun-sdk==0.42.0" opencv-python-headless
```

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

- **In a browser:** `./view_run.sh` on the Jetson (newest run, or pass a
  file), then open the URL it prints. Chrome works best.
- **On another machine:** copy the file over, then
  `pip install rerun-sdk==0.42.0 && rerun FILE.rrd`. Better for long runs.
