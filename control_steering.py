#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "pyserial>=3.5",
#     "pyPS4Controller>=1.2",
# ]
# ///
"""
Ramp controller: PS4 controller -> two VESCs + Teensy actuator board.

Hardware layout
---------------
  Jetson --USB--> back VESC --CAN--> front VESC      (set by USB_MOTOR / --usb-motor)
  Jetson --USB--> Teensy (actuators, U/D/L/R)

The CAN-only VESC is commanded through the USB one (COMM_FORWARD_CAN), so it
only needs the one CAN link between the two VESCs.

Autodetection (on startup)
--------------------------
  * VESC:     probes USB serial ports for a VESC (asks for its firmware version)
  * CAN VESC: asks the USB VESC to ping its CAN bus and uses the VESC it finds
  * Teensy:   USB vendor ID 0x16C0 (PJRC), else the one leftover serial port
  * Joystick: first /dev/input/js*
Any of these can be forced with command-line options. Use --detect to just
print what was found and exit.

How the control works
---------------------
* Controller callbacks ONLY record what is being held.
* A control loop runs at CONTROL_HZ and every tick computes the target speed,
  ramps toward it, and sends it to both VESCs (a steady stream, so the VESC's
  own timeout acts as a watchdog if this script dies).
* L1 / L2 raise / lower the speed of everything in 10% steps.
* R1 is a dead-man switch. Releasing it stops everything immediately.

Arming and faults
-----------------
Every tick the loop is in one of four states:

  FAULT    something is wrong (listed in the log): controller gone or out of
           range, VESC link/telemetry lost, a VESC fault code, Teensy silent.
  REARM    the fault cleared but R1 is still held: release it.
  READY    all good, R1 released.
  ARMED    R1 held. Only reachable from READY, and only if nothing else is
           pressed at the moment R1 goes down.

Only ARMED moves anything. The Teensy has its own watchdog (outputs drop if
this script stops talking to it) which is tested at startup, and the VESCs
have theirs (App Settings > General > Timeout -- keep it at 300-500 ms with a
non-zero timeout brake current, on BOTH VESCs).

The loop also sends its full state as JSON over UDP to STATE_ADDR every tick,
for the camera page and the run recorder.

Run:  uv run control_steering.py        (or ./control_steering.py after chmod +x)
"""

import argparse
import glob
import json
import logging
import os
import queue
import select
import signal
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger("ramp")

# =============================================================================
# Configuration
# =============================================================================

MOTORS = ("front", "back")

# Which motor's VESC is plugged into the Jetson's USB. The other one is reached
# over CAN through it. (Front VESC's USB port is damaged, so it's the back one.)
USB_MOTOR = "back"

# Speeds are ERPM (electrical RPM), the same units pyvesc's set_rpm used.
FRONT_DRIVE_RPM = 3100
BACK_DRIVE_RPM = 2900
FRONT_SPIN_RPM = 3300
BACK_SPIN_RPM = 3300

# What each face button does. If several are held, the most recently pressed
# wins. This matches the old script (both motors got FRONT_DRIVE_RPM); swap in
# BACK_DRIVE_RPM / *_SPIN_RPM here if you want them.
BUTTON_ACTIONS = {
    "x":        {"front": FRONT_DRIVE_RPM,  "back": FRONT_DRIVE_RPM},
    "circle":   {"front": FRONT_DRIVE_RPM,  "back": FRONT_DRIVE_RPM},
    "square":   {"front": -FRONT_DRIVE_RPM, "back": -FRONT_DRIVE_RPM},
    "triangle": {"front": -FRONT_DRIVE_RPM, "back": -FRONT_DRIVE_RPM},
}

# Left stick (vertical) = proportional speed when no face button is held.
STICK_MAX_RPM = {"front": FRONT_DRIVE_RPM, "back": FRONT_DRIVE_RPM}
STICK_DEADZONE = 0.12          # fraction of full travel
STICK_INVERT = False           # False = stick DOWN drives forward (old behaviour)

# Live speed adjustment: L1 = faster, L2 = slower. Scales every button and stick
# speed above. Starts at SPEED_START each run.
SPEED_START = 1.0              # 100% = the ERPMs above
SPEED_STEP = 0.10              # 10% per press
SPEED_MIN = 0.10
SPEED_MAX = 1.50
MAX_ERPM = 6000                # hard ceiling on any command, whatever the speed setting

CONTROL_HZ = 50                # VESC App timeout should be ~10x this period (250-500 ms)
RAMP_ERPM_PER_S = 8000         # accel/decel limit; 0..3100 in ~0.4 s. None = no ramp.

# What to send once a motor has ramped down to 0:
#   "rpm"   - hold 0 ERPM actively (what the old script did; regenerates while braking)
#   "coast" - 0 A, motor freewheels (use this on a bench power supply)
#   "brake" - hold BRAKE_CURRENT_A of braking current
STOP_MODE = "rpm"
BRAKE_CURRENT_A = 10.0

TEENSY_FW_VERSION = 2          # must match FW_VERSION in control_steering.ino
TEENSY_HEARTBEAT_TIMEOUT_S = 0.25   # must match HEARTBEAT_TIMEOUT_MS in the firmware

TELEMETRY_STALE_S = 0.5        # VESC / Teensy status older than this is a fault
CONTROLLER_STALE_S = 0.3       # controller sensor stream silent this long = out of range

STATE_ADDR = ("127.0.0.1", 47800)   # loop state goes here as JSON over UDP, every tick

# USB IDs used for autodetection
STM32_VCP = (0x0483, 0x5740)   # most VESCs
PJRC_VID = 0x16C0              # Teensy

# =============================================================================
# VESC serial protocol
# =============================================================================

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34
COMM_PING_CAN = 62

FAULTS = {0: "none", 1: "over voltage", 2: "under voltage", 3: "DRV", 4: "abs over current",
          5: "FET over temp", 6: "motor over temp"}


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_packet(payload):
    n = len(payload)
    head = bytes([2, n]) if n <= 255 else bytes([3, n >> 8, n & 0xFF])
    return head + payload + struct.pack(">H", crc16(payload)) + b"\x03"


class PacketParser:
    """Turns a byte stream into VESC packet payloads, resyncing on garbage."""

    MAX_LEN = 2048

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            # drop bytes until a start byte
            i = 0
            while i < len(self.buf) and self.buf[i] not in (2, 3):
                i += 1
            del self.buf[:i]
            if len(self.buf) < 2:
                break
            if self.buf[0] == 2:
                hl, n = 2, self.buf[1]
            else:
                if len(self.buf) < 3:
                    break
                hl, n = 3, (self.buf[1] << 8) | self.buf[2]
                if n > self.MAX_LEN:
                    del self.buf[0]
                    continue
            total = hl + n + 3
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[hl:hl + n])
            crc = (self.buf[hl + n] << 8) | self.buf[hl + n + 1]
            if self.buf[total - 1] == 3 and crc16(payload) == crc:
                out.append(payload)
                del self.buf[:total]
            else:
                del self.buf[0]   # false start byte; resync
        return out


VALUES_FMT = ">hhiiiihihiiiiiiBiB"   # COMM_GET_VALUES layout (fw 3.x and later)
VALUES_LEN = struct.calcsize(VALUES_FMT)


def parse_values(payload):
    body = payload[1:]
    if len(body) < VALUES_LEN:
        return None
    f = struct.unpack(VALUES_FMT, body[:VALUES_LEN])
    return {
        "temp_fet": f[0] / 10, "temp_motor": f[1] / 10,
        "current_motor": f[2] / 100, "current_in": f[3] / 100,
        "duty": f[6] / 1000, "erpm": f[7], "v_in": f[8] / 10,
        "tach": f[13], "tach_abs": f[14],
        "fault": f[15], "vesc_id": f[17],
    }


class VescUsb:
    """
    A VESC on USB serial, plus the VESCs behind it on CAN.
    can_id=None means the USB-connected VESC itself.
    Reconnects automatically (by USB serial number) if the link drops.
    """

    def __init__(self, path, usb_serial=None):
        import serial
        self._serial_mod = serial
        self.path = path
        self.usb_serial = usb_serial
        self.ser = None
        self._lock = threading.RLock()
        self._parser = PacketParser()
        self._replies = queue.Queue()
        self._last_open_try = 0.0
        self._last_err_log = 0.0
        self.reconnects = 0          # bumps every time the link comes back
        self.values = {}             # vesc_id -> (t, values dict)
        self._stop = threading.Event()
        self._open()
        self._reader = threading.Thread(target=self._read_loop, daemon=True, name="vesc-rx")
        self._reader.start()

    # --- connection handling ---------------------------------------------------
    def _resolve_path(self):
        if not self.usb_serial:
            return self.path
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.serial_number == self.usb_serial:
                return p.device
        return self.path

    def _open(self):
        self.path = self._resolve_path()
        self.ser = self._serial_mod.Serial(self.path, 115200, timeout=0.05, write_timeout=0.05)
        self.ser.reset_input_buffer()

    def _mark_broken(self, err):
        with self._lock:
            if self.ser is not None:
                log.error("VESC USB link lost (%s) -- motors will stop", err)
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _ensure_open(self):
        if self.ser is not None:
            return True
        now = time.monotonic()
        if now - self._last_open_try < 1.0:
            return False
        self._last_open_try = now
        try:
            self._open()
        except Exception:
            return False
        self.reconnects += 1
        log.warning("VESC USB link restored on %s (re-arm with R1 to continue)", self.path)
        return True

    def _read_loop(self):
        while not self._stop.is_set():
            ser = self.ser
            if ser is None:
                time.sleep(0.1)
                continue
            try:
                data = ser.read(256)
            except Exception as e:
                self._mark_broken(e)
                continue
            if not data:
                continue
            for payload in self._parser.feed(data):
                if payload and payload[0] == COMM_GET_VALUES:
                    v = parse_values(payload)
                    if v:
                        self.values[v["vesc_id"]] = (time.monotonic(), v)
                else:
                    self._replies.put(payload)

    def _write(self, payload):
        with self._lock:
            if not self._ensure_open():
                return False
            try:
                self.ser.write(encode_packet(payload))
                return True
            except Exception as e:
                self._mark_broken(e)
                return False

    def close(self):
        self._stop.set()
        with self._lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    @property
    def connected(self):
        return self.ser is not None

    # --- commands ----------------------------------------------------------------
    @staticmethod
    def _wrap(can_id, payload):
        if can_id is None:
            return payload
        return bytes([COMM_FORWARD_CAN, can_id & 0xFF]) + payload

    def _cmd_i32(self, can_id, cmd, value):
        self._write(self._wrap(can_id, struct.pack(">Bi", cmd, int(round(value)))))

    def set_rpm(self, can_id, erpm):
        self._cmd_i32(can_id, COMM_SET_RPM, erpm)

    def set_duty(self, can_id, duty):
        self._cmd_i32(can_id, COMM_SET_DUTY, duty * 100_000)

    def set_current(self, can_id, amps):
        self._cmd_i32(can_id, COMM_SET_CURRENT, amps * 1000)

    def set_brake_current(self, can_id, amps):
        self._cmd_i32(can_id, COMM_SET_CURRENT_BRAKE, amps * 1000)

    def request_values(self, can_id):
        self._write(self._wrap(can_id, bytes([COMM_GET_VALUES])))

    # --- queries (used at startup, before the control loop runs) -----------------
    def _query(self, payload, expect, timeout):
        while not self._replies.empty():
            self._replies.get_nowait()
        if not self._write(payload):
            return None
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                reply = self._replies.get(timeout=remaining)
            except queue.Empty:
                return None
            if reply and reply[0] == expect:
                return reply

    def fw_version(self, can_id=None, timeout=0.5):
        r = self._query(self._wrap(can_id, bytes([COMM_FW_VERSION])), COMM_FW_VERSION, timeout)
        if not r or len(r) < 3:
            return None
        name = r[3:].split(b"\0", 1)[0].decode(errors="replace") if len(r) > 3 else ""
        return {"major": r[1], "minor": r[2], "hw": name}

    def ping_can(self, timeout=5.0):
        """CAN IDs of every VESC the USB one can see on its CAN bus."""
        r = self._query(bytes([COMM_PING_CAN]), COMM_PING_CAN, timeout)
        return list(r[1:]) if r else None


class FakeVesc:
    """Logs commands instead of sending them (--dry-run)."""

    connected = True

    reconnects = 0

    def __init__(self):
        self.values = {}

    def _log(self, what, can_id, v):
        log.debug("VESC %s %s %.1f", "usb" if can_id is None else f"can{can_id}", what, v)

    def set_rpm(self, c, v):           self._log("rpm", c, v)
    def set_duty(self, c, v):          self._log("duty", c, v)
    def set_current(self, c, v):       self._log("current", c, v)
    def set_brake_current(self, c, v): self._log("brake", c, v)
    def close(self):                   pass

    def request_values(self, c):
        vid = 0 if c is None else c
        self.values[vid] = (time.monotonic(), {
            "temp_fet": 0.0, "temp_motor": 0.0, "current_motor": 0.0, "current_in": 0.0,
            "duty": 0.0, "erpm": 0, "v_in": 0.0, "tach": 0, "tach_abs": 0, "fault": 0, "vesc_id": vid})

# =============================================================================
# Teensy actuator board (protocol described in control_steering.ino)
# =============================================================================

TEENSY_CMD, TEENSY_STATUS, TEENSY_STATUS_LEN = 0xA5, 0x5A, 8


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_teensy_cmd(steer, jack, seq):
    body = struct.pack("<bbBB", steer, jack, seq & 0xFF, 0)
    return bytes([TEENSY_CMD]) + body + bytes([crc8(body)])


def parse_teensy_status(buf):
    """Consumes complete status packets from a bytearray; returns the newest, or None."""
    latest = None
    n = TEENSY_STATUS_LEN
    while len(buf) >= n:
        if buf[0] == TEENSY_STATUS and crc8(buf[1:n - 1]) == buf[n - 1]:
            version, steer, jack, flags, seq, trips = struct.unpack("<BbbBBB", buf[1:n - 1])
            latest = {"version": version, "steer": steer, "jack": jack, "seq": seq,
                      "timed_out": bool(flags & 1), "steer_capped": bool(flags & 2),
                      "jack_capped": bool(flags & 4), "wd_trips": trips}
            del buf[:n]
        else:
            del buf[0]
    return latest


class Teensy:
    """
    send() must be called every control tick: the stream is the board's
    heartbeat. Never blocks for long; reopens the port if the link drops.
    """

    def __init__(self, port):
        import serial
        self._serial_mod = serial
        self.port = port
        self.ser = None
        self.status = None           # newest status dict from the board
        self._status_at = 0.0
        self._seq = 0
        self._lock = threading.Lock()
        self._last_open_try = 0.0
        self._stop = threading.Event()
        self._open()
        threading.Thread(target=self._read_loop, daemon=True, name="teensy-rx").start()

    def _open(self):
        if not os.path.exists(self.port):       # the port name can change after a replug
            pjrc = [p.device for p in serial_ports() if p.vid == PJRC_VID]
            if pjrc:
                self.port = pjrc[0]
        self.ser = self._serial_mod.Serial(self.port, 115200, timeout=0.05, write_timeout=0.05)

    def _drop(self, err):
        with self._lock:
            if self.ser is not None:
                log.error("Teensy link lost (%s)", err)
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _read_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            ser = self.ser
            if ser is None:
                time.sleep(0.1)
                continue
            try:
                data = ser.read(64)
            except Exception as e:
                self._drop(e)
                continue
            if data:
                buf += data
                st = parse_teensy_status(buf)
                if st:
                    self.status, self._status_at = st, time.monotonic()

    def send(self, steer, jack):
        if self.ser is None:
            now = time.monotonic()
            if now - self._last_open_try < 1.0:
                return
            self._last_open_try = now
            try:
                self._open()
            except Exception:
                return
            log.warning("Teensy link restored on %s", self.port)
        self._seq += 1
        try:
            self.ser.write(encode_teensy_cmd(steer, jack, self._seq))
        except Exception as e:
            self._drop(e)

    def fresh(self):
        return self.status is not None and time.monotonic() - self._status_at < TELEMETRY_STALE_S

    def selftest(self):
        """
        Checks the firmware version and that the board's watchdog really trips
        when the heartbeat stops. Commands only "off". Returns None if OK,
        otherwise a description of the problem.
        """
        def heartbeat(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                self.send(0, 0)
                time.sleep(0.02)

        heartbeat(0.3)
        if not self.fresh():
            return (f"no status packets from the Teensy -- is firmware v{TEENSY_FW_VERSION} "
                    "(control_steering/control_steering.ino) flashed?")
        st = self.status
        if st["version"] != TEENSY_FW_VERSION:
            return f"Teensy firmware is v{st['version']}, this script needs v{TEENSY_FW_VERSION}"
        if st["timed_out"]:
            return "Teensy is not acknowledging the heartbeat"
        time.sleep(TEENSY_HEARTBEAT_TIMEOUT_S + 0.15)      # go silent: the watchdog must trip
        st = self.status
        if not self.fresh():
            return "Teensy stopped sending status during the watchdog test"
        if not st["timed_out"] or st["steer"] or st["jack"]:
            return "Teensy watchdog did not trip when the heartbeat stopped"
        heartbeat(0.15)
        if self.status["timed_out"]:
            return "Teensy did not recover after the watchdog test"
        return None

    def close(self):
        self._stop.set()
        with self._lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None


class NullTeensy:
    """Stands in when there is no board (--no-teensy, --dry-run)."""

    def __init__(self):
        self.status = None
        self._sent = (0, 0)

    def send(self, steer, jack):
        if (steer, jack) != self._sent:
            log.info("Teensy steer=%+d jack=%+d", steer, jack)
            self._sent = (steer, jack)

    def fresh(self):
        return True

    def selftest(self):
        return None

    def close(self):
        pass


# =============================================================================
# Controller heartbeat
# =============================================================================


class ControllerHeartbeat(threading.Thread):
    """
    The joystick interface only reports changes, so if the radio link drops
    while R1 is held nothing says so until the kernel gives up on the
    controller. Underneath, though, the controller sends raw HID reports
    continuously whether or not anything changed. This watches that raw stream
    (hidraw): silence means the controller is out of reach.

    Needs read access to the controller's hidraw device (udev/99-skate-ramp.rules).
    If it can't be read, `available` stays False and nothing is checked.
    """

    def __init__(self):
        super().__init__(daemon=True, name="pad-heartbeat")
        self.available = False
        self._last = 0.0
        self._warned = False

    @staticmethod
    def _find():
        for uevent in glob.glob("/sys/class/hidraw/hidraw*/device/uevent"):
            try:
                with open(uevent) as f:
                    if ":0000054C:" in f.read().upper():        # Sony's vendor ID, USB or Bluetooth
                        return "/dev/" + uevent.split("/")[4]
            except OSError:
                pass
        return None

    def run(self):
        while True:
            path = self._find()
            fd = None
            if path:
                try:
                    fd = os.open(path, os.O_RDONLY)
                except PermissionError:
                    if not self._warned:
                        self._warned = True
                        log.warning("can't read %s, so a controller that goes out of range won't be "
                                    "noticed quickly. Install udev/99-skate-ramp.rules to fix.", path)
                except OSError:
                    pass
            if fd is None:
                self.available = False
                time.sleep(1.0)
                continue
            self._last = time.monotonic()
            self.available = True
            try:
                while True:
                    readable, _, _ = select.select([fd], [], [], 1.0)
                    if readable:
                        if not os.read(fd, 4096):
                            break
                        self._last = time.monotonic()
            except OSError:
                pass                      # device went away; look for it again
            finally:
                self.available = False
                os.close(fd)

    def stale(self):
        return self.available and time.monotonic() - self._last > CONTROLLER_STALE_S

# =============================================================================
# Autodetection
# =============================================================================


def serial_ports():
    from serial.tools import list_ports
    return [p for p in list_ports.comports()
            if p.device.startswith(("/dev/ttyACM", "/dev/ttyUSB"))]


def describe(p):
    vid = f"{p.vid:04x}:{p.pid:04x}" if p.vid is not None else "?:?"
    return f"{p.device} [{vid}] {p.product or ''}".strip()


def find_vesc(ports, forced=None):
    """Returns (VescUsb, fw, port) for the first port that answers like a VESC."""
    if forced:
        candidates = [p for p in ports if p.device == forced] or [_FakePort(forced)]
    else:
        # likely VESCs first, never probe a Teensy
        candidates = sorted((p for p in ports if p.vid != PJRC_VID),
                            key=lambda p: (p.vid, p.pid) != STM32_VCP)
    for p in candidates:
        likely = forced or (p.vid, p.pid) == STM32_VCP
        try:
            v = VescUsb(p.device, usb_serial=getattr(p, "serial_number", None))
        except Exception as e:
            (log.error if likely else log.debug)("can't open %s: %s", p.device, e)
            if likely and "ermission" in str(e):
                log.error("  -> add yourself to the dialout group: sudo usermod -aG dialout $USER "
                          "(then log out and back in)")
            continue
        fw = None
        for attempt in range(3):            # the VESC can miss the first packet after the port opens
            fw = v.fw_version(timeout=1.0)
            if fw:
                break
        if fw:
            return v, fw, p
        if likely:
            log.error("%s opened but didn't answer as a VESC. Is VESC Tool or another script "
                      "connected to it? Is ModemManager running (systemctl status ModemManager)?",
                      p.device)
        v.close()
    return None, None, None


def find_teensy(ports, vesc_port, forced=None):
    if forced:
        return forced
    pjrc = [p for p in ports if p.vid == PJRC_VID]
    if pjrc:
        return pjrc[0].device
    leftovers = [p for p in ports if vesc_port is None or p.device != vesc_port.device]
    if len(leftovers) == 1:
        return leftovers[0].device
    return None


def find_can_id(vesc, forced=None):
    ids = vesc.ping_can()
    if ids is None:
        log.warning("CAN ping got no reply (old VESC firmware?)")
        ids = []
    log.info("VESCs on CAN behind the USB one: %s", ids or "none")
    if forced is not None:
        if forced not in ids:
            log.warning("CAN id %d didn't answer the ping; using it anyway", forced)
        return forced
    if not ids:
        return None
    if len(ids) > 1:
        log.warning("several VESCs on CAN %s; using %d (override with --can-id)",
                    ids, ids[0])
    return ids[0]


def find_joystick(forced=None):
    if forced:
        return forced
    js = sorted(glob.glob("/dev/input/js*"))
    return js[0] if js else "/dev/input/js0"   # listen() will wait for it to appear


class _FakePort:
    def __init__(self, device):
        self.device, self.vid, self.pid, self.product, self.serial_number = device, None, None, "", None

# =============================================================================
# Shared input state
# =============================================================================


@dataclass
class Inputs:
    connected: bool = False
    safety: bool = False                       # R1 held
    held: list = field(default_factory=list)   # face buttons, most recent last
    stick: float = 0.0                         # -1..1 after deadzone, + = forward
    vertical: int = 0                          # D-pad: +1 up, -1 down
    horizontal: int = 0                        # D-pad: +1 right, -1 left
    speed: float = SPEED_START                 # live speed scale (L1/L2)


class SharedInputs:
    def __init__(self):
        self._lock = threading.Lock()
        self._s = Inputs()

    def update(self, fn):
        with self._lock:
            fn(self._s)

    def snapshot(self):
        with self._lock:
            s = self._s
            return Inputs(s.connected, s.safety, list(s.held), s.stick, s.vertical, s.horizontal, s.speed)


def apply_deadzone(raw):
    """raw: pyPS4Controller axis value (-32767..32767) -> -1..1 with deadzone."""
    x = max(-1.0, min(1.0, raw / 32767.0))
    if abs(x) < STICK_DEADZONE:
        return 0.0
    mag = (abs(x) - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)
    return mag if x > 0 else -mag

# =============================================================================
# Control logic
# =============================================================================


def is_neutral(s):
    return not (s.held or s.stick or s.vertical or s.horizontal)


def compute_targets(s):
    """What the inputs ask for while armed: ({motor: erpm}, steer, jack)."""
    if s.held:
        rpm = {m: 0 for m in MOTORS}
        rpm.update(BUTTON_ACTIONS[s.held[-1]])
    elif s.stick:
        rpm = {m: s.stick * STICK_MAX_RPM.get(m, 0) for m in MOTORS}
    else:
        rpm = {m: 0 for m in MOTORS}
    rpm = {m: max(-MAX_ERPM, min(MAX_ERPM, v * s.speed)) for m, v in rpm.items()}
    return rpm, s.horizontal, s.vertical      # steer: +1 right; jack: +1 up


def step_toward(current, target, max_delta):
    if max_delta is None:
        return target
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


class ControlLoop(threading.Thread):
    def __init__(self, inputs, vesc, teensy, can_ids, heartbeat=None, checks=True, telemetry=False):
        """can_ids: {motor: None for the USB VESC, or its CAN id}. Motors not in it are skipped."""
        super().__init__(daemon=True, name="control")
        self.inputs = inputs
        self.vesc = vesc
        self.teensy = teensy
        self.can_ids = can_ids
        self.heartbeat = heartbeat
        self.checks = checks
        self.telemetry = telemetry
        self.stop_event = threading.Event()
        self.commanded = {m: 0.0 for m in MOTORS}
        self._ready = False          # no faults and R1 has been seen released
        self._armed = False
        self._last_report = None
        self._seen_reconnects = vesc.reconnects
        self._last_telemetry = 0.0
        self._tick_n = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def run(self):
        period = 1.0 / CONTROL_HZ
        max_delta = None if RAMP_ERPM_PER_S is None else RAMP_ERPM_PER_S * period
        next_tick = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.tick(max_delta)
            except Exception:
                log.exception("control tick failed")
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def values(self, motor):
        """Fresh telemetry for a motor's VESC, or None."""
        can_id = self.can_ids[motor]
        on_can = {c for c in self.can_ids.values() if c is not None}
        now = time.monotonic()
        for vid, (t, v) in list(self.vesc.values.items()):
            mine = vid == can_id if can_id is not None else vid not in on_can
            if mine and now - t < TELEMETRY_STALE_S:
                return v
        return None

    def faults(self, s):
        out = []
        if not s.connected:
            out.append("controller not connected")
        elif self.heartbeat and self.heartbeat.stale():
            out.append("controller signal lost")
        if not self.vesc.connected:
            out.append("VESC USB link down")
        elif self.vesc.reconnects != self._seen_reconnects:
            self._seen_reconnects = self.vesc.reconnects
            out.append("VESC USB link restarted")      # one tick of fault forces a re-arm
        elif self.checks:
            for m in self.can_ids:
                v = self.values(m)
                if v is None:
                    out.append(f"no telemetry from {m} VESC")
                elif v["fault"]:
                    out.append(f"{m} VESC fault: {FAULTS.get(v['fault'], v['fault'])}")
        if self.checks and not self.teensy.fresh():
            out.append("Teensy not responding")
        return out

    def tick(self, max_delta):
        s = self.inputs.snapshot()
        faults = self.faults(s)

        if faults:
            self._ready = self._armed = False
        elif not s.safety:
            self._ready, self._armed = True, False
        elif self._ready and not self._armed and is_neutral(s):
            self._armed = True

        if self._armed:
            state = "ARMED"
            targets, steer, jack = compute_targets(s)
        else:
            targets, steer, jack = {m: 0 for m in MOTORS}, 0, 0
            if faults:
                state = "FAULT"
            elif not self._ready:
                state = "REARM"
            else:
                state = "READY"

        report = {"FAULT": "FAULT: " + "; ".join(faults),
                  "REARM": "REARM: release R1",
                  "READY": "READY: let go of everything else, then hold R1" if s.safety
                           else "READY (hold R1 to arm)",
                  "ARMED": "ARMED"}[state]
        if report != self._last_report:
            log.info("%s", report)
            self._last_report = report

        for m, can_id in self.can_ids.items():
            if self._armed:
                self.commanded[m] = step_toward(self.commanded[m], targets[m], max_delta)
            else:
                self.commanded[m] = 0.0                    # stop now, no ramp
            self._send_motor(can_id, self.commanded[m])

        self.teensy.send(steer, jack)

        # Ask one VESC per tick for its values (each gets polled at CONTROL_HZ / n).
        motors = list(self.can_ids)
        self.vesc.request_values(self.can_ids[motors[self._tick_n % len(motors)]])
        self._tick_n += 1

        self._publish(state, faults, s, steer, jack)

        if self.telemetry and time.monotonic() - self._last_telemetry > 1.0:
            self._last_telemetry = time.monotonic()
            self._print_telemetry()

    def _publish(self, state, faults, s, steer, jack):
        msg = {"t": time.time(), "state": state, "faults": faults,
               "r1": s.safety, "stick": s.stick, "held": s.held, "speed": s.speed,
               "steer": steer, "jack": jack,
               "erpm_cmd": {m: self.commanded[m] for m in self.can_ids},
               "vesc": {m: self.values(m) for m in self.can_ids},
               "teensy": self.teensy.status}
        try:
            self._sock.sendto(json.dumps(msg).encode(), STATE_ADDR)
        except OSError:
            pass

    def _send_motor(self, can_id, erpm):
        if erpm != 0:
            self.vesc.set_rpm(can_id, erpm)
        elif STOP_MODE == "coast":
            self.vesc.set_current(can_id, 0)
        elif STOP_MODE == "brake":
            self.vesc.set_brake_current(can_id, BRAKE_CURRENT_A)
        else:
            self.vesc.set_rpm(can_id, 0)

    def _print_telemetry(self):
        now = time.monotonic()
        recent = [v for t, v in self.vesc.values.values() if now - t < 2.5]
        parts = []
        for m in self.can_ids:
            parts.append(f"{m}: cmd {self.commanded[m]:+6.0f}")
        for v in recent:
            fault = FAULTS.get(v["fault"], str(v["fault"]))
            parts.append(f"[id{v['vesc_id']} {v['erpm']:+6d} ERPM {v['current_motor']:+5.1f}A "
                         f"in {v['current_in']:+5.1f}A {v['v_in']:.1f}V {v['temp_fet']:.0f}C "
                         f"fault {fault}]")
        log.info(" ".join(parts))

    def shutdown(self):
        self.stop_event.set()
        if self.is_alive():
            self.join(timeout=1.0)
        for _ in range(5):
            for can_id in self.can_ids.values():
                self.vesc.set_current(can_id, 0)
            time.sleep(0.01)
        for _ in range(3):
            self.teensy.send(0, 0)
            time.sleep(0.01)

# =============================================================================
# PS4 controller -> shared inputs
# =============================================================================


def make_controller(inputs, **kwargs):
    from pyPS4Controller.controller import Controller

    def press(name):
        def fn(s):
            if name in s.held:
                s.held.remove(name)
            s.held.append(name)
        return fn

    def release(name):
        def fn(s):
            if name in s.held:
                s.held.remove(name)
        return fn

    def set_attr(attr, value):
        def fn(s):
            setattr(s, attr, value)
        return fn

    def change_speed(direction):
        def fn(s):
            new = round(min(SPEED_MAX, max(SPEED_MIN, s.speed + direction * SPEED_STEP)), 2)
            if new == s.speed:
                log.info("speed already at %s (%d%%)", "max" if direction > 0 else "min", round(new * 100))
                return
            s.speed = new
            x = BUTTON_ACTIONS["x"]
            log.info("speed %d%%  (X = %s ERPM)", round(new * 100),
                     ", ".join(f"{m} {min(MAX_ERPM, abs(v) * new):.0f}" for m, v in x.items()))
        return fn

    class RampController(Controller):
        _l2_down = False

        # --- live speed: L1 faster, L2 slower ---
        def on_L1_press(self):   inputs.update(change_speed(+1))
        def on_L1_release(self): pass

        # L2 is an analog trigger: it reports continuously while held, so only
        # count it once per pull.
        def on_L2_press(self, value):
            if not self._l2_down:
                self._l2_down = True
                inputs.update(change_speed(-1))

        def on_L2_release(self):
            self._l2_down = False

        # --- dead-man switch ---
        def on_R1_press(self):   inputs.update(set_attr("safety", True))
        def on_R1_release(self): inputs.update(set_attr("safety", False))

        # --- face buttons ---
        def on_x_press(self):          inputs.update(press("x"))
        def on_x_release(self):        inputs.update(release("x"))
        def on_circle_press(self):     inputs.update(press("circle"))
        def on_circle_release(self):   inputs.update(release("circle"))
        def on_square_press(self):     inputs.update(press("square"))
        def on_square_release(self):   inputs.update(release("square"))
        def on_triangle_press(self):   inputs.update(press("triangle"))
        def on_triangle_release(self): inputs.update(release("triangle"))

        # --- D-pad -> Teensy actuators ---
        def on_up_arrow_press(self):           inputs.update(set_attr("vertical", 1))
        def on_down_arrow_press(self):         inputs.update(set_attr("vertical", -1))
        def on_up_down_arrow_release(self):    inputs.update(set_attr("vertical", 0))
        def on_left_arrow_press(self):         inputs.update(set_attr("horizontal", -1))
        def on_right_arrow_press(self):        inputs.update(set_attr("horizontal", 1))
        def on_left_right_arrow_release(self): inputs.update(set_attr("horizontal", 0))

        # --- left stick (vertical) -> proportional speed ---
        # pyPS4Controller: up = negative values, down = positive values.
        def _stick(self, value):
            v = apply_deadzone(value)
            inputs.update(set_attr("stick", -v if STICK_INVERT else v))

        def on_L3_up(self, value):   self._stick(value)
        def on_L3_down(self, value): self._stick(value)
        def on_L3_y_at_rest(self):   inputs.update(set_attr("stick", 0.0))

        # silence the library's default printing for stick X
        def on_L3_left(self, value):  pass
        def on_L3_right(self, value): pass
        def on_L3_x_at_rest(self):    pass

    return RampController(**kwargs)

# =============================================================================
# Main
# =============================================================================


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vesc-port", help="force the VESC serial port (default: autodetect)")
    ap.add_argument("--usb-motor", choices=MOTORS, default=USB_MOTOR,
                    help=f"which motor's VESC is on USB (default: {USB_MOTOR})")
    ap.add_argument("--can-id", type=int, help="force the CAN-only VESC's id (default: autodetect)")
    ap.add_argument("--usb-only", action="store_true", help="run even if no CAN VESC is found")
    ap.add_argument("--teensy-port", help="force the Teensy serial port (default: autodetect)")
    ap.add_argument("--no-teensy", action="store_true", help="run without the Teensy")
    ap.add_argument("--joystick", help="force the joystick device (default: first /dev/input/js*)")
    ap.add_argument("--detect", action="store_true", help="print what autodetection finds and exit")
    ap.add_argument("--dry-run", action="store_true", help="no hardware; log commands with -v")
    ap.add_argument("--no-checks", action="store_true",
                    help="skip the Teensy self-test and the VESC/Teensy telemetry checks "
                         "(the controller and VESC-link checks always apply)")
    ap.add_argument("--telemetry", action="store_true", help="print VESC status once a second")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")

    # ---- find hardware ----
    usb_motor = args.usb_motor
    can_motor = next(m for m in MOTORS if m != usb_motor)

    if args.dry_run:
        vesc, teensy = FakeVesc(), NullTeensy()
        can_ids = {usb_motor: None, can_motor: args.can_id if args.can_id is not None else 1}
    else:
        ports = serial_ports()
        log.info("serial ports: %s", ", ".join(describe(p) for p in ports) or "none")

        vesc, fw, vport = find_vesc(ports, args.vesc_port)
        if not vesc:
            raise SystemExit("No VESC found on USB. Plugged in and powered? Try --vesc-port.")
        log.info("%s VESC: USB %s (%s, fw %d.%02d)", usb_motor, vesc.path, fw["hw"], fw["major"], fw["minor"])

        can_id = find_can_id(vesc, args.can_id)
        can_ids = {usb_motor: None}
        if can_id is not None:
            can_ids[can_motor] = can_id
            log.info("%s VESC: CAN id %d (via the %s VESC)", can_motor, can_id, usb_motor)
        elif args.usb_only:
            log.warning("no CAN VESC found; running the %s motor only", usb_motor)
        else:
            vesc.close()
            raise SystemExit(f"The {can_motor} VESC didn't answer on CAN. Check the CAN cable and that "
                             "both VESCs use the same CAN baud rate, or pass --can-id / --usb-only.")

        teensy_port = None if args.no_teensy else find_teensy(ports, vport, args.teensy_port)
        if not args.no_teensy and not teensy_port:
            vesc.close()
            raise SystemExit("No Teensy found. Pass --teensy-port, or --no-teensy to run without it.")
        log.info("teensy: %s", teensy_port or "disabled")

        joystick = find_joystick(args.joystick)
        log.info("joystick: %s", joystick)

        if args.detect:
            vesc.close()
            return
        teensy = NullTeensy() if args.no_teensy else Teensy(teensy_port)
        if not args.no_checks:
            problem = teensy.selftest()
            if problem:
                teensy.close()
                vesc.close()
                raise SystemExit(f"Teensy self-test failed: {problem}")
            if not args.no_teensy:
                log.info("teensy: firmware v%d, watchdog OK", TEENSY_FW_VERSION)

    joystick = find_joystick(args.joystick)

    # ---- run ----
    inputs = SharedInputs()
    heartbeat = None
    if not args.dry_run:
        heartbeat = ControllerHeartbeat()
        heartbeat.start()
    loop = ControlLoop(inputs, vesc, teensy, can_ids, heartbeat=heartbeat,
                       checks=not args.no_checks, telemetry=args.telemetry)
    loop.start()

    stopping = threading.Event()

    def on_signal(signum, frame):
        stopping.set()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    controller = make_controller(inputs, interface=joystick, connecting_using_ds4drv=False)

    def on_connect():
        log.info("controller connected")
        inputs.update(lambda s: setattr(s, "connected", True))

    def on_disconnect():
        log.warning("controller disconnected")

        def reset(s):
            s.connected = s.safety = False
            s.held.clear()
            s.stick, s.vertical, s.horizontal = 0.0, 0, 0
        inputs.update(reset)

    try:
        while not stopping.is_set():
            try:
                controller.listen(timeout=30, on_connect=on_connect, on_disconnect=on_disconnect)
            except (SystemExit, Exception) as e:
                if stopping.is_set():
                    break
                log.warning("controller listener ended (%r); retrying", e)
            on_disconnect()
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        log.info("shutting down")
        loop.shutdown()
        teensy.close()
        vesc.close()


if __name__ == "__main__":
    main()
