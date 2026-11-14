"""
Keeping the ramp on the sidewalk's centre line: the estimate of where it is,
and the decision to tap the steering left or right.

Nothing here talks to hardware. steering_replay.py runs it against recorded
runs and in simulation; the control loop can call the same two classes live.

    est, ctl = Estimator(p), Controller(p)
    every control tick (50 Hz):
        est.predict(dt, erpm, steer_sent_last_tick)
        est.gyro(yaw_rate)                          # rad/s, + = turning right
        est.vision(offset_m, heading_rad, edges)    # when a picture has been processed
        steer = ctl.step(est, dt)                   # -1 left, 0, +1 right

Signs, everywhere: offset + = right of the centre line (facing forwards),
heading + = turned clockwise seen from above, steer angle + = steering right,
speed + = forwards (the RealSense end leading).

The model behind it (L = axle to axle, v = speed, the camera over the front axle,
the front truck steered):
    steer angle    changes at steer_rate while the actuator is tapped
    heading        changes at v * steer / L
    offset         changes at v * (heading + steer)     (the front axle goes where its wheels point)
"""
from dataclasses import asdict, dataclass

import numpy as np

DEG = np.pi / 180


@dataclass
class Params:
    # the ramp, measured
    wheelbase_m: float = 0.84           # axle to axle (33")
    # the ramp, fitted from runs by steering_replay.py
    m_per_erpm: float = 3.9e-4          # speed in m/s per unit of VESC ERPM
    steer_rate: float = 0.15            # rad of steer angle per second of tapping
    tap_dead_s: float = 0.0             # start of each tap during which nothing moves yet

    # controller
    settle_m: float = 2.0               # distance scale of the correction: smaller = sharper
    damping: float = 1.0
    max_heading: float = 10 * DEG       # never aim more than this across the sidewalk
    max_steer: float = 8 * DEG
    deadband: float = 1.0 * DEG         # steer error not worth a tap
    min_tap_s: float = 0.06
    max_tap_s: float = 0.4
    tap_fraction: float = 0.8           # each tap aims to close this much of the steer error
    min_pause_s: float = 0.15           # between taps, for the turn to show in the gyro
    min_speed: float = 0.3              # m/s; below this, leave the steering alone

    # estimator: how much to trust what
    vision_offset_sd: float = 0.03      # m, with both edges seen
    vision_heading_sd: float = 1.5 * DEG
    one_edge_factor: float = 1.5        # one edge seen: this much less trusted
    gyro_sd: float = 0.06               # rad/s per control tick, driving (bumps included)
    drift_offset: float = 0.02          # per sqrt(second): how fast the truth wanders off the model
    drift_heading: float = 2.0 * DEG
    drift_steer: float = 1.0 * DEG
    tap_rate_error: float = 0.4         # the actuator's speed is only known this well (fraction)
    gate: float = 4.0                   # vision further than this many sd from the estimate is thrown out...
    gate_agree: int = 3                 # ...unless this many pictures in a row say the same thing
    max_offset: float = 1.0             # m; vision claiming more than this is never believed

    def to_json(self):
        return asdict(self)


class Estimator:
    """Offset, heading and steer angle (self.x), with how sure it is (self.P)."""

    def __init__(self, p):
        self.p = p
        self.x = np.zeros(3)
        self.P = np.diag([0.5 ** 2, (20 * DEG) ** 2, (10 * DEG) ** 2])
        self.v = 0.0
        self.gyro_bias = 0.0
        self._bias_n = 0
        self._tap = 0.0                 # how long the current tap has lasted
        self._thrown = []               # the pictures thrown out in a row, while they agree with each other
        self._t_vision = 0.0            # time since the last vision update that was used
        self.last = {}                  # last innovations, for looking at afterwards

    def predict(self, dt, erpm, steer):
        """Advance by one tick. steer is what was sent to the actuator over that tick (-1, 0, 1)."""
        p = self.p
        v = self.v = p.m_per_erpm * erpm
        self._tap = self._tap + dt if steer else 0.0
        moving = steer if self._tap > p.tap_dead_s else 0
        F = np.eye(3)
        F[0, 1] = F[0, 2] = v * dt
        F[1, 2] = v * dt / p.wheelbase_m
        self.x = F @ self.x
        self.x[2] += p.steer_rate * moving * dt
        s = min(abs(v), 1.0)            # standing still, nothing drifts but the actuator
        q = np.array([p.drift_offset * s, p.drift_heading * s, p.drift_steer * s]) ** 2 * dt
        q[2] += (p.tap_rate_error * p.steer_rate) ** 2 * 0.15 * dt * abs(steer)    # about right after a usual 0.15 s tap
        self.P = F @ self.P @ F.T + np.diag(q)
        self._t_vision += dt
        for z in self._thrown:          # carry the thrown-out pictures along with the ramp's own motion
            z[0] += v * (z[1] + self.x[2]) * dt
            z[1] += v * self.x[2] / p.wheelbase_m * dt

    def _update(self, z, H, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(3) - K @ H) @ self.P

    def gyro(self, yaw_rate):
        """Yaw rate in rad/s (+ = turning right). Driving, it shows the steer angle; standing, the gyro's own offset."""
        p = self.p
        if abs(self.v) < 0.02:
            if abs(yaw_rate - self.gyro_bias) < 0.05 or self._bias_n == 0:
                self._bias_n = min(self._bias_n + 1, 200)
                self.gyro_bias += (yaw_rate - self.gyro_bias) / self._bias_n
            return
        if abs(self.v) < p.min_speed:
            return
        H = np.array([[0.0, 0.0, self.v / p.wheelbase_m]])
        z = np.array([yaw_rate - self.gyro_bias])
        self.last["gyro"] = float(z[0] - (H @ self.x)[0])
        self._update(z, H, np.array([[p.gyro_sd ** 2]]))

    def vision(self, offset_m, heading_rad, edges=2):
        """One processed picture. Returns False if it was thrown out as not believable."""
        p = self.p
        if not edges or not np.isfinite(offset_m) or not np.isfinite(heading_rad) or abs(offset_m) > p.max_offset:
            return False
        k = 1.0 if edges >= 2 else p.one_edge_factor
        H = np.eye(3)[:2]
        R = np.diag([(p.vision_offset_sd * k) ** 2, (p.vision_heading_sd * k) ** 2])
        z = np.array([offset_m, heading_rad])
        d = z - H @ self.x
        S = H @ self.P @ H.T + R
        far = float(d @ np.linalg.inv(S) @ d) > p.gate ** 2
        self.last["vision"] = (float(d[0]), float(d[1]), far)
        if far:
            if self._thrown and (abs(z[0] - self._thrown[-1][0]) > 0.10 or abs(z[1] - self._thrown[-1][1]) > 4 * DEG):
                self._thrown = []       # it does not even agree with the last one thrown out
            self._thrown.append(z.copy())
            if len(self._thrown) < p.gate_agree:
                return False
            # vision has insisted, consistently: it is the estimate that is lost
            self.P[:2, :2] += np.diag([0.5 ** 2, (20 * DEG) ** 2])
        self._thrown = []
        self._t_vision = 0.0
        self._update(z, H, R)
        return True

    @property
    def blind_s(self):
        """How long since a picture was last used."""
        return self._t_vision


class Controller:
    """Decides the tap. Call step() every control tick."""

    def __init__(self, p):
        self.p = p
        self.out = 0
        self._since = 1e9               # how long the output has been what it is
        self._length = 0.0              # how long the current tap is meant to last

    def target(self, x, v):
        """The steer angle that brings the leading axle back to the centre line."""
        p = self.p
        L = p.wheelbase_m
        d = 1.0 if v >= 0 else -1.0
        offset, heading, _ = x
        lead = offset if d > 0 else offset - L * heading    # the camera is over the front axle
        along = d * heading                                 # heading, as seen in the direction of travel
        k_heading = 2 * p.damping / p.settle_m
        aim = np.clip(-lead / (2 * p.damping * p.settle_m), -p.max_heading, p.max_heading)
        return float(np.clip(-L * k_heading * (along - aim), -p.max_steer, p.max_steer))

    def want(self, est):
        """-1, 0 or +1: which way the steer angle is off by more than the deadband (no tap timing)."""
        if abs(est.v) < self.p.min_speed:
            return 0
        err = self.target(est.x, est.v) - est.x[2]
        return int(np.sign(err)) if abs(err) > self.p.deadband else 0

    def step(self, est, dt):
        p = self.p
        self._since += dt
        if abs(est.v) < p.min_speed:
            new = 0
        else:
            err = self.target(est.x, est.v) - est.x[2]
            if self.out == 0:
                new = int(np.sign(err)) if abs(err) > p.deadband and self._since >= p.min_pause_s else 0
                # the length is set at the start from the known actuator speed, not by watching the
                # estimate cross the target: the estimate trails the actuator, so that would overshoot
                self._length = float(np.clip(p.tap_fraction * abs(err) / p.steer_rate + p.tap_dead_s, p.min_tap_s, p.max_tap_s))
            else:
                new = self.out if self._since < self._length - 1e-9 else 0
        if new != self.out:
            self.out, self._since = new, 0.0
        return self.out
