"""Arm / waist / chassis / end-effector mapping (design §8, §10)."""

import math
from dataclasses import dataclass

import numpy as np

from .frames import heading_frame, invert, planar, rot_y, wrap_angle
from .transforms import rot_z


def apply_deadzone(value: float, deadzone: float) -> float:
    """Copied from lookie_teleop.policy: rescales the live range to keep the output continuous."""
    deadzone = min(max(deadzone, 0.0), 0.95)
    value = min(max(float(value), -1.0), 1.0)
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def rate_limit(current, target, max_step):
    """Moves ``current`` toward ``target`` by at most ``max_step`` per element."""
    current = np.asarray(current, dtype=float)
    return current + np.clip(np.asarray(target, dtype=float) - current, -max_step, max_step)


def to_base(ft: np.ndarray, pos, rot) -> np.ndarray:
    """Pose in F_t -> 4x4 in ``chassis_base_link``."""
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = pos
    return ft @ t


class ArmMap:
    """``e_i = m0 + R (p_i - M0)``, ``Q_i = R Q^p_i C_i`` with ``p_i = F_h(t)^-1 h_i`` (§8.1)."""

    def __init__(self, M0, m0, R: np.ndarray, C: dict):
        self.M0 = np.asarray(M0, dtype=float)
        self.m0 = np.asarray(m0, dtype=float)
        self.R = R
        self.C = C

    def target(self, side: str, fh: np.ndarray, controller: np.ndarray):
        p = invert(fh) @ controller
        return self.m0 + self.R @ (p[:3, 3] - self.M0), self.R @ p[:3, :3] @ self.C[side]


@dataclass
class WaistConfig:
    k_x: float = 1.0
    k_z: float = 1.0
    k_pitch: float = 1.0
    z_range: tuple = (-0.30, 0.10)       # relative to A
    pitch_limits: tuple = (-math.pi / 6, math.pi / 2)  # absolute in base frame: back 30°, forward 90°
    yaw_limits: tuple = (-math.radians(89.0), math.radians(89.0))  # absolute in base frame (torso_joint4 ±90°)
    x_rate: float = 0.20
    z_rate: float = 0.20
    pitch_rate: float = 0.80
    yaw_rate: float = 1.00


class Waist:
    """Torso target maintained by teleop; y held at its A value, roll not mapped (§8.2).

    ``base`` is the headset-derived part, held while not driven. Z/pitch are absolute w.r.t. A, so
    re-engaging after the operator moved re-aligns them smoothly (rate limit); x (forward) and yaw are
    re-referenced, so walking while not dual-active does not move the torso. The stick axes are
    limited: z by z_range around A, pitch by the absolute pitch_limits; yaw only by the absolute
    yaw_limits (joint range; the operator's turn is unwrapped, so passing 180° never flips the target
    to the other side); x is not limited. Stick offsets
    are added on top and only cleared by a new instance (created after Home).
    The torso has no roll DOF, so roll is never mapped.
    """

    def __init__(self, position, pitch: float, yaw: float, cfg: WaistConfig, axes: frozenset):
        self.cfg, self.axes = cfg, axes
        self.x0, self.y = float(position[0]), float(position[1])
        self.base_x = self.target_x = self.x_ref_robot = self.x0
        self.z0, self.pitch0, self.yaw0 = float(position[2]), float(pitch), float(yaw)
        self.base = np.array([self.z0, self.pitch0, self.yaw0])
        self.target = self.base.copy()
        self.offset = np.zeros(2)            # dz, dpitch
        self.zh0 = self.theta0 = 0.0
        self.yaw_ref_robot = self.yaw0
        self.yaw_body_last = 0.0
        self.yaw_turn = 0.0                  # operator's turn since the reference, unwrapped

    def set_body_reference(self, zh0: float, theta0: float) -> None:
        self.zh0, self.theta0 = zh0, theta0

    def record_yaw_reference(self, yaw_body: float) -> None:
        self.yaw_ref_robot, self.yaw_body_last, self.yaw_turn = float(self.base[2]), yaw_body, 0.0

    def record_x_reference(self) -> None:
        """The body forward displacement passed to update() is measured from this moment."""
        self.x_ref_robot = self.base_x

    def _bounds(self, i: int):
        """Absolute [lo, hi] of z (i=0, range around A) or pitch (i=1, hard limits in the base frame)."""
        cfg = self.cfg
        if i == 0:
            return self.z0 + cfg.z_range[0], self.z0 + cfg.z_range[1]
        return cfg.pitch_limits

    def add_offset(self, axis: str, delta: float) -> None:
        """Stick offset without wind-up: pushing past the limit of base + offset has no effect,
        so reversing the stick acts immediately."""
        i = 0 if axis == "z" else 1
        lo, hi = self._bounds(i)
        span = hi - lo
        cur = self.offset[i]
        new = cur + delta
        if delta > 0:
            new = min(new, max(cur, hi - self.base[i]))
        elif delta < 0:
            new = max(new, min(cur, lo - self.base[i]))
        self.offset[i] = float(np.clip(new, -span, span))

    def update(self, dt: float, body: tuple | None, axes: frozenset | None = None) -> None:
        """``body`` = (zh, theta, yaw_h[, forward]) while dual-active, else None (hold).

        ``forward``: body displacement (m) along the heading recorded with record_x_reference().
        ``axes``: the axes the headset drives now (default all enabled ones); the others hold."""
        cfg = self.cfg
        axes = self.axes if axes is None else self.axes & axes
        if body is not None:
            zh, theta, yaw_h, *rest = body
            if "waist_x" in axes and rest:
                self.base_x = self.x_ref_robot + cfg.k_x * rest[0]
            if "waist_z" in axes:
                self.base[0] = self.z0 + cfg.k_z * (zh - self.zh0)
            if "waist_pitch" in axes:
                self.base[1] = self.pitch0 + cfg.k_pitch * (theta - self.theta0)
            if "waist_yaw" in axes:
                self.yaw_turn += wrap_angle(yaw_h - self.yaw_body_last)
                self.yaw_body_last = yaw_h
                self.base[2] = np.clip(self.yaw_ref_robot + self.yaw_turn, *cfg.yaw_limits)
            # The range bounds headset + stick together (around the robot home at A), so the headset can
            # reach the same limit as the stick whatever the stick offset is.
            for i in (0, 1):
                lo, hi = self._bounds(i)
                self.base[i] = np.clip(self.base[i], lo - self.offset[i], hi - self.offset[i])
        self.target_x = float(rate_limit(self.target_x, self.base_x, cfg.x_rate * dt))
        # one final clamp for headset + stick, so the torso target sent to kstone is always in range
        desired = self.base + np.array([self.offset[0], self.offset[1], 0.0])
        for i in (0, 1):
            desired[i] = np.clip(desired[i], *self._bounds(i))
        step = np.array([cfg.z_rate, cfg.pitch_rate, cfg.yaw_rate]) * dt
        self.target = rate_limit(self.target, desired, step)

    @property
    def position(self) -> np.ndarray:
        return np.array([self.target_x, self.y, self.target[0]])

    @property
    def rotation(self) -> np.ndarray:
        return rot_z(self.target[2]) @ rot_y(self.target[1])

    @property
    def pitch_rotation(self) -> np.ndarray:
        return rot_y(self.target[1])

    def frame(self) -> np.ndarray:
        """F_t: torso target position, yaw only."""
        return heading_frame(self.position, self.target[2])


@dataclass
class ChassisConfig:
    scale: float = 1.0
    deadband_pos: float = 0.02
    deadband_ang: float = math.radians(2.0)
    vx_max: float = 0.5                   # body-x (forward) target speed (m/s)
    vy_max: float = 0.5                   # body-y (lateral) target speed (m/s); ellipse with vx_max
    w_max: float = 1.0
    diverge_pos: float = 0.2              # base farther than this from the desired pose (m) ...
    diverge_ang: float = math.radians(15.0)
    diverge_v: float = 0.05               # ... and moving away from it faster than this (m/s) ...
    diverge_w: float = 0.1                # ... or turning away faster than this (rad/s) ...
    diverge_s: float = 1.5                # ... for this long -> fault (a lagging base under a swaying
                                          # operator moves away for up to ~0.7 s)


# The pose target carries a wrapped yaw and the base turns the short way to it, so the target may never
# lead the measured yaw by 180° or more; this only engages when the base is far behind a spinning operator.
YAW_LEAD_MAX = math.radians(150.0)


def _soft_deadband(value, band):
    """|out| = |in| - band beyond 2*band, |in|^2 / (4*band) inside: output and its slope are continuous,
    so the target does not jump from standstill to full speed when the operator crosses the band."""
    n = float(np.linalg.norm(value))
    if n <= 0.0 or band <= 0.0:
        return np.asarray(value, dtype=float) * (1.0 if band <= 0.0 else 0.0)
    out = n - band if n >= 2.0 * band else n * n / (4.0 * band)
    return value * (out / n)


class Chassis:
    """``C_t = C_ref (+) planar(F_h(ref)^-1 F_h(t))`` in the world frame (§8.3)."""

    def __init__(self, cfg: ChassisConfig):
        self.cfg = cfg
        self.ref = None
        self.fh_ref = None
        self.target = None
        self.fault = None
        self._fb = None
        self._vel = np.zeros(3)
        self._diverging = 0.0
        self._yaw = 0.0           # yaw relative to ref, unwrapped: desired / target / feedback
        self._tyaw = 0.0
        self._fb_yaw = 0.0

    def record(self, chassis_xyyaw, fh: np.ndarray) -> None:
        self.ref = np.asarray(chassis_xyyaw, dtype=float).copy()
        self.fh_ref = fh.copy()
        self.target = self.ref.copy()
        self.fault = None
        self._fb = None
        self._vel = np.zeros(3)
        self._diverging = 0.0
        self._yaw = 0.0
        self._tyaw = 0.0
        self._fb_yaw = 0.0

    def update(self, fh: np.ndarray, dt: float, feedback=None) -> np.ndarray:
        """``feedback``: measured chassis (x, y, yaw). A base that keeps moving away from the desired
        pose beyond ``diverge_*`` (e.g. a reversed axis) sets ``fault``."""
        cfg = self.cfg
        d = planar(invert(self.fh_ref) @ fh)
        trans = _soft_deadband(d[:2] * cfg.scale, cfg.deadband_pos)
        # Unwrapped: turning past 180° from the reference must not flip the desired turn direction.
        self._yaw += wrap_angle(d[2] - self._yaw)
        dyaw = float(_soft_deadband(np.array([self._yaw]), cfg.deadband_ang)[0])
        c, s = math.cos(self.ref[2]), math.sin(self.ref[2])
        desired = np.array([
            self.ref[0] + c * trans[0] - s * trans[1],
            self.ref[1] + s * trans[0] + c * trans[1],
            self.ref[2] + dyaw,
        ])
        step = desired[:2] - self.target[:2]
        # Speed is bounded per body axis of the current target heading: (vx/vx_max)^2 + (vy/vy_max)^2 <= 1.
        c, s = math.cos(self.ref[2] + self._tyaw), math.sin(self.ref[2] + self._tyaw)
        bx, by = c * step[0] + s * step[1], -s * step[0] + c * step[1]
        n = math.hypot(bx / (cfg.vx_max * dt), by / (cfg.vy_max * dt))
        if n > 1.0:
            step /= n
        # Yaw is chased in the unwrapped space too: a wrapped difference would turn the
        # target the short way round once the operator is more than 180° ahead of it.
        self._tyaw += float(np.clip(dyaw - self._tyaw, -cfg.w_max * dt, cfg.w_max * dt))
        self.target = np.array([self.target[0] + step[0], self.target[1] + step[1], 0.0])
        if feedback is not None:
            fb = np.asarray(feedback, dtype=float)
            self._fb_yaw += wrap_angle(fb[2] - self.ref[2] - self._fb_yaw)
            self._check_divergence(desired, dyaw, fb, dt)
            self._tyaw = self._fb_yaw + float(np.clip(self._tyaw - self._fb_yaw, -YAW_LEAD_MAX, YAW_LEAD_MAX))
        self.target[2] = wrap_angle(self.ref[2] + self._tyaw)
        return self.target

    def _check_divergence(self, desired, dyaw, fb, dt):
        cfg = self.cfg
        if self._fb is not None and dt > 0.0:
            d = fb - self._fb
            d[2] = wrap_angle(d[2])
            a = dt / (0.15 + dt)                  # ~0.15 s low-pass on the feedback velocity
            self._vel += a * (d / dt - self._vel)
        self._fb = fb.copy()
        err = desired[:2] - fb[:2]
        n = float(np.linalg.norm(err))
        eyaw = dyaw - self._fb_yaw
        away = ((n > cfg.diverge_pos and float(self._vel[:2] @ err) / n < -cfg.diverge_v)
                or (abs(eyaw) > cfg.diverge_ang and self._vel[2] * math.copysign(1.0, eyaw) < -cfg.diverge_w))
        self._diverging = self._diverging + dt if away else 0.0
        if self._diverging > cfg.diverge_s and self.fault is None:
            self.fault = f"底盘背离目标运动（误差 {n:.2f} m / {math.degrees(eyaw):.0f}°）"


class Settle:
    """True once the measured planar pose has held within ``pos_tol``/``ang_tol`` for ``hold_s``, or
    ``timeout_s`` after reset(). Used after a stick release: the base keeps coasting (response delay +
    deceleration), so a headset reference recorded at the release would pull it back."""

    def __init__(self, pos_tol=0.005, ang_tol=math.radians(0.5), hold_s=0.3, timeout_s=2.0):
        self.pos_tol, self.ang_tol, self.hold_s, self.timeout_s = pos_tol, ang_tol, hold_s, timeout_s
        self.start = self.anchor = self.since = None

    def reset(self, now: float) -> None:
        self.start, self.anchor, self.since = now, None, None

    def update(self, pose, now: float) -> bool:
        if self.start is None:
            self.reset(now)
        if pose is not None:
            p = np.asarray(pose, dtype=float)
            if (self.anchor is None or np.linalg.norm(p[:2] - self.anchor[:2]) > self.pos_tol
                    or abs(wrap_angle(p[2] - self.anchor[2])) > self.ang_tol):
                self.anchor, self.since = p.copy(), now
            if now - self.since >= self.hold_s:
                return True
        return now - self.start >= self.timeout_s


def ee_target(trigger: float, deadzone: float, q_open, q_close, lower, upper) -> np.ndarray:
    """``q = q_open + u (q_close - q_open)``, clamped to the discover limits (§10)."""
    u = float(np.clip((trigger - deadzone) / (1.0 - deadzone), 0.0, 1.0))
    q = np.asarray(q_open, dtype=float) + u * (np.asarray(q_close, dtype=float) - np.asarray(q_open, dtype=float))
    return np.clip(q, lower, upper)
