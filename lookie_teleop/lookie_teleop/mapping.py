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
    k_z: float = 1.0
    k_pitch: float = 1.0
    z_range: tuple = (-0.30, 0.10)       # relative to A
    pitch_range: tuple = (-0.20, 0.60)   # relative to A
    yaw_range: tuple = (-0.80, 0.80)     # relative to A
    pitch_limits: tuple = (-math.pi / 4, math.pi / 2)  # absolute in base frame: back 45°, forward 90°
    z_rate: float = 0.20
    pitch_rate: float = 0.80
    yaw_rate: float = 1.00


class Waist:
    """Torso target maintained by teleop; x/y held at their A values, roll not mapped (§8.2).

    ``base`` is the headset-derived part, held while not driven. Z/pitch are absolute w.r.t. A, so
    re-engaging after the operator moved re-aligns them smoothly (rate limit); yaw is re-referenced.
    Stick offsets are added on top and only cleared by a new instance (created after Home).
    The torso has no roll DOF, so roll is never mapped.
    """

    def __init__(self, position, pitch: float, yaw: float, cfg: WaistConfig, axes: frozenset):
        self.cfg, self.axes = cfg, axes
        self.x, self.y = float(position[0]), float(position[1])
        self.z0, self.pitch0, self.yaw0 = float(position[2]), float(pitch), float(yaw)
        self.base = np.array([self.z0, self.pitch0, self.yaw0])
        self.target = self.base.copy()
        self.offset = np.zeros(2)            # dz, dpitch
        self.zh0 = self.theta0 = 0.0
        self.yaw_ref_robot = self.yaw0
        self.yaw_ref_body = 0.0

    def set_body_reference(self, zh0: float, theta0: float) -> None:
        self.zh0, self.theta0 = zh0, theta0

    def record_yaw_reference(self, yaw_body: float) -> None:
        self.yaw_ref_robot, self.yaw_ref_body = float(self.base[2]), yaw_body

    def _bounds(self, i: int):
        """Absolute [lo, hi] of z (i=0) or pitch (i=1): relative range around A, pitch also hard limits."""
        cfg = self.cfg
        if i == 0:
            return self.z0 + cfg.z_range[0], self.z0 + cfg.z_range[1]
        lo = max(self.pitch0 + cfg.pitch_range[0], cfg.pitch_limits[0])
        hi = min(self.pitch0 + cfg.pitch_range[1], cfg.pitch_limits[1])
        return lo, hi

    def add_offset(self, axis: str, delta: float) -> None:
        """Stick offset without wind-up: pushing past the limit of base + offset has no effect,
        so reversing the stick acts immediately."""
        i = 0 if axis == "z" else 1
        lo, hi = self._bounds(i)
        rel = self.cfg.z_range if i == 0 else self.cfg.pitch_range
        cur = self.offset[i]
        new = cur + delta
        if delta > 0:
            new = min(new, max(cur, hi - self.base[i]))
        elif delta < 0:
            new = max(new, min(cur, lo - self.base[i]))
        self.offset[i] = float(np.clip(new, *rel))

    def update(self, dt: float, body: tuple | None) -> None:
        """``body`` = (zh, theta, yaw_h) while dual-active in waist mode, else None (hold)."""
        cfg = self.cfg
        if body is not None:
            zh, theta, yaw_h = body
            if "waist_z" in self.axes:
                self.base[0] = self.z0 + cfg.k_z * (zh - self.zh0)
            if "waist_pitch" in self.axes:
                self.base[1] = self.pitch0 + cfg.k_pitch * (theta - self.theta0)
            if "waist_yaw" in self.axes:
                self.base[2] = self.yaw_ref_robot + wrap_angle(yaw_h - self.yaw_ref_body)
        for i in (0, 1):
            self.base[i] = np.clip(self.base[i], *self._bounds(i))
        self.base[2] = self.yaw0 + np.clip(wrap_angle(self.base[2] - self.yaw0), *cfg.yaw_range)
        # one final clamp for headset + stick, so the torso target sent to kstone is always in range
        desired = self.base + np.array([self.offset[0], self.offset[1], 0.0])
        for i in (0, 1):
            desired[i] = np.clip(desired[i], *self._bounds(i))
        step = np.array([cfg.z_rate, cfg.pitch_rate, cfg.yaw_rate]) * dt
        self.target = rate_limit(self.target, desired, step)

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.target[0]])

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
    v_max: float = 0.5
    w_max: float = 1.0
    max_dev_pos: float = 0.2              # target stays within this of the feedback (m)
    max_dev_ang: float = math.radians(15.0)


def _soft_deadband(value, band):
    n = float(np.linalg.norm(value))
    if n <= band:
        return np.zeros_like(value)
    return value * ((n - band) / n)


class Chassis:
    """``C_t = C_ref (+) planar(F_h(ref)^-1 F_h(t))`` in the world frame (§8.3)."""

    def __init__(self, cfg: ChassisConfig):
        self.cfg = cfg
        self.ref = None
        self.fh_ref = None
        self.target = None

    def record(self, chassis_xyyaw, fh: np.ndarray) -> None:
        self.ref = np.asarray(chassis_xyyaw, dtype=float).copy()
        self.fh_ref = fh.copy()
        self.target = self.ref.copy()

    def update(self, fh: np.ndarray, dt: float, feedback=None) -> np.ndarray:
        """``feedback``: measured chassis (x, y, yaw). When given, the target is clamped to
        within ``max_dev_*`` of it, so a lagging base is chased at the limit instead of
        accumulating a large error (no disable)."""
        cfg = self.cfg
        d = planar(invert(self.fh_ref) @ fh)
        trans = _soft_deadband(d[:2] * cfg.scale, cfg.deadband_pos)
        dyaw = float(_soft_deadband(np.array([d[2]]), cfg.deadband_ang)[0])
        c, s = math.cos(self.ref[2]), math.sin(self.ref[2])
        desired = np.array([
            self.ref[0] + c * trans[0] - s * trans[1],
            self.ref[1] + s * trans[0] + c * trans[1],
            self.ref[2] + dyaw,
        ])
        step = desired[:2] - self.target[:2]
        n = np.linalg.norm(step)
        if n > cfg.v_max * dt:
            step *= cfg.v_max * dt / n
        dy = float(np.clip(wrap_angle(desired[2] - self.target[2]), -cfg.w_max * dt, cfg.w_max * dt))
        self.target = np.array([self.target[0] + step[0], self.target[1] + step[1],
                                wrap_angle(self.target[2] + dy)])
        if feedback is not None:
            fb = np.asarray(feedback, dtype=float)
            off = self.target[:2] - fb[:2]
            n = np.linalg.norm(off)
            if n > cfg.max_dev_pos:
                self.target[:2] = fb[:2] + off * (cfg.max_dev_pos / n)
            dyaw = float(np.clip(wrap_angle(self.target[2] - fb[2]), -cfg.max_dev_ang, cfg.max_dev_ang))
            self.target[2] = wrap_angle(fb[2] + dyaw)
        return self.target


def ee_target(trigger: float, deadzone: float, q_open, q_close, lower, upper) -> np.ndarray:
    """``q = q_open + u (q_close - q_open)``, clamped to the discover limits (§10)."""
    u = float(np.clip((trigger - deadzone) / (1.0 - deadzone), 0.0, 1.0))
    q = np.asarray(q_open, dtype=float) + u * (np.asarray(q_close, dtype=float) - np.asarray(q_open, dtype=float))
    return np.clip(q, lower, upper)
