"""Rate-limited pose stepping and the per-arm catch-up (design §9.3)."""

import numpy as np

from .transforms import slerp


def quat_angle(q0: np.ndarray, q1: np.ndarray) -> float:
    dot = abs(float(np.dot(q0, q1)) / (np.linalg.norm(q0) * np.linalg.norm(q1)))
    return float(2.0 * np.arccos(min(1.0, dot)))


def limit_step(pos, goal_pos, step: float):
    """Move ``pos`` toward ``goal_pos`` by at most ``step`` metres."""
    pos, goal_pos = np.asarray(pos, dtype=float), np.asarray(goal_pos, dtype=float)
    delta = goal_pos - pos
    dist = np.linalg.norm(delta)
    return goal_pos.copy() if dist <= step else pos + delta * (step / dist)


def step_toward(pos, quat, goal_pos, goal_quat, v: float, w: float, dt: float):
    """Advance (pos, quat) toward the goal by at most ``v * dt`` metres and ``w * dt`` radians."""
    new_pos = limit_step(pos, goal_pos, v * dt)
    angle = quat_angle(quat, goal_quat)
    max_angle = w * dt
    new_quat = np.asarray(goal_quat, dtype=float) if angle <= max_angle else slerp(quat, goal_quat, max_angle / angle)
    if new_quat[0] < 0.0:
        new_quat = -new_quat
    return new_pos, new_quat


class CatchUp:
    """Moves the published target from the measured end effector to the projected mapped target."""

    def __init__(self, v: float, w: float, pos_tol: float, ori_tol: float, timeout_s: float,
                 fast_v: float, fast_w: float):
        self.v, self.w = v, w
        self.pos_tol, self.ori_tol = pos_tol, ori_tol
        self.timeout_s = timeout_s
        self.fast_v, self.fast_w = fast_v, fast_w
        self.started = None

    def start(self, now: float) -> None:
        self.started = now

    def step(self, pos, quat, goal_pos, goal_quat, dt: float, now: float,
             ctrl_speed: tuple[float, float]):
        """Returns (pos, quat, status) with status ``"catching"``, ``"converged"`` or a failure reason."""
        if ctrl_speed[0] > self.fast_v or ctrl_speed[1] > self.fast_w:
            return pos, quat, "controller moving too fast"
        if now - self.started > self.timeout_s:
            return pos, quat, "catch-up timed out"
        pos, quat = step_toward(pos, quat, goal_pos, goal_quat, self.v, self.w, dt)
        converged = (np.linalg.norm(np.asarray(goal_pos) - pos) < self.pos_tol
                     and quat_angle(quat, goal_quat) < self.ori_tol)
        return pos, quat, "converged" if converged else "catching"
