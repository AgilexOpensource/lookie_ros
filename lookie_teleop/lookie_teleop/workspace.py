"""Workspace projection Π (design §9.2), positions in the torso heading frame F_t.

Only positions are constrained; constraints are convex and projected in sequence a few times.
Being clamped never disables teleop, it is only reported.
"""

from dataclasses import dataclass

import numpy as np

SIDES = ("left", "right")


@dataclass
class WorkspaceConfig:
    shoulder_offsets: dict          # side -> shoulder in the chest frame (before waist pitch)
    r_min: float = 0.20
    r_max: float = 0.75
    x_min: float = 0.10
    z_range: tuple = (-0.6, 0.3)
    midline_delta: float = 0.05
    min_hand_gap: float = 0.10
    iterations: int = 3


def _shell(e, s, r_min, r_max):
    d = e - s
    n = np.linalg.norm(d)
    if n < 1e-9:
        return s + np.array([r_min, 0.0, 0.0])
    if n > r_max:
        return s + d * (r_max / n)
    if n < r_min:
        return s + d * (r_min / n)
    return e


def project(targets: dict, cfg: WorkspaceConfig, waist_pitch_rot: np.ndarray, fixed: dict | None = None):
    """Project ``targets`` (side -> position in F_t) into the workspace.

    ``waist_pitch_rot`` rotates the chest-frame shoulder offsets into F_t. ``fixed`` holds the
    measured end effector of a stopped arm: it only bounds the moving arm for the no-crossing
    constraint and is never moved itself. Returns (projected, clamped side set).
    """
    fixed = fixed or {}
    shoulders = {s: waist_pitch_rot @ np.asarray(cfg.shoulder_offsets[s], dtype=float) for s in SIDES}
    out = {s: np.asarray(p, dtype=float).copy() for s, p in targets.items()}
    for _ in range(cfg.iterations):
        for side, e in out.items():
            e = _shell(e, shoulders[side], cfg.r_min, cfg.r_max)
            e[0] = max(e[0], cfg.x_min)
            e[2] = float(np.clip(e[2], cfg.z_range[0], cfg.z_range[1]))
            if side == "left":
                e[1] = max(e[1], -cfg.midline_delta)
            else:
                e[1] = min(e[1], cfg.midline_delta)
            out[side] = e
        if "left" in out and "right" in out:
            gap = out["left"][1] - out["right"][1]
            if gap < cfg.min_hand_gap:
                half = 0.5 * (cfg.min_hand_gap - gap)
                out["left"][1] += half
                out["right"][1] -= half
        elif "left" in out and "right" in fixed:
            out["left"][1] = max(out["left"][1], fixed["right"][1] + cfg.min_hand_gap)
        elif "right" in out and "left" in fixed:
            out["right"][1] = min(out["right"][1], fixed["left"][1] - cfg.min_hand_gap)
    clamped = {s for s in out if np.linalg.norm(out[s] - np.asarray(targets[s], dtype=float)) > 1e-6}
    return out, clamped
