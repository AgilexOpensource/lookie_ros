"""Arm workspace (design §9.2), positions in the torso heading frame F_t.

Each arm is clamped on its own into the same axis-aligned cube, fixed relative to the torso: centred
on the midpoint of the two end effectors at A (taken right after Home, so the home points) with a
half-size equal to their distance. Laterally each arm can reach half that distance past the other
arm's home point. Crossing the midline is allowed; the arms stay clear of the body through kstone
self-collision. Being clamped never disables teleop, it is only reported.
"""

from dataclasses import dataclass

import numpy as np

SIDES = ("left", "right")


@dataclass
class Workspace:
    center: np.ndarray   # midpoint of the two end effectors at A, in F_t
    half: float          # half-size of the cube: their distance at A

    @classmethod
    def from_home(cls, left, right) -> "Workspace":
        left, right = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
        return cls(0.5 * (left + right), float(np.linalg.norm(left - right)))


def project(targets: dict, ws: Workspace) -> tuple[dict, set]:
    """Clamp ``targets`` (side -> position in F_t) into the cube. Returns (clamped targets, clamped sides)."""
    out = {s: np.clip(np.asarray(p, dtype=float), ws.center - ws.half, ws.center + ws.half)
           for s, p in targets.items()}
    clamped = {s for s in out if np.linalg.norm(out[s] - np.asarray(targets[s], dtype=float)) > 1e-12}
    return out, clamped
