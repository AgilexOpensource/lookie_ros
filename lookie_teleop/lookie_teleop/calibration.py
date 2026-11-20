"""A calibration (design §7): geometry checks and reference recording.

Device/kstone health checks live in the node; this module only checks the pose geometry.
Positions are in F_h(0): x forward, y left, z up (m).
"""

from dataclasses import dataclass, field

import numpy as np

from .frames import Mount, invert, wrap_angle

SIDES = ("left", "right")


@dataclass
class CalibConfig:
    forward: tuple = (1.0, 0.0, 0.0)
    mount_max: float = np.radians(35.0)
    yaw_warn: float = np.radians(15.0)
    x_range: tuple = (0.15, 0.70)
    z_range: tuple = (-0.60, 0.30)
    min_cross: float = 0.10
    max_side: float = 0.20
    spacing_tol: float = 0.10     # warning only; a mismatch is closed by the catch-up


@dataclass
class Reference:
    mount: Mount
    p0: dict                      # side -> 4x4 controller pose in F_h(0)
    M0: np.ndarray                # controller midpoint in F_h(0)
    m0: np.ndarray                # end-effector midpoint in F_t(0)
    C: dict                       # side -> end-effector orientation at A (Home), in the body frame
    zh0: float                    # body height (world z of B_0)
    theta0: float                 # body lean
    yaw_h0: float                 # body yaw in XR world
    warnings: list = field(default_factory=list)


def check(headset: np.ndarray, controllers: dict, ee: dict, R: np.ndarray,
          cfg: CalibConfig) -> tuple[str | None, Reference | None]:
    """``controllers``: side -> 4x4 in XR world (TCP applied); ``ee``: side -> 4x4 in F_t(0).

    Positions are aligned at the midpoint: the vector between the hands maps 1:1, so the arms take
    the operator's hand spacing. A spacing that differs from the arms' only warns; the per-arm
    catch-up then moves each arm onto its mapped target. Orientation is absolute: ``C_i = R^T Q^ee_i(0)``
    (A is taken right after Home, so the Home orientation), so a controller TCP level and pointing along
    the body heading maps to the Home orientation and any other grip to that rotation of it; an arm whose
    mapped orientation differs at engage reaches it through the catch-up.

    Returns ``(reject_reason, None)`` or ``(None, Reference)``; only the first failure is reported.
    """
    mount = Mount(headset, cfg.forward)
    if abs(mount.pitch) > cfg.mount_max or abs(mount.roll) > cfg.mount_max:
        return (f"headset mount tilted too far (pitch {np.degrees(mount.pitch):.0f}°, "
                f"roll {np.degrees(mount.roll):.0f}°)"), None
    fh_inv = invert(mount.heading(headset))
    p0 = {s: fh_inv @ controllers[s] for s in SIDES}
    pl, pr = p0["left"][:3, 3], p0["right"][:3, 3]
    for side, p in (("left", pl), ("right", pr)):
        if not cfg.x_range[0] <= p[0] <= cfg.x_range[1]:
            return f"{side} controller forward distance {p[0]:.2f} m outside [{cfg.x_range[0]}, {cfg.x_range[1]}]", None
    if pl[1] - pr[1] < cfg.min_cross:
        return "controllers crossed or too close", None
    if abs(0.5 * (pl[1] + pr[1])) > cfg.max_side:
        return "controllers off to one side", None
    for side, p in (("left", pl), ("right", pr)):
        if not cfg.z_range[0] <= p[2] <= cfg.z_range[1]:
            return f"{side} controller height {p[2]:.2f} m outside [{cfg.z_range[0]}, {cfg.z_range[1]}]", None

    warnings = []
    el, er = ee["left"][:3, 3], ee["right"][:3, 3]
    gap_ctrl, gap_ee = np.linalg.norm(pl - pr), np.linalg.norm(el - er)
    if abs(gap_ctrl - gap_ee) >= cfg.spacing_tol:
        warnings.append(f"controller spacing {gap_ctrl:.2f} m differs from end-effector spacing {gap_ee:.2f} m; "
                        "the arms catch up smoothly to the controller spacing when engaged")
    C = {s: R.T @ ee[s][:3, :3] for s in SIDES}
    d = pl - pr
    if abs(wrap_angle(np.arctan2(-d[0], d[1]))) > cfg.yaw_warn:
        warnings.append("headset heading differs from the chest; chassis/waist yaw may keep a fixed offset")

    body = mount.body(headset)
    return None, Reference(
        mount=mount, p0=p0, M0=0.5 * (pl + pr), m0=0.5 * (el + er), C=C,
        zh0=float(body[2, 3]), theta0=mount.lean(headset), yaw_h0=mount.yaw(headset),
        warnings=warnings)
