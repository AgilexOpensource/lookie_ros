"""Frames of design §6: headset mount correction, body heading frame F_h and torso heading frame F_t.

All poses are 4x4 homogeneous matrices; quaternions are (w, x, y, z).
"""

import numpy as np

from .transforms import make_transform, matrix_to_quat, rot_z, yaw_of


def rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def pitch_of(r: np.ndarray) -> float:
    """ZYX pitch; positive tilts +X down (leaning forward)."""
    r = np.asarray(r, dtype=float)
    return float(np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0])))


def roll_of(r: np.ndarray) -> float:
    r = np.asarray(r, dtype=float)
    return float(np.arctan2(r[2, 1], r[2, 2]))


def rotation_angle(r: np.ndarray) -> float:
    return float(np.arccos(np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)))


def wrap_angle(a: float) -> float:
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


def invert(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def heading_frame(position, yaw: float) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rot_z(yaw)
    t[:3, 3] = position
    return t


def tcp_transform(offset, side: str = "right") -> np.ndarray:
    """Controller TCP offset [x, y, z, roll, pitch, yaw] in the controller frame (+X pointing, +Y left, +Z up),
    given for the right controller, where +y is inward (towards the palm); the left controller uses its mirror
    image about the controller XZ plane (y, roll and yaw negated), so +y is inward on both sides."""
    o = np.zeros(6) if offset is None else np.array(offset, dtype=float)
    if side == "left":
        o[[1, 3, 5]] *= -1.0
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(o[3], o[4], o[5])
    t[:3, 3] = o[:3]
    return t


class Mount:
    """``H0 = Rz(psi0) R_mount`` solved once at A; ``B_t = H_t R_mount^-1``."""

    def __init__(self, headset0: np.ndarray, forward=(1.0, 0.0, 0.0)):
        self.forward = np.asarray(forward, dtype=float)
        yaw0 = yaw_of(headset0[:3, :3], self.forward)
        self.rotation = rot_z(yaw0).T @ headset0[:3, :3]
        # R_mount = Ry(theta_m) Rx(phi_m)
        self.pitch = pitch_of(self.rotation)
        self.roll = roll_of(self.rotation)

    def body(self, headset: np.ndarray) -> np.ndarray:
        b = np.eye(4)
        b[:3, :3] = headset[:3, :3] @ self.rotation.T
        b[:3, 3] = headset[:3, 3]
        return b

    def heading(self, headset: np.ndarray) -> np.ndarray:
        """F_h(t): position of B_t, yaw-only orientation (gravity aligned)."""
        b = self.body(headset)
        return heading_frame(b[:3, 3], yaw_of(b[:3, :3]))

    def lean(self, headset: np.ndarray) -> float:
        return pitch_of(self.body(headset)[:3, :3])

    def yaw(self, headset: np.ndarray) -> float:
        return yaw_of(self.body(headset)[:3, :3])


def planar(t: np.ndarray) -> np.ndarray:
    """(x, y, yaw) of a transform."""
    return np.array([t[0, 3], t[1, 3], yaw_of(t[:3, :3])])


def pose_to_matrix(position, quat_wxyz) -> np.ndarray:
    return make_transform(np.asarray(position, dtype=float), np.asarray(quat_wxyz, dtype=float))


def matrix_to_pose(t: np.ndarray):
    return t[:3, 3].copy(), matrix_to_quat(t[:3, :3])
