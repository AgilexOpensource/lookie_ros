"""Disable conditions (design §12). Each check returns a reason string, or None when healthy."""

import numpy as np

from .frames import rotation_angle


def kstone_health(state: dict | None, age: float, timeout_s: float, require_fresh: bool,
                  modules: list, check_held: bool = True) -> str | None:
    if state is None or age > timeout_s:
        return "运控状态超时"
    if state.get("fault"):
        return f"运控故障: {state['fault']}"
    if require_fresh and not state.get("fresh", False):
        return "运控状态不新鲜 (fresh=false)"
    mods = state.get("modules", {})
    for mid in modules:
        m = mods.get(mid)
        if m is None or not m.get("enabled", False):
            return f"运控模块 {mid} 未启用"
        if check_held and m.get("held_by"):
            return f"运控模块 {mid} 被 {m['held_by']} 抢占"
    return None


class XrMonitor:
    """XR link and tracking checks on the three transforms (headset, left, right).

    Results are per name so a single lost controller can stop only its arm (design §12).
    """

    def __init__(self, timeout_s: float, jump_pos: float, jump_ang: float, max_reach: float):
        self.timeout_s = timeout_s
        self.jump_pos, self.jump_ang = jump_pos, jump_ang
        self.max_reach = max_reach
        self._last = {}

    def reset(self) -> None:
        self._last.clear()

    def forget(self, name: str) -> None:
        """Drop the jump reference of one name (after its tracking recovered)."""
        self._last.pop(name, None)

    def stale_issues(self, stamps: dict, now: float) -> dict:
        """``stamps``: name -> receive time (node clock) or None. Returns name -> reason."""
        return {name: f"XR {name} 数据超时" for name, t in stamps.items()
                if t is None or now - t > self.timeout_s}

    def stale(self, stamps: dict, now: float) -> str | None:
        issues = self.stale_issues(stamps, now)
        return next(iter(issues.values()), None)

    def issues(self, poses: dict, stamps: dict) -> dict:
        """``poses``: name -> 4x4 in the XR world; ``stamps``: name -> XR header stamp (s).

        Every pose is stored even when it jumped, so a step change is reported once.
        """
        out = {}
        for name, pose in poses.items():
            last = self._last.get(name)
            if last is not None:
                last_pose, last_stamp = last
                if stamps[name] < last_stamp:
                    out[name] = f"XR {name} 时间戳回退"
                elif stamps[name] > last_stamp:
                    if np.linalg.norm(pose[:3, 3] - last_pose[:3, 3]) > self.jump_pos:
                        out[name] = f"XR {name} 位置跳变"
                    elif rotation_angle(last_pose[:3, :3].T @ pose[:3, :3]) > self.jump_ang:
                        out[name] = f"XR {name} 姿态跳变"
            self._last[name] = (pose, stamps[name])
        head = poses.get("headset")
        if head is not None:
            for name in ("left", "right"):
                if (name in poses and name not in out
                        and np.linalg.norm(poses[name][:3, 3] - head[:3, 3]) > self.max_reach):
                    out[name] = f"{name} 手柄离头盔过远"
        return out

    def check(self, poses: dict, stamps: dict) -> str | None:
        issues = self.issues(poses, stamps)
        for name in ("headset", "left", "right"):
            if name in issues:
                return issues[name]
        return next(iter(issues.values()), None)


class ErrorRateMonitor:
    """Flags tracking-error jumps: low-passed d(error)/dt above a limit for n consecutive samples.

    A sustained (constant) error never trips it.
    """

    def __init__(self, rate_pos: float, rate_ang: float, n: int, lpf_hz: float):
        self.rate_pos, self.rate_ang = rate_pos, rate_ang
        self.n = n
        self.lpf_hz = lpf_hz
        self.reset()

    def reset(self) -> None:
        self._prev = None
        self._rate = np.zeros(2)
        self._count = 0

    def update(self, stamp: float, pos_err: float, ang_err: float) -> bool:
        err = np.array([pos_err, ang_err], dtype=float)
        if self._prev is None:
            self._prev = (stamp, err)
            return False
        dt = stamp - self._prev[0]
        if dt <= 0.0:
            return False
        raw = (err - self._prev[1]) / dt
        self._prev = (stamp, err)
        alpha = 1.0 - np.exp(-2.0 * np.pi * self.lpf_hz * dt)
        self._rate += alpha * (raw - self._rate)
        over = abs(self._rate[0]) > self.rate_pos or abs(self._rate[1]) > self.rate_ang
        self._count = self._count + 1 if over else 0
        return self._count >= self.n
