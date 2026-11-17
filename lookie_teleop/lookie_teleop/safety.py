"""Disable conditions (design §12). Each check returns a reason string, or None when healthy."""

import numpy as np

from .frames import rotation_angle


def kstone_health(state: dict | None, age: float, timeout_s: float, require_fresh: bool,
                  modules: list, check_held: bool = True, ignore_groups=()) -> str | None:
    """``ignore_groups``: kstone feedback groups that may go stale without disabling (end effectors)."""
    if state is None or age > timeout_s:
        return "kstone state timed out"
    if state.get("fault"):
        return f"kstone fault: {state['fault']}"
    if require_fresh:
        groups = state.get("groups")
        if groups:
            stale = [g.get("name", "?") for g in groups if not g.get("fresh", False) and g.get("name") not in ignore_groups]
            if stale:
                return f"kstone state not fresh (fresh=false: {', '.join(stale)})"
        elif not state.get("fresh", False):
            return "kstone state not fresh (fresh=false)"
    mods = state.get("modules", {})
    for mid in modules:
        m = mods.get(mid)
        if m is None or not m.get("enabled", False):
            return f"kstone module {mid} not enabled"
        if check_held and m.get("held_by"):
            return f"kstone module {mid} held by {m['held_by']}"
    return None


def stale_groups(state: dict | None, require_fresh: bool) -> list:
    """Names of the kstone feedback groups not fresh or not yet attached again (empty if freshness is not
    required). A group counts as recovered only once kstone has reattached it and rebased its modules."""
    if not require_fresh or not state:
        return []
    return [g.get("name", "?") for g in state.get("groups") or ()
            if not g.get("fresh", False) or not g.get("attached", True)]


class XrMonitor:
    """XR link and tracking checks on the three transforms (headset, left, right).

    Results are per name so a single lost controller can stop only its arm (design §12).
    """

    def __init__(self, timeout_s: float, jump_pos: float, jump_ang: float, max_reach: float,
                 jump_speed: float = 0.0, jump_rate: float = 0.0):
        """Controllers may also move ``jump_speed`` (m/s) / ``jump_rate`` (rad/s) times the stamp interval
        between two checks (at most 0.1 s), so a fast but continuous hand motion is not a jump; the headset
        stays strict."""
        self.timeout_s = timeout_s
        self.jump_pos, self.jump_ang = jump_pos, jump_ang
        self.jump_speed, self.jump_rate = jump_speed, jump_rate
        self.max_reach = max_reach
        self._last = {}

    def reset(self) -> None:
        self._last.clear()

    def forget(self, name: str) -> None:
        """Drop the jump reference of one name (after its tracking recovered)."""
        self._last.pop(name, None)

    def stale_issues(self, stamps: dict, now: float) -> dict:
        """``stamps``: name -> receive time (node clock) or None. Returns name -> reason."""
        return {name: f"XR {name} data timed out" for name, t in stamps.items()
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
                    out[name] = f"XR {name} timestamp went backwards"
                elif stamps[name] > last_stamp:
                    # capped so a pose resuming after a data gap is still checked for a jump
                    dt = min(stamps[name] - last_stamp, 0.1) if name in ("left", "right") else 0.0
                    if np.linalg.norm(pose[:3, 3] - last_pose[:3, 3]) > self.jump_pos + self.jump_speed * dt:
                        out[name] = f"XR {name} position jump"
                    elif rotation_angle(last_pose[:3, :3].T @ pose[:3, :3]) > self.jump_ang + self.jump_rate * dt:
                        out[name] = f"XR {name} orientation jump"
            self._last[name] = (pose, stamps[name])
        head = poses.get("headset")
        if head is not None:
            for name in ("left", "right"):
                if (name in poses and name not in out
                        and np.linalg.norm(poses[name][:3, 3] - head[:3, 3]) > self.max_reach):
                    out[name] = f"{name} controller too far from headset"
        return out

    def check(self, poses: dict, stamps: dict) -> str | None:
        issues = self.issues(poses, stamps)
        for name in ("headset", "left", "right"):
            if name in issues:
                return issues[name]
        return next(iter(issues.values()), None)
