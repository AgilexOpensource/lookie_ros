"""Robot capabilities from kstone ``~/discover`` (design §3).

No robot model names here: the headset-driven axes come from the ``body_axes`` parameter and
are intersected with what the discovered hardware can do. Mapping code checks ``axes`` only.
"""

from dataclasses import dataclass, field

import numpy as np

ALL_AXES = frozenset({"chassis", "waist_x", "waist_z", "waist_pitch", "waist_yaw"})
# Waist DOFs a joint-driven waist may map (``torso_joints``), each to the waist_<role> body axis.
TORSO_ROLES = ("x", "z", "pitch", "yaw")
# Bases that accept vy (stick lateral and headset lateral following).
HOLONOMIC = frozenset({"omni"})


@dataclass
class PosePort:
    topic: str
    ros_frame: str
    home_position: np.ndarray
    home_quat: np.ndarray


@dataclass
class VectorPort:
    topic: str
    labels: list
    lower: np.ndarray
    upper: np.ndarray


@dataclass
class Profile:
    kinematics: str          # discover chassis_kinematics, "" without a chassis
    axes: frozenset          # headset-driven axes
    world_frame: str
    chassis_frame: str
    pose_ports: dict = field(default_factory=dict)      # module id -> PosePort
    vector_ports: dict = field(default_factory=dict)    # module id -> VectorPort
    chassis_velocity: VectorPort | None = None
    chassis_pose_topic: str = ""
    chassis_pose_frame: str = ""
    feedback_topics: dict = field(default_factory=dict)  # module id -> PoseStamped feedback topic
    homeable: set = field(default_factory=set)

    def has(self, axis: str) -> bool:
        return axis in self.axes

    @property
    def holonomic(self) -> bool:
        """Chassis accepts vy (stick lateral and headset pose following)."""
        return self.kinematics in HOLONOMIC


def parse_torso_joints(entries) -> list:
    """``torso_joints`` entries ``role:joint`` (``role:-joint`` when the joint turns against the waist
    convention: x forward, z up, pitch > 0 leaning forward, yaw > 0 left) -> [(role, joint, sign)]."""
    out = []
    for e in entries:
        role, sep, joint = (v.strip() for v in str(e).partition(":"))
        sign = -1.0 if joint.startswith("-") else 1.0
        joint = joint.lstrip("-").strip()
        if not sep or role not in TORSO_ROLES or not joint:
            raise ValueError(f"torso_joints entry {e!r} is not role:joint with role in {list(TORSO_ROLES)}")
        if any(r == role for r, _, _ in out):
            raise ValueError(f"torso_joints maps {role} twice")
        out.append((role, joint, sign))
    return out


def _bounds(values, default):
    return np.array([default if v is None else float(v) for v in values], dtype=float)


def parse_discover(doc: dict, body_axes=ALL_AXES) -> Profile:
    unknown = set(body_axes) - ALL_AXES
    if unknown:
        raise ValueError(f"unknown body_axes {sorted(unknown)} (valid: {sorted(ALL_AXES)})")
    modules = {m["id"]: m for m in doc.get("modules", [])}
    chassis = modules.get("chassis")
    kinematics = chassis.get("chassis_kinematics", "") if chassis else ""
    axes = set(body_axes)
    profile = Profile(
        kinematics=kinematics,
        axes=frozenset(axes),
        world_frame=doc.get("world_frame", "world"),
        chassis_frame=chassis.get("chassis_frame", "chassis_base_link") if chassis else "chassis_base_link",
    )
    for mid, module in modules.items():
        if module.get("homeable"):
            profile.homeable.add(mid)
        for iface in module.get("ros_interfaces", []):
            if iface.get("semantics") == "feedback" and iface.get("type", "").endswith("PoseStamped"):
                profile.feedback_topics[mid] = iface["topic"]
        for port in module.get("ports", []):
            if not port.get("ros_exposed", True):
                continue
            if mid == "chassis":
                if port["name"] == "velocity":
                    profile.chassis_velocity = VectorPort(
                        port["control_topic"], list(port.get("labels", [])),
                        _bounds(port["lower"], -np.inf), _bounds(port["upper"], np.inf))
                elif port["name"] == "pose_target":
                    profile.chassis_pose_topic = port["control_topic"]
                    profile.chassis_pose_frame = port.get("ros_frame", profile.world_frame)
            elif port.get("type") == "pose" and "home_pose" in port:
                home = port["home_pose"]
                profile.pose_ports[mid] = PosePort(
                    port["control_topic"], port.get("ros_frame", profile.chassis_frame),
                    np.asarray(home["position"], dtype=float), np.asarray(home["orientation"], dtype=float))
            elif port.get("type") == "vector" and port.get("labels"):
                n = len(port["labels"])
                profile.vector_ports[mid] = VectorPort(
                    port["control_topic"], list(port["labels"]),
                    _bounds(port.get("lower", [None] * n), -np.inf), _bounds(port.get("upper", [None] * n), np.inf))
    if not profile.chassis_pose_topic:
        profile.axes = profile.axes - {"chassis"}   # headset following goes through the pose target
    return profile
