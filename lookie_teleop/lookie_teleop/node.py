"""ROS 2 node: xr_media_ros -> kstone_ros whole-body teleop (docs/vr_teleop_dev_design.md).

Holds the teleop state machine (未就绪 / 回Home / 已归位 / 已使能 / 失能) and the per-arm
substates (停住 / 追赶 / 激活); all geometry lives in the pure modules.
"""

import json
import math
import signal
from functools import partial

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import calibration
from .buttons import DoubleTapHold, Hysteresis, LongPress, ShortLongPress
from .catchup import CatchUp, limit_step, step_toward
from .frames import heading_frame, invert, matrix_to_pose, pitch_of, planar, pose_to_matrix, \
    rpy_to_matrix, rotation_angle, tcp_transform
from .home import HomeTracker, parse_request_id
from .mapping import ArmMap, Chassis, ChassisConfig, Waist, WaistConfig, apply_deadzone, ee_target, \
    to_base
from .profile import parse_discover
from .safety import ErrorRateMonitor, XrMonitor, kstone_health
from .transforms import matrix_to_quat, quat_to_matrix, yaw_of
from .workspace import SIDES, WorkspaceConfig, project

NOT_READY, HOMING, HOMED, ENABLED, DISABLED = "未就绪", "回Home", "已归位", "已使能", "失能"
STOPPED, CATCHING, ACTIVE = "停住", "追赶", "激活"
ARM_MODULES = {"left": "left_arm_pose", "right": "right_arm_pose"}
XR_NAMES = {"headset": "headset", "left": "left_controller", "right": "right_controller"}
# Joy layout from xr_media_ros: axes [sx, sy, trigger, grip], buttons [trigger, grip, stick, X/A, Y/B]
AX_SX, AX_SY, AX_TRIGGER, AX_GRIP = 0, 1, 2, 3
BTN_XA, BTN_YB = 3, 4

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

PARAMS = {
    "xr_namespace": "/xr_media", "kstone_namespace": "kstone", "rate_hz": 50.0,
    # axes the headset may drive; chassis is dropped unless discover reports a holonomic base
    "body_axes": ["chassis", "waist_z", "waist_pitch", "waist_yaw"],
    "long_press_s": 1.0, "double_tap_s": 0.4, "grip_on": 0.6, "grip_off": 0.4,
    "deadzone": 0.15, "max_vx": 0.5, "max_vy": 0.5, "max_wz": 1.0,
    "joy_z_rate": 0.10, "joy_pitch_rate": 0.3,
    "headset_forward_axis": [1.0, 0.0, 0.0], "mount_max_deg": 35.0, "yaw_warn_deg": 15.0,
    "calib_x": [0.15, 0.70], "calib_z": [-0.6, 0.3], "min_cross": 0.10, "max_side": 0.20,
    "spacing_tol": 0.10, "rel_ori_warn_deg": 45.0,
    "R_body_to_torso": [0.0, 0.0, 0.0], "controller_tcp": [0.0] * 6,
    "k_z": 1.0, "k_pitch": 1.0, "waist_z_range": [-0.3, 0.1], "waist_pitch_range": [-0.2, 0.6],
    "waist_yaw_range": [-0.8, 0.8], "waist_pitch_limits_deg": [-45.0, 90.0], "waist_z_rate": 0.2, "waist_pitch_rate": 0.8, "waist_yaw_rate": 1.0,
    "chassis_scale": 1.0, "chassis_deadband_pos": 0.02, "chassis_deadband_deg": 2.0,
    "chassis_v_max": 0.5, "chassis_w_max": 1.0, "chassis_max_dev_pos": 0.2, "chassis_max_dev_deg": 15.0,
    "shoulder_offsets": [0.0, 0.2005, 0.05, 0.0, -0.2005, 0.05],
    "r_min": 0.2, "r_max": 0.75, "x_min": 0.1, "ws_z_range": [-0.6, 0.3], "midline_delta": 0.05,
    "min_hand_gap": 0.1,
    "v_catch": 0.15, "w_catch": 0.8, "catch_pos_tol": 0.02, "catch_ori_tol_deg": 5.0,
    "catch_timeout_s": 5.0, "catch_fast_v": 0.8, "catch_fast_w": 3.0,
    "v_max": 1.0, "w_max": 3.0,
    "xr_timeout_s": 0.2, "jump_pos": 0.10, "jump_ang_deg": 30.0, "max_reach": 1.0,
    "kstone_timeout_s": 0.3, "require_fresh": True,
    "track_err_rate_pos": 1.0, "track_err_rate_ang": 6.0, "track_err_rate_n": 3, "track_err_lpf_hz": 10.0,
    "cmd_vel_min_hz": 10.0, "trigger_deadzone": 0.05,
    # end effectors are never homed: they stay under the main trigger in every state
    "home_modules": ["torso_pose", "left_arm_pose", "right_arm_pose"],
    "home_timeout_s": 20.0, "outcome_timeout_s": 1.0,
    "default_body_mode": "waist",
    "ee_modules": ["left_hand_joint", "right_hand_joint"],
}
# Parameters whose default is empty: declared by type, "unset" means fall back (C: home orientation,
# ee joints/limits: discover, ee_open: zeros, ee_close: upper limit).
TYPED_PARAMS = {
    "C_left": Parameter.Type.DOUBLE_ARRAY, "C_right": Parameter.Type.DOUBLE_ARRAY,
    "ee_open": Parameter.Type.DOUBLE_ARRAY, "ee_close": Parameter.Type.DOUBLE_ARRAY,
    "ee_lower": Parameter.Type.DOUBLE_ARRAY, "ee_upper": Parameter.Type.DOUBLE_ARRAY,
    "ee_joints_left": Parameter.Type.STRING_ARRAY, "ee_joints_right": Parameter.Type.STRING_ARRAY,
}


def _stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _transform_to_matrix(msg: TransformStamped) -> np.ndarray:
    t, q = msg.transform.translation, msg.transform.rotation
    return pose_to_matrix([t.x, t.y, t.z], [q.w, q.x, q.y, q.z])


def _pose_to_matrix(msg: PoseStamped) -> np.ndarray:
    p, q = msg.pose.position, msg.pose.orientation
    return pose_to_matrix([p.x, p.y, p.z], [q.w, q.x, q.y, q.z])


class Arm:
    """Per-arm substate and the last commanded target in F_t (held there while stopped)."""

    def __init__(self, pos=None, quat=None):
        self.mode = STOPPED
        self.pos = pos
        self.quat = quat
        self.catch = None
        self.goal_proj = None


class TeleopNode(Node):
    def __init__(self):
        super().__init__("lookie_teleop")
        self.declare_parameters("", list(PARAMS.items()))
        self.declare_parameters("", list(TYPED_PARAMS.items()))
        self.p = {k: self.get_parameter(k).value for k in PARAMS}
        p = self.p

        self.lp_a = LongPress(p["long_press_s"])
        self.lp_b = ShortLongPress(p["long_press_s"])
        self.lp_x = LongPress(p["long_press_s"])
        self.lp_y = LongPress(p["long_press_s"])
        self.grip_hyst = {s: Hysteresis(p["grip_on"], p["grip_off"]) for s in SIDES}
        self.grip_tap = {s: DoubleTapHold(p["double_tap_s"]) for s in SIDES}

        self.calib_cfg = calibration.CalibConfig(
            forward=tuple(p["headset_forward_axis"]), mount_max=math.radians(p["mount_max_deg"]),
            yaw_warn=math.radians(p["yaw_warn_deg"]), x_range=tuple(p["calib_x"]),
            z_range=tuple(p["calib_z"]), min_cross=p["min_cross"], max_side=p["max_side"],
            spacing_tol=p["spacing_tol"], rel_ori_warn=math.radians(p["rel_ori_warn_deg"]))
        self.waist_cfg = WaistConfig(
            k_z=p["k_z"], k_pitch=p["k_pitch"], z_range=tuple(p["waist_z_range"]),
            pitch_range=tuple(p["waist_pitch_range"]), yaw_range=tuple(p["waist_yaw_range"]),
            pitch_limits=tuple(math.radians(v) for v in p["waist_pitch_limits_deg"]),
            z_rate=p["waist_z_rate"], pitch_rate=p["waist_pitch_rate"], yaw_rate=p["waist_yaw_rate"])
        self.chassis_cfg = ChassisConfig(
            scale=p["chassis_scale"], deadband_pos=p["chassis_deadband_pos"],
            deadband_ang=math.radians(p["chassis_deadband_deg"]),
            v_max=p["chassis_v_max"], w_max=p["chassis_w_max"], max_dev_pos=p["chassis_max_dev_pos"],
            max_dev_ang=math.radians(p["chassis_max_dev_deg"]))
        so = p["shoulder_offsets"]
        self.ws_cfg = WorkspaceConfig(
            shoulder_offsets={"left": so[0:3], "right": so[3:6]}, r_min=p["r_min"], r_max=p["r_max"],
            x_min=p["x_min"], z_range=tuple(p["ws_z_range"]), midline_delta=p["midline_delta"],
            min_hand_gap=p["min_hand_gap"])
        self.R = rpy_to_matrix(*p["R_body_to_torso"])
        self.tcp = tcp_transform(p["controller_tcp"])
        self.xr_monitor = XrMonitor(p["xr_timeout_s"], p["jump_pos"], math.radians(p["jump_ang_deg"]),
                                    p["max_reach"])
        self.err_monitor = {s: ErrorRateMonitor(p["track_err_rate_pos"], p["track_err_rate_ang"],
                                                int(p["track_err_rate_n"]), p["track_err_lpf_hz"])
                            for s in SIDES}

        self.profile = None
        self.state = NOT_READY
        self.reason = "等待运控 discover"
        self.warnings = []
        self.clamped = set()
        self.body_mode = p["default_body_mode"]
        self.stick_axis = "z"
        self.arms = {s: Arm() for s in SIDES}
        self.arm_faults = {}                          # side -> reason its controller is unusable
        self.arm_map = None
        self.mount = None
        self.waist = None
        self.chassis = Chassis(self.chassis_cfg)
        self.dual_active = False
        self.stick_was_active = False
        self.home = None

        self.xr = {k: None for k in XR_NAMES}        # name -> (4x4, header stamp, receive time)
        self.joy = {s: None for s in SIDES}           # side -> (Joy, receive time)
        self.ctrl_speed = {s: (0.0, 0.0) for s in SIDES}
        self._speed_last = {s: None for s in SIDES}
        self.kstate = None
        self.kstate_t = None
        self._err_stamp = None
        self.fb = {}                                  # module id -> 4x4 feedback
        self.ee = {}                                # side -> dict(module, names, open, close, lower, upper)
        self._ee_last = {s: (None, -1.0) for s in SIDES}
        self._last_tick = None
        self._last_status = -1.0
        self._last_cmd_vel = -1.0

        xr = p["xr_namespace"].rstrip("/")
        for key, name in XR_NAMES.items():
            self.create_subscription(TransformStamped, f"{xr}/{name}/transform",
                                     partial(self._on_xr, key), 10)
        for side in SIDES:
            self.create_subscription(Joy, f"{xr}/{side}_controller/buttons", partial(self._on_joy, side), 10)
        ks = p["kstone_namespace"].rstrip("/")
        self.create_subscription(String, f"{ks}/discover", self._on_discover, LATCHED)
        self.create_subscription(String, f"{ks}/state", self._on_state, LATCHED)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.pose_pubs = {}
        self.home_clients = {}
        self.cmd_vel_pub = None
        self.chassis_pose_pub = None
        self.joint_pub = None
        self.create_timer(1.0 / p["rate_hz"], self._tick)
        self.get_logger().info("lookie_teleop 已启动，等待 kstone discover")

    # ------------------------------------------------------------------ helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _typed(self, name):
        try:
            v = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            return None
        return list(v) if v is not None and len(v) else None

    def _stamped(self, msg_type, frame: str):
        msg = msg_type()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        return msg

    # ------------------------------------------------------------------ callbacks
    def _on_xr(self, key, msg: TransformStamped):
        self.xr[key] = (_transform_to_matrix(msg), _stamp_s(msg.header), self._now())

    def _on_joy(self, side, msg: Joy):
        self.joy[side] = (msg, self._now())

    def _on_feedback(self, mid, msg: PoseStamped):
        self.fb[mid] = _pose_to_matrix(msg)

    def _on_state(self, msg: String):
        try:
            self.kstate = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("无法解析 kstone state")
            return
        self.kstate_t = self._now()
        if self.state == HOMING:
            self._update_home(self.kstate_t)

    def _on_discover(self, msg: String):
        if self.profile is not None:
            return
        try:
            profile = parse_discover(json.loads(msg.data), self.p["body_axes"])
        except (ValueError, KeyError, json.JSONDecodeError) as e:
            self.reason = f"discover 解析失败: {e}"
            self.get_logger().error(self.reason)
            return
        self._setup(profile)

    def _setup(self, profile):
        self.profile = profile
        for mid, topic in profile.feedback_topics.items():
            self.create_subscription(PoseStamped, topic, partial(self._on_feedback, mid), 10)
        for mid, port in profile.pose_ports.items():
            self.pose_pubs[mid] = (self.create_publisher(PoseStamped, port.topic, 10), port.ros_frame)
        if profile.chassis_velocity is not None:
            self.cmd_vel_pub = self.create_publisher(Twist, profile.chassis_velocity.topic, 10)
        if profile.has("chassis") and profile.chassis_pose_topic:
            self.chassis_pose_pub = self.create_publisher(PoseStamped, profile.chassis_pose_topic, 10)
        ks = self.p["kstone_namespace"].rstrip("/")
        for mid in self.p["home_modules"]:
            self.home_clients[mid] = self.create_client(Trigger, f"{ks}/{mid}/home")
        self._setup_end_effectors(profile)
        if not profile.has("chassis"):
            self.body_mode = "waist"
        self.reason = "长按 B 回 Home"
        self.get_logger().info(f"底盘 {profile.kinematics or '无'}，头盔可控轴 {sorted(profile.axes)}")

    def _setup_end_effectors(self, profile):
        topic = None
        for side, mid in zip(SIDES, self.p["ee_modules"]):
            port = profile.vector_ports.get(mid)
            names = self._typed(f"ee_joints_{side}") or (port.labels if port else None)
            if not names:
                self.get_logger().warn(f"{mid} 无关节名，{side} 末端不受控")
                continue
            n = len(names)
            lower = np.asarray(self._typed("ee_lower") or (port.lower if port else [-np.inf] * n), float)
            upper = np.asarray(self._typed("ee_upper") or (port.upper if port else [np.inf] * n), float)
            q_open = np.asarray(self._typed("ee_open") or [0.0] * n, float)
            q_close = np.asarray(self._typed("ee_close") or upper, float)
            if not (len(lower) == len(upper) == len(q_open) == len(q_close) == n) \
                    or not np.all(np.isfinite(q_close)):
                self.get_logger().error(f"{side} 末端关节配置长度不一致或缺少 ee_close，{side} 末端不受控")
                continue
            self.ee[side] = dict(module=mid, names=list(names), open=q_open, close=q_close,
                                   lower=lower, upper=upper)
            topic = topic or (port.topic if port else f"{self.p['kstone_namespace'].rstrip('/')}/control/joint")
        if topic:
            self.joint_pub = self.create_publisher(JointState, topic, 10)

    # ------------------------------------------------------------------ inputs
    def _joy_msg(self, side, now):
        j = self.joy[side]
        if j is None or now - j[1] > self.p["xr_timeout_s"]:
            return None
        return j[0]

    @staticmethod
    def _axis(msg, i):
        return float(msg.axes[i]) if msg is not None and len(msg.axes) > i else 0.0

    @staticmethod
    def _button(msg, i):
        return bool(msg.buttons[i]) if msg is not None and len(msg.buttons) > i else False

    def _xr_stale(self, now):
        return self.xr_monitor.stale({k: (v[2] if v else None) for k, v in self.xr.items()}, now)

    def _controller(self, side):
        return self.xr[side][0] @ self.tcp

    def _update_ctrl_speed(self, side):
        pose, stamp, _ = self.xr[side]
        last = self._speed_last[side]
        if last is not None and stamp > last[1]:
            dt = stamp - last[1]
            self.ctrl_speed[side] = (float(np.linalg.norm(pose[:3, 3] - last[0][:3, 3])) / dt,
                                     rotation_angle(last[0][:3, :3].T @ pose[:3, :3]) / dt)
        if last is None or stamp > last[1]:
            self._speed_last[side] = (pose, stamp)

    def _health_modules(self):
        return ["torso_pose", *ARM_MODULES.values(), *[h["module"] for h in self.ee.values()]]

    def _kstone_reason(self, now):
        age = now - self.kstate_t if self.kstate_t is not None else math.inf
        return kstone_health(self.kstate, age, self.p["kstone_timeout_s"], self.p["require_fresh"],
                             self._health_modules())

    # ------------------------------------------------------------------ tick
    def _tick(self):
        now = self._now()
        dt = 0.0 if self._last_tick is None else min(max(now - self._last_tick, 0.0), 0.1)
        self._last_tick = now
        if self.profile is not None:
            left, right = self._joy_msg("left", now), self._joy_msg("right", now)
            self._buttons(left, right, now)
            if self.state == HOMING:
                self._update_home(now)
            elif self.state == ENABLED:
                self._run_enabled(left, right, now, dt)
            self._end_effectors(left, right, now)
        if now - self._last_status >= 0.1:
            self._last_status = now
            self._publish_status()

    def _buttons(self, left, right, now):
        b = self.lp_b.update(self._button(right, BTN_YB), now)  # "long" | "short" | None
        a = self.lp_a.update(self._button(right, BTN_XA), now)
        y = self.lp_y.update(self._button(left, BTN_YB), now)
        x = self.lp_x.update(self._button(left, BTN_XA), now)
        if b == "long" and self.state != HOMING:
            self._start_home(now)
            return
        if b == "short" and self.state in (HOMED, ENABLED):
            self._disable("用户停止（短按 B）")
            return
        if self.state not in (HOMED, ENABLED):
            return
        if a and self.state == HOMED:
            self._calibrate(now)
        if y and self.profile.has("chassis"):
            self.body_mode = "chassis" if self.body_mode == "waist" else "waist"
            if self.state == ENABLED and self.dual_active:
                self._record_references()
        if x:
            self.stick_axis = "pitch" if self.stick_axis == "z" else "z"

    # ------------------------------------------------------------------ home
    def _start_home(self, now):
        self._halt()
        self.state = HOMING
        self.reason = "回 Home 中"
        self.home = HomeTracker(self.p["home_modules"], now, self.p["home_timeout_s"],
                                self.p["outcome_timeout_s"])
        for mid in self.p["home_modules"]:
            client = self.home_clients.get(mid)
            if client is None or not client.service_is_ready():
                self.home.fail(f"{mid} Home 服务不可用")
                break
            client.call_async(Trigger.Request()).add_done_callback(partial(self._on_home_reply, self.home, mid))
        self._update_home(now)

    def _on_home_reply(self, tracker, mid, future):
        if tracker is not self.home:
            return
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001 - any transport error fails the Home
            tracker.fail(f"{mid} Home 调用失败: {e}")
        else:
            if res.success:
                tracker.accepted(mid, parse_request_id(res.message), self._now())
            else:
                tracker.fail(f"{mid} Home 被拒绝: {res.message}")
        if self.state == HOMING:
            self._update_home(self._now())

    def _update_home(self, now):
        result = self.home.update(self.kstate, now)
        if result == "succeeded":
            self.state = HOMED
            self.reason = "已归位，摆好手柄后长按 A 使能"
            self.waist = None
            self.home = None
            self.get_logger().info("回 Home 成功")
        elif result == "failed":
            reason = self.home.failure
            self.home = None
            self._disable(f"回 Home 失败: {reason}")

    # ------------------------------------------------------------------ enable / disable
    def _calibrate(self, now):
        reason = self._xr_stale(now) or self._kstone_reason(now)
        missing = [m for m in ("torso_pose", *ARM_MODULES.values()) if m not in self.fb]
        if reason is None and missing:
            reason = f"缺少反馈 {missing}"
        if reason is None:
            torso = self.fb["torso_pose"]
            ft0 = heading_frame(torso[:3, 3], yaw_of(torso[:3, :3]))
            ft0_inv = invert(ft0)
            ee = {s: ft0_inv @ self.fb[ARM_MODULES[s]] for s in SIDES}
            C = {}
            for s in SIDES:
                q = self._typed(f"C_{s}")
                C[s] = quat_to_matrix(q if q else self.profile.pose_ports[ARM_MODULES[s]].home_quat)
            ctrls = {s: self._controller(s) for s in SIDES}
            reason, ref = calibration.check(self.xr["headset"][0], ctrls, ee, self.R, C, self.calib_cfg)
        if reason is not None:
            self.reason = f"使能被拒绝: {reason}"
            self.get_logger().warn(self.reason)
            return
        self.mount = ref.mount
        self.arm_map = ArmMap(ref.M0, ref.m0, self.R, C)
        self.waist = Waist(torso[:3, 3], pitch_of(torso[:3, :3]), yaw_of(torso[:3, :3]), self.waist_cfg,
                           self.profile.axes)
        self.waist.set_body_reference(ref.zh0, ref.theta0)
        self.waist.record_yaw_reference(ref.yaw_h0)
        headset = self.xr["headset"][0]
        self._record_chassis(self.mount.heading(headset))
        self.warnings = list(ref.warnings)
        self.xr_monitor.reset()
        for s in SIDES:
            self.arms[s] = Arm()
            self.grip_tap[s].reset()
            self.err_monitor[s].reset()
            self._speed_last[s] = None
            self.ctrl_speed[s] = (0.0, 0.0)
        self.arm_faults = {}
        self.dual_active = False
        self.stick_was_active = False
        self._err_stamp = None
        self.state = ENABLED
        self.reason = ""
        self.get_logger().info("已使能" + (f"（提示: {'；'.join(self.warnings)}）" if self.warnings else ""))

    def _record_chassis(self, fh):
        fb = self.fb.get("chassis")
        if fb is not None:
            self.chassis.record(planar(fb), fh)

    def _record_references(self):
        headset = self.xr["headset"][0]
        self.waist.record_yaw_reference(self.mount.yaw(headset))
        self._record_chassis(self.mount.heading(headset))

    def _halt(self):
        """Stop publishing arm/torso/chassis commands; one zero Twist; end effectors simply stop being sent."""
        for s in SIDES:
            self.arms[s] = Arm()
            self.grip_tap[s].reset()
        self.arm_faults = {}
        self.dual_active = False
        self.clamped = set()
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(Twist())

    def _disable(self, reason):
        self._halt()
        self.state = DISABLED
        self.reason = reason
        self.get_logger().warn(f"失能: {reason}")

    # ------------------------------------------------------------------ enabled
    def _xr_issues(self, left, right, now):
        """Per-name XR problems (§12): stale transform / Joy, stamp rollback, jump, reach."""
        stale = self.xr_monitor.stale_issues({k: (v[2] if v else None) for k, v in self.xr.items()}, now)
        for s, joy in (("left", left), ("right", right)):
            if joy is None:
                stale.setdefault(s, f"XR {s} 按键数据超时")
        for name in stale:
            self.xr_monitor.forget(name)  # no spurious jump against the pre-loss pose on recovery
        fresh = [k for k in self.xr if k not in stale and self.xr[k] is not None]
        jumps = self.xr_monitor.issues({k: self.xr[k][0] for k in fresh}, {k: self.xr[k][1] for k in fresh})
        return {**jumps, **stale}

    def _run_enabled(self, left, right, now, dt):
        # Every check runs before any mapping or publishing, so a bad frame never reaches kstone.
        issues = self._xr_issues(left, right, now)
        reason = issues.get("headset") or self._kstone_reason(now)
        missing = [m for m in ("torso_pose", *ARM_MODULES.values()) if m not in self.fb]
        if reason is None and missing:
            reason = f"缺少反馈 {missing}"
        if reason is None:
            reason = self._error_rate()
        if reason is not None:
            self._disable(reason)
            return
        for s in SIDES:
            if s in issues:
                self._stop_arm(s, issues[s])
            elif s in self.arm_faults:
                self.get_logger().info(f"{s} 手柄恢复，双击侧扳机重新接管")
                del self.arm_faults[s]

        headset = self.xr["headset"][0]
        fh = self.mount.heading(headset)
        for s in SIDES:
            grip = self.grip_hyst[s].update(self._axis(left if s == "left" else right, AX_GRIP))
            event = self.grip_tap[s].update(grip, now)
            if s in self.arm_faults:
                self.grip_tap[s].reset()  # track the edge but swallow taps while the controller is lost
                continue
            self._update_ctrl_speed(s)
            if event == "engage" and self.arms[s].mode == STOPPED:
                self._engage(s, now)
            elif event == "release":
                self.arms[s] = Arm(self.arms[s].pos, self.arms[s].quat)

        # sticks: left = translation, right x = yaw, right y = selected waist offset
        dz = self.p["deadzone"]
        vx = -self.p["max_vx"] * apply_deadzone(self._axis(left, AX_SY), dz)
        vy = -self.p["max_vy"] * apply_deadzone(self._axis(left, AX_SX), dz) if self.profile.holonomic else 0.0
        wz = -self.p["max_wz"] * apply_deadzone(self._axis(right, AX_SX), dz)
        ry = apply_deadzone(self._axis(right, AX_SY), dz)
        if ry:
            rate = self.p["joy_z_rate"] if self.stick_axis == "z" else self.p["joy_pitch_rate"]
            self.waist.add_offset(self.stick_axis, -ry * rate * dt)

        dual = all(self.arms[s].mode == ACTIVE for s in SIDES)
        if dual and not self.dual_active:
            self._record_references()
        self.dual_active = dual
        drive_waist = dual and (self.body_mode == "waist" or not self.profile.has("chassis"))
        body = (self.mount.body(headset)[2, 3], self.mount.lean(headset), self.mount.yaw(headset))
        self.waist.update(dt, body if drive_waist else None)

        if not self._arms(now, dt, fh):
            return
        self._publish_pose("torso_pose", self.waist.position, matrix_to_quat(self.waist.rotation))
        self._chassis(vx, vy, wz, fh, dt, now)

    def _stop_arm(self, side, reason):
        """Controller loss stops only that arm: hold its pose, require a fresh double-tap later."""
        if side not in self.arm_faults:
            self.get_logger().warn(f"{side}臂停住: {reason}")
        self.arm_faults[side] = reason
        held = self.arms[side]
        self.arms[side] = Arm(held.pos, held.quat)
        self.grip_tap[side].reset()
        self.err_monitor[side].reset()
        self._speed_last[side] = None
        self.ctrl_speed[side] = (0.0, 0.0)

    def _engage(self, side, now):
        held = self.arms[side]
        arm = Arm(held.pos, held.quat)
        if arm.pos is None:
            ee = invert(self.waist.frame()) @ self.fb[ARM_MODULES[side]]
            arm.pos, arm.quat = ee[:3, 3].copy(), matrix_to_quat(ee[:3, :3])
        p = self.p
        arm.catch = CatchUp(p["v_catch"], p["w_catch"], p["catch_pos_tol"], math.radians(p["catch_ori_tol_deg"]),
                            p["catch_timeout_s"], p["catch_fast_v"], p["catch_fast_w"])
        arm.catch.start(now)
        arm.mode = CATCHING
        self.arms[side] = arm

    def _arms(self, now, dt, fh) -> bool:
        """Map, project, catch up / track, re-project, rate-limit and publish both arms (§8.1, §9).

        Stopped arms hold their pose in F_t, so waist Z moves them with the torso while waist
        pitch (not part of F_t) leaves every arm target unchanged in the base frame.
        """
        ft = self.waist.frame()
        ft_inv = invert(ft)
        fixed, goals = {}, {}
        for s in SIDES:
            arm = self.arms[s]
            if arm.mode == STOPPED:
                if arm.pos is None:
                    ee = ft_inv @ self.fb[ARM_MODULES[s]]
                    arm.pos, arm.quat = ee[:3, 3].copy(), matrix_to_quat(ee[:3, :3])
                fixed[s] = arm.pos
            else:
                goals[s] = self.arm_map.target(s, fh, self._controller(s))
        if not goals:
            self.clamped = set()
            self._publish_arms(ft)
            return True
        projected, clamped = project({s: g[0] for s, g in goals.items()}, self.ws_cfg,
                                     self.waist.pitch_rotation, fixed,
                                     {s: self.arms[s].goal_proj for s in goals})
        starts, speeds = {}, {}
        for s, (_, rot) in goals.items():
            arm = self.arms[s]
            arm.goal_proj = projected[s]
            starts[s] = arm.pos.copy()
            speeds[s] = arm.catch.v if arm.mode == CATCHING else self.p["v_max"]
            gquat = matrix_to_quat(rot)
            if arm.mode == CATCHING:
                arm.pos, arm.quat, status = arm.catch.step(arm.pos, arm.quat, projected[s], gquat, dt, now,
                                                           self.ctrl_speed[s])
                if status == "converged":
                    arm.mode = ACTIVE
                    self.err_monitor[s].reset()
                elif status != "catching":
                    self._disable(f"{s}臂追赶失败: {status}")
                    return False
            else:
                arm.pos, arm.quat = step_toward(arm.pos, arm.quat, projected[s], gquat,
                                                self.p["v_max"], self.p["w_max"], dt)
        final, clamped2 = project({s: self.arms[s].pos for s in goals}, self.ws_cfg,
                                  self.waist.pitch_rotation, fixed)
        self.clamped = set(clamped) | set(clamped2)
        for s in goals:
            # Re-projection may push a point out of the inner sphere by more than one step.
            self.arms[s].pos = limit_step(starts[s], final[s], speeds[s] * dt)
        self._publish_arms(ft)
        return True

    def _publish_arms(self, ft):
        for s in SIDES:
            arm = self.arms[s]
            pos, quat = matrix_to_pose(to_base(ft, arm.pos, quat_to_matrix(arm.quat)))
            self._publish_pose(ARM_MODULES[s], pos, quat)

    def _error_rate(self):
        """Feed per-arm tracking error from each new state snapshot; only jumps (its derivative) trip."""
        stamp = self.kstate.get("snapshot_stamp") if self.kstate else None
        if stamp is None or stamp == self._err_stamp:
            return None
        self._err_stamp = stamp
        mods = self.kstate.get("modules", {})
        for s in SIDES:
            if self.arms[s].mode != ACTIVE:
                continue
            out = mods.get(ARM_MODULES[s], {}).get("outputs", {})
            pe, oe = out.get("position_error"), out.get("orientation_error")
            if pe is None or oe is None:
                continue
            if self.err_monitor[s].update(float(stamp), float(pe), float(oe)):
                return f"{s}臂跟踪误差突变"
        return None

    def _chassis(self, vx, vy, wz, fh, dt, now):
        """Stick has priority; else headset pose (chassis axis, both arms active, chassis mode); else zero Twist."""
        if self.cmd_vel_pub is None:
            return
        lim = self.profile.chassis_velocity
        cmd = np.clip([vx, vy, wz], lim.lower, lim.upper)
        if np.any(cmd != 0.0):
            self.stick_was_active = True
            self._publish_twist(cmd, now)
            return
        if self.stick_was_active:
            self.stick_was_active = False
            self._record_chassis(fh)
        if (self.dual_active and self.body_mode == "chassis" and self.chassis_pose_pub is not None
                and self.chassis.ref is not None):
            fb = self.fb.get("chassis")
            x, y, yaw = self.chassis.update(fh, dt, planar(fb) if fb is not None else None)
            msg = self._stamped(PoseStamped, self.profile.chassis_pose_frame)
            msg.pose.position.x, msg.pose.position.y = float(x), float(y)
            msg.pose.orientation.w, msg.pose.orientation.z = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
            self.chassis_pose_pub.publish(msg)
            return
        self._publish_twist((0.0, 0.0, 0.0), now)

    def _publish_twist(self, cmd, now):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = (float(v) for v in cmd)
        self.cmd_vel_pub.publish(msg)
        self._last_cmd_vel = now

    def _publish_pose(self, mid, pos, quat):
        pub = self.pose_pubs.get(mid)
        if pub is None:
            return
        msg = self._stamped(PoseStamped, pub[1])
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pos)
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = \
            (float(v) for v in quat)
        pub[0].publish(msg)

    # ------------------------------------------------------------------ end effectors
    def _end_effectors(self, left, right, now):
        """Main trigger -> end-effector closure in every state, HOMING included (§10); end effectors are never homed."""
        if self.joint_pub is None:
            return
        msg = None
        for side, joy in (("left", left), ("right", right)):
            h = self.ee.get(side)
            if h is None or joy is None:
                continue
            q = ee_target(self._axis(joy, AX_TRIGGER), self.p["trigger_deadzone"], h["open"], h["close"],
                            h["lower"], h["upper"])
            last, t = self._ee_last[side]
            span = np.abs(h["close"] - h["open"])
            span = np.where(np.isfinite(span) & (span > 1e-6), span, 1.0)
            changed = last is None or np.any(np.abs(q - last) > 0.01 * span)
            if not changed and now - t < 0.1:
                continue
            self._ee_last[side] = (q, now)
            if msg is None:
                msg = self._stamped(JointState, "")
            msg.name.extend(h["names"])
            msg.position.extend(float(v) for v in q)
        if msg is not None:
            self.joint_pub.publish(msg)

    # ------------------------------------------------------------------ status
    def _publish_status(self):
        status = {
            "state": self.state,
            "chassis_kinematics": self.profile.kinematics if self.profile else None,
            "arms": {s: self.arms[s].mode for s in SIDES},
            "body_mode": self.body_mode,
            "stick_axis": self.stick_axis,
            "arm_faults": self.arm_faults,
            "clamped": sorted(self.clamped),
            "reason": self.reason,
            "warnings": self.warnings,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch may deliver a second SIGINT; don't let it abort the zero-velocity/cleanup path
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok() and node.cmd_vel_pub is not None:
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
