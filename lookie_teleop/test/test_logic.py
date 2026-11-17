"""Unit tests for the pure teleop logic (design §16)."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from lookie_teleop import calibration, safety
from lookie_teleop.buttons import DoubleTapHold, Hysteresis, LongPress, ShortLongPress
from lookie_teleop.catchup import CatchUp, step_toward
from lookie_teleop.frames import Mount, heading_frame, invert, rpy_to_matrix, wrap_angle
from lookie_teleop.home import HomeTracker, parse_request_id
from lookie_teleop.mapping import (ArmMap, Chassis, ChassisConfig, Settle, Waist, WaistConfig,
                                       YAW_LEAD_MAX, _soft_deadband, apply_deadzone, ee_target,
                                       to_base)
from lookie_teleop.profile import ALL_AXES, parse_discover
from lookie_teleop.workspace import Workspace, project

FIXTURE = Path(__file__).parent / "fixtures" / "discover_trio.json"


def pose(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


# --- profile ---------------------------------------------------------------------------------

def test_profile_omni_from_discover():
    p = parse_discover(json.loads(FIXTURE.read_text()))
    assert p.kinematics == "omni" and p.holonomic and p.axes == ALL_AXES
    assert {"left_arm_pose", "right_arm_pose", "torso_pose"} <= set(p.pose_ports)
    assert p.pose_ports["left_arm_pose"].ros_frame == "chassis_base_link"
    assert p.chassis_velocity.labels == ["vx", "vy", "wz"]
    assert p.chassis_pose_topic
    assert "left_hand_joint" in p.vector_ports and "torso_pose" in p.homeable


def test_profile_body_axes_restrict_headset_axes():
    p = parse_discover(json.loads(FIXTURE.read_text()), ["waist_pitch"])
    assert p.has("waist_pitch") and p.holonomic
    assert not (p.has("waist_z") or p.has("waist_yaw") or p.has("chassis"))


def test_profile_non_holonomic_drops_chassis_axis():
    doc = json.loads(FIXTURE.read_text())
    for m in doc["modules"]:
        if "chassis_kinematics" in m:
            m["chassis_kinematics"] = "differential"
    p = parse_discover(doc)
    assert p.kinematics == "differential" and not p.holonomic
    assert not p.has("chassis") and p.has("waist_z")


def test_profile_unknown_body_axis_raises():
    with pytest.raises(ValueError):
        parse_discover(json.loads(FIXTURE.read_text()), ["waist_roll"])


# --- calibration -----------------------------------------------------------------------------

CAL = calibration.CalibConfig(
    forward=(1.0, 0.0, 0.0), mount_max=math.radians(35), yaw_warn=math.radians(15),
    x_range=(0.15, 0.7), z_range=(-0.6, 0.3), min_cross=0.1, max_side=0.2, spacing_tol=0.1)
EYE_C = {"left": np.eye(3), "right": np.eye(3)}


def calib_inputs(spacing=0.4, headset=None):
    headset = pose((0.0, 0.0, 1.4)) if headset is None else headset
    ctrl = {"left": headset @ pose((0.35, spacing / 2, -0.25)),
            "right": headset @ pose((0.35, -spacing / 2, -0.25))}
    ee = {"left": pose((0.35, 0.2, -0.25)), "right": pose((0.35, -0.2, -0.25))}
    return headset, ctrl, ee


def test_calibration_accepts_nominal_pose():
    reason, ref = calibration.check(*calib_inputs(), np.eye(3), CAL)
    assert reason is None
    np.testing.assert_allclose(ref.M0, [0.35, 0.0, -0.25], atol=1e-9)
    assert ref.zh0 == pytest.approx(1.4) and ref.warnings == []


def test_calibration_anchors_orientation_at_a():
    # Controllers held with arbitrary grips, arms at their own orientations: at A the mapped
    # orientation equals each end effector's, and later controller rotations apply on top.
    headset, ctrl, ee = calib_inputs()
    ctrl["left"] = ctrl["left"] @ _rot4(rpy_to_matrix(0.4, -1.2, 2.0))
    ctrl["right"] = ctrl["right"] @ _rot4(rpy_to_matrix(-1.0, 0.3, -0.7))
    ee["left"] = ee["left"] @ _rot4(rpy_to_matrix(0.0, 1.5, 0.2))
    ee["right"] = ee["right"] @ _rot4(rpy_to_matrix(0.9, 0.0, -1.1))
    R = rpy_to_matrix(0.0, 0.0, 0.1)
    reason, ref = calibration.check(headset, ctrl, ee, R, CAL)
    assert reason is None
    fh = ref.mount.heading(headset)
    amap = ArmMap(ref.M0, ref.m0, R, ref.C)
    for s in ("left", "right"):
        _, rot = amap.target(s, fh, ctrl[s])
        np.testing.assert_allclose(rot, ee[s][:3, :3], atol=1e-9)
        turn = rpy_to_matrix(0.0, 0.0, 0.3)      # a yaw of the grip in the body heading frame
        moved = fh @ _rot4(turn) @ invert(fh) @ ctrl[s]
        moved[:3, 3] = ctrl[s][:3, 3]
        _, rot2 = amap.target(s, fh, moved)
        np.testing.assert_allclose(rot2, R @ turn @ R.T @ ee[s][:3, :3], atol=1e-9)


def test_calibration_spacing_mismatch_only_warns_and_keeps_hand_spacing():
    # Hard to judge by eye, so a different hand spacing is accepted; positions still align at the
    # midpoint, so the arms take the operator's spacing (the catch-up closes the gap).
    headset, ctrl, ee = calib_inputs(spacing=0.7)
    reason, ref = calibration.check(headset, ctrl, ee, np.eye(3), CAL)
    assert reason is None and any("spacing" in w for w in ref.warnings)
    fh = ref.mount.heading(headset)
    amap = ArmMap(ref.M0, ref.m0, np.eye(3), ref.C)
    left, _ = amap.target("left", fh, ctrl["left"])
    right, _ = amap.target("right", fh, ctrl["right"])
    np.testing.assert_allclose(left - right, [0.0, 0.7, 0.0], atol=1e-9)
    np.testing.assert_allclose(0.5 * (left + right), [0.35, 0.0, -0.25], atol=1e-9)


def test_calibration_rejects_tilted_mount():
    reason, _ = calibration.check(*calib_inputs(headset=pose((0, 0, 1.4), (0.0, 0.9, 0.0))),
                                  np.eye(3), CAL)
    assert "tilted" in reason


def test_calibration_rejects_crossed_controllers():
    headset, ctrl, ee = calib_inputs()
    ctrl = {"left": ctrl["right"], "right": ctrl["left"]}
    reason, _ = calibration.check(headset, ctrl, ee, np.eye(3), CAL)
    assert "crossed" in reason


def test_mount_removes_chest_tilt_but_keeps_yaw():
    headset = pose((0.1, 0.2, 1.3), (0.0, 0.3, 0.5))
    m = Mount(headset)
    assert m.pitch == pytest.approx(0.3)
    np.testing.assert_allclose(m.heading(headset), heading_frame([0.1, 0.2, 1.3], 0.5), atol=1e-9)
    assert m.lean(headset) == pytest.approx(0.0, abs=1e-9)


def test_lean_sign_independent_of_heading():
    mount = rpy_to_matrix(0.05, 0.3, 0.0)
    m = Mount(pose((0.0, 0.0, 1.3)) @ _rot4(rpy_to_matrix(0.0, 0.0, 0.5) @ mount))
    for yaw in (0.5, 0.5 + math.pi, 0.5 + math.pi / 2, -2.0):
        for lean in (0.4, -0.3):
            h = pose((1.0, -2.0, 1.3)) @ _rot4(rpy_to_matrix(0.0, lean, yaw) @ mount)
            assert m.lean(h) == pytest.approx(lean, abs=1e-9)


def _rot4(r):
    t = np.eye(4)
    t[:3, :3] = r
    return t


# --- mapping ---------------------------------------------------------------------------------

def test_armmap_identity_at_calibration_and_relative_motion():
    fh = heading_frame([0.0, 0.0, 1.4], 0.3)
    ctrl0 = fh @ pose((0.35, 0.2, -0.25))
    M0, m0 = np.array([0.35, 0.0, -0.25]), np.array([0.4, 0.0, 0.0])
    amap = ArmMap(M0, m0, np.eye(3), EYE_C)
    pos, rot = amap.target("left", fh, ctrl0)
    np.testing.assert_allclose(pos, [0.4, 0.2, 0.0], atol=1e-9)
    np.testing.assert_allclose(rot, np.eye(3), atol=1e-9)
    # Body turns with the controller: target unchanged (body-relative mapping)
    turn = heading_frame([0.0, 0.0, 1.4], 1.0)
    pos2, _ = amap.target("left", turn, turn @ invert(fh) @ ctrl0)
    np.testing.assert_allclose(pos2, pos, atol=1e-9)


def test_waist_maps_height_and_lean_with_limits_and_rate():
    cfg = WaistConfig(z_rate=10.0, pitch_rate=10.0, yaw_rate=10.0)
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, cfg, ALL_AXES)
    w.set_body_reference(1.4, 0.0)
    w.update(1.0, (1.3, 0.2, 0.0))
    assert w.position[2] == pytest.approx(0.9) and w.target[1] == pytest.approx(0.2)
    w.update(1.0, (0.0, 5.0, 0.0))
    assert w.position[2] == pytest.approx(1.0 + cfg.z_range[0])
    assert w.target[1] == pytest.approx(cfg.pitch_limits[1])
    slow = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(z_rate=0.1), ALL_AXES)
    slow.set_body_reference(1.4, 0.0)
    slow.update(0.1, (1.2, 0.0, 0.0))
    assert slow.position[2] == pytest.approx(0.99)


def test_waist_maps_forward_displacement_relative_unlimited_with_rate():
    cfg = WaistConfig(x_rate=10.0, z_rate=10.0, pitch_rate=10.0, yaw_rate=10.0)
    w = Waist([0.05, 0.0, 1.0], 0.0, 0.0, cfg, ALL_AXES)
    w.set_body_reference(1.4, 0.0)
    w.update(1.0, (1.4, 0.0, 0.0, 0.08))
    assert w.position[0] == pytest.approx(0.13)
    w.update(1.0, (1.4, 0.0, 0.0, 1.0))
    assert w.position[0] == pytest.approx(1.05)    # x is not range-limited
    w.update(1.0, (1.4, 0.0, 0.0, -1.0))
    assert w.position[0] == pytest.approx(-0.95)
    # Re-referenced at dual activation: the displacement counts from there.
    w.update(1.0, (1.4, 0.0, 0.0, 0.1))
    w.record_x_reference()
    w.update(1.0, (1.4, 0.0, 0.0, 0.03))
    assert w.position[0] == pytest.approx(0.18)
    held = w.position[0]
    w.update(1.0, None)
    assert w.position[0] == pytest.approx(held)
    slow = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(x_rate=0.1), ALL_AXES)
    slow.update(0.1, (1.4, 0.0, 0.0, 0.2))
    assert slow.position[0] == pytest.approx(0.01)
    pitch_only = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(x_rate=10.0), frozenset({"waist_pitch"}))
    pitch_only.update(1.0, (1.4, 0.0, 0.0, 0.1))
    assert pitch_only.position[0] == pytest.approx(0.0)


def test_waist_neo_only_pitch():
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(z_rate=10, pitch_rate=10, yaw_rate=10), frozenset({"waist_pitch"}))
    w.set_body_reference(1.4, 0.0)
    w.record_yaw_reference(0.0)
    w.update(1.0, (1.2, 0.3, 0.5))
    assert w.position[2] == pytest.approx(1.0) and w.target[2] == pytest.approx(0.0)
    assert w.target[1] == pytest.approx(0.3)


def test_waist_stick_offset_survives_hold():
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(z_rate=10), ALL_AXES)
    w.add_offset("z", -0.05)
    w.update(1.0, None)
    assert w.position[2] == pytest.approx(0.95)


def test_waist_headset_and_stick_share_the_home_range():
    cfg = WaistConfig(z_rate=10, z_range=(-0.3, 0.1))
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, cfg, ALL_AXES)
    w.set_body_reference(1.0, 0.0)           # seated at A
    w.add_offset("z", -0.2)
    w.update(1.0, None)
    assert w.position[2] == pytest.approx(0.8)
    w.update(1.0, (1.5, 0.0, 0.0))           # stands up: the headset reaches home + 0.1 despite the offset
    assert w.position[2] == pytest.approx(1.1)
    w.add_offset("z", 0.05)                  # and the stick cannot go beyond it
    w.update(1.0, (1.5, 0.0, 0.0))
    assert w.position[2] == pytest.approx(1.1)
    w.update(1.0, (1.25, 0.0, 0.0))          # sitting back down acts at once
    assert w.position[2] < 1.1


def test_waist_chassis_mode_follows_only_height_and_lean():
    cfg = WaistConfig(x_rate=10, z_rate=10, pitch_rate=10, yaw_rate=10)
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, cfg, ALL_AXES)
    w.set_body_reference(1.4, 0.0)
    w.record_yaw_reference(0.0)
    w.record_x_reference()
    w.update(1.0, (1.3, 0.2, 0.5, 0.3), frozenset({"waist_z", "waist_pitch"}))
    assert w.position[0] == pytest.approx(0.0) and w.target[2] == pytest.approx(0.0)   # x, yaw held
    assert w.position[2] == pytest.approx(0.9) and w.target[1] == pytest.approx(0.2)


def test_waist_stick_z_moves_arms_pitch_does_not():
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(z_rate=10, pitch_rate=10), ALL_AXES)
    held = [0.4, 0.2, -0.2]                  # arm target held in F_t
    before = to_base(w.frame(), held, np.eye(3))
    w.add_offset("z", -0.05)
    w.update(1.0, None)
    after_z = to_base(w.frame(), held, np.eye(3))
    np.testing.assert_allclose(after_z[:3, 3] - before[:3, 3], [0, 0, -0.05], atol=1e-9)
    w.add_offset("pitch", 0.2)
    w.update(1.0, None)
    assert w.target[1] == pytest.approx(0.2)
    np.testing.assert_allclose(to_base(w.frame(), held, np.eye(3)), after_z, atol=1e-9)


def test_waist_pitch_hard_limits_and_stick_no_windup():
    cfg = WaistConfig(pitch_rate=100.0, yaw_rate=100.0)
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, cfg, ALL_AXES)
    w.set_body_reference(1.4, 0.0)
    w.update(1.0, (1.4, 2.0, 0.0))                       # headset beyond 90° forward
    assert w.target[1] == pytest.approx(math.pi / 2)
    for _ in range(50):
        w.add_offset("pitch", 0.1)                       # keep pushing at the limit
    assert w.offset[1] == pytest.approx(0.0)
    w.add_offset("pitch", -0.1)
    w.update(1.0, (1.4, math.pi / 2, 0.0))
    assert w.target[1] == pytest.approx(math.pi / 2 - 0.1)   # reversing acts immediately
    w.update(1.0, (1.4, -2.0, 0.0))
    assert w.target[1] == pytest.approx(-math.pi / 6)   # back 30°
    w.record_yaw_reference(0.0)
    w.update(1.0, (1.4, 0.0, 1.5))                       # yaw has no range around A, only the joint limit
    assert w.target[2] == pytest.approx(1.5)


def test_waist_yaw_unwrapped_and_held_at_joint_limit():
    cfg = WaistConfig(yaw_rate=100.0)
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, cfg, ALL_AXES)
    w.record_yaw_reference(0.0)
    for deg in range(0, 271, 10):                        # operator turns left past 180°
        w.update(0.02, (0.0, 0.0, wrap_angle(math.radians(deg))))
        assert w.target[2] == pytest.approx(min(math.radians(deg), cfg.yaw_limits[1]))
    for deg in range(270, 99, -10):                      # turning back: held until inside the range again
        w.update(0.02, (0.0, 0.0, wrap_angle(math.radians(deg))))
    assert w.target[2] == pytest.approx(cfg.yaw_limits[1])
    w.update(0.02, (0.0, 0.0, math.radians(45.0)))
    assert w.target[2] == pytest.approx(math.radians(45.0))


def test_waist_reengage_realigns_z_pitch_and_rereferences_yaw():
    w = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(z_rate=10, pitch_rate=10, yaw_rate=10), ALL_AXES)
    w.set_body_reference(1.4, 0.0)
    w.record_yaw_reference(0.0)
    w.update(1.0, (1.3, 0.2, 0.3))
    w.update(1.0, None)                                  # released: operator walks and turns 90°
    w.record_yaw_reference(1.57)                         # dual-active again
    w.update(1.0, (1.35, 0.1, 1.57))
    assert w.position[2] == pytest.approx(0.95)          # absolute w.r.t. A
    assert w.target[1] == pytest.approx(0.1)
    assert w.target[2] == pytest.approx(0.3)             # yaw continues, no jump


def test_chassis_follows_body_translation_in_world():
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0, vx_max=10, vy_max=10, w_max=10))
    fh0 = heading_frame([0.0, 0.0, 1.4], 0.0)
    ch.record([1.0, 2.0, math.pi / 2], fh0)
    target = ch.update(heading_frame([0.5, 0.0, 1.4], 0.2), 1.0)
    np.testing.assert_allclose(target, [1.0, 2.5, math.pi / 2 + 0.2], atol=1e-9)


def test_chassis_deadband_and_rate_limit():
    ch = Chassis(ChassisConfig(deadband_pos=0.02, vx_max=0.5))
    fh0 = heading_frame([0, 0, 0], 0.0)
    ch.record([0.0, 0.0, 0.0], fh0)
    np.testing.assert_allclose(ch.update(heading_frame([0.01, 0, 0], 0.0), 0.02), [0.00125, 0, 0], atol=1e-12)
    assert ch.update(heading_frame([2.0, 0, 0], 0.0), 0.02)[0] == pytest.approx(0.00125 + 0.01)   # vx_max * dt


def test_chassis_rate_limit_is_per_body_axis():
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0, vx_max=1.0, vy_max=0.5))
    ch.record([0.0, 0.0, math.pi / 2], heading_frame([0, 0, 0], 0.0))
    # Base faces world +y: operator steps forward/left in its own frame -> body x / body y limits.
    assert ch.update(heading_frame([5.0, 0, 0], 0.0), 0.1)[:2] == pytest.approx([0.0, 0.1])
    ch.record([0.0, 0.0, math.pi / 2], heading_frame([0, 0, 0], 0.0))
    assert ch.update(heading_frame([0, 5.0, 0], 0.0), 0.1)[:2] == pytest.approx([-0.05, 0.0])


def test_chassis_deadband_is_smooth_at_its_edge():
    # Crossing the band must not switch the target from standstill to full speed (stop-go near zero).
    band = 0.02
    xs = np.linspace(0.0, 0.08, 801)
    ys = np.array([_soft_deadband(np.array([x, 0.0]), band)[0] for x in xs])
    slope = np.diff(ys) / np.diff(xs)
    assert ys[0] == 0.0 and np.all(np.diff(slope) >= -1e-9) and np.max(np.abs(np.diff(slope))) < 0.02
    assert _soft_deadband(np.array([0.05, 0.0]), band)[0] == pytest.approx(0.03)   # unchanged far out


def test_chassis_target_ignores_a_lagging_base():
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0, vx_max=10, vy_max=10, w_max=10))
    ch.record([0.0, 0.0, 0.0], heading_frame([0, 0, 0], 0.0))
    t = ch.update(heading_frame([1.0, 0, 0], 1.0), 1.0, feedback=[0.1, 0.0, 0.0])   # base lags
    np.testing.assert_allclose(t, [1.0, 0.0, 1.0], atol=1e-9)                      # no clamp to feedback


def test_chassis_faults_when_base_runs_away_from_target():
    # Reversed lateral axis: the target barely moves and the base keeps moving the other way; this
    # must fault instead of chasing forever.
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0))
    fh = heading_frame([0, 0, 0], 0.0)
    ch.record([0.0, 0.0, 0.0], fh)
    fb, dt, t = np.zeros(3), 0.02, 0.0
    while ch.fault is None and t < 3.0:
        ch.update(heading_frame([0.0, 0.01, 0], 0.0), dt, feedback=fb.copy())
        fb[1] -= 0.35 * dt
        t += dt
    assert ch.fault is not None and 1.8 < t < 2.5


def test_chassis_yaw_keeps_its_direction_past_180_degrees():
    # Operator turns a full circle quickly; the base lags behind. Neither the desired turn nor the
    # chased target may flip sign once more than 180° ahead (the base turned back and faulted).
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0, w_max=10))
    ch.record([0.0, 0.0, 0.0], heading_frame([0, 0, 0], 0.0))
    fb, dt, lead = np.zeros(3), 0.02, 0.0
    for i in range(1, 151):
        yaw = -math.radians(360.0) * min(1.0, i / 50)
        target = ch.update(heading_frame([0, 0, 0], yaw), dt, feedback=fb.copy())
        assert wrap_angle(target[2] - fb[2]) <= 1e-9          # always asks to keep turning negative
        lead = max(lead, abs(wrap_angle(target[2] - fb[2])))
        fb[2] = wrap_angle(fb[2] - 0.6 * dt)
    assert ch.fault is None
    assert lead == pytest.approx(YAW_LEAD_MAX)                 # the base fell far behind; lead capped


def test_chassis_lagging_base_moving_toward_target_does_not_fault():
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0, vx_max=10, vy_max=10))
    ch.record([0.0, 0.0, 0.0], heading_frame([0, 0, 0], 0.0))
    fb, dt = np.zeros(3), 0.02
    for _ in range(200):                      # operator 1 m ahead, base creeping after it
        ch.update(heading_frame([1.0, 0, 0], 0.0), dt, feedback=fb.copy())
        fb[0] += 0.2 * dt
    assert ch.fault is None
    ch.record([0.0, 0.0, 0.0], heading_frame([0, 0, 0], 0.0))
    assert ch.fault is None


def test_settle_waits_for_the_base_to_stop_after_stick_release():
    s = Settle(pos_tol=0.005, ang_tol=math.radians(0.5), hold_s=0.3, timeout_s=2.0)
    s.reset(0.0)
    t, x, v = 0.0, 0.0, 0.3
    while not s.update([x, 0.0, 0.0], t):    # coasting: delay then decelerating at 0.6 m/s^2
        t += 0.02
        if t > 0.2:
            v = max(0.0, v - 0.6 * 0.02)
        x += v * 0.02
    assert v == 0.0 and 0.9 < t < 1.3        # stopped (~0.7 s) + held 0.3 s
    s.reset(10.0)
    assert not s.update(None, 11.9) and s.update(None, 12.0)   # no feedback: timeout


def test_ee_target_and_deadzone():
    q = ee_target(1.0, 0.05, [0, 0], [1.5, 1.0], [0, 0], [1.2, 2.0])
    np.testing.assert_allclose(q, [1.2, 1.0])
    np.testing.assert_allclose(ee_target(0.03, 0.05, [0, 0], [1, 1], [-5, -5], [5, 5]), [0, 0])
    assert apply_deadzone(0.1, 0.15) == 0.0 and apply_deadzone(1.0, 0.15) == pytest.approx(1.0)


def test_to_base_composes_torso_frame():
    ft = heading_frame([0.0, 0.0, 1.0], math.pi / 2)
    out = to_base(ft, [0.5, 0.0, 0.0], np.eye(3))
    np.testing.assert_allclose(out[:3, 3], [0.0, 0.5, 1.0], atol=1e-9)


# --- workspace -------------------------------------------------------------------------------

WS = Workspace.from_home([0.3, 0.2, 0.0], [0.3, -0.2, 0.0])   # cube centre (0.3, 0, 0), half-size 0.4


def test_workspace_from_home_points():
    np.testing.assert_allclose(WS.center, [0.3, 0.0, 0.0])
    assert WS.half == pytest.approx(0.4)


def test_workspace_passes_targets_inside_and_across_the_midline():
    t = {"left": np.array([0.5, -0.15, 0.2]), "right": np.array([0.2, 0.3, -0.3])}
    out, clamped = project(t, WS)
    assert clamped == set()
    np.testing.assert_allclose(out["left"], t["left"])
    np.testing.assert_allclose(out["right"], t["right"])


def test_workspace_clamps_each_arm_on_its_own():
    out, clamped = project({"left": np.array([0.3, 0.9, 0.0]), "right": np.array([1.0, -0.1, -0.6])}, WS)
    assert clamped == {"left", "right"}
    np.testing.assert_allclose(out["left"], [0.3, 0.4, 0.0])     # half the home distance past the left home point
    np.testing.assert_allclose(out["right"], [0.7, -0.1, -0.4])


# --- catch-up --------------------------------------------------------------------------------

Q0 = np.array([1.0, 0.0, 0.0, 0.0])


def test_step_toward_rate_limited():
    p, q = step_toward(np.zeros(3), Q0, np.array([1.0, 0, 0]), Q0, 0.5, 1.0, 0.1)
    np.testing.assert_allclose(p, [0.05, 0, 0])


def test_catchup_converges_and_times_out():
    cu = CatchUp(1.0, 1.0, 0.01, 0.05, 2.0, 0.8, 3.0)
    cu.start(0.0)
    pos, status, t = np.zeros(3), None, 0.0
    goal = np.array([0.1, 0.0, 0.0])
    while status != "converged" and t < 1.0:
        t += 0.02
        pos, _, status = cu.step(pos, Q0, goal, Q0, 0.02, t, (0.0, 0.0))
    assert status == "converged"
    cu.start(0.0)
    assert cu.step(np.zeros(3), Q0, np.array([5.0, 0, 0]), Q0, 0.02, 2.5, (0, 0))[2] == "catch-up timed out"
    assert cu.step(np.zeros(3), Q0, goal, Q0, 0.02, 0.1, (1.0, 0))[2] == "controller moving too fast"


def test_workspace_clamps_after_engage_until_the_target_is_first_inside():
    from types import SimpleNamespace
    node = pytest.importorskip("lookie_teleop.node")
    arm = node.Arm(np.array([0.3, 0.2, 0.0]), Q0.copy())
    arm.mode, arm.catch = node.CATCHING, CatchUp(0.15, 0.8, 0.02, 0.09, 5.0, 0.8, 3.0)
    arm.catch.start(0.0)
    goal = np.array([0.3, 0.9, 0.0])
    other = node.Arm(np.array([0.3, -0.2, 0.0]), Q0.copy())
    published = []
    fake = SimpleNamespace(
        waist=SimpleNamespace(frame=lambda: np.eye(4)),
        arms={"left": arm, "right": other}, fb={},
        arm_map=SimpleNamespace(target=lambda *a: (goal.copy(), np.eye(3))),
        _controller=lambda s: None, p={}, ctrl_speed={"left": (0.0, 0.0)},
        ws=WS, _disable=lambda reason: pytest.fail(reason), _publish_arms=lambda ft: published.append(arm.pos.copy()))
    prev, t = arm.pos.copy(), 0.0
    while arm.mode == node.CATCHING and t < 3.0:
        t += 0.01
        assert node.TeleopNode._arms(fake, t, 0.01, np.eye(4))
        assert np.linalg.norm(published[-1] - prev) <= 0.15 * 0.01 + 1e-12
        prev = published[-1]
    assert arm.mode == node.ACTIVE and fake.clamped == {"left"}
    np.testing.assert_allclose(arm.pos, [0.3, 0.4, 0.0], atol=0.021)
    goal[:] = [0.3, 0.3, 0.0]   # back inside: the clamp is released for this engage
    for _ in range(20):
        t += 0.01
        assert node.TeleopNode._arms(fake, t, 0.01, np.eye(4))
    assert not arm.bounded and fake.clamped == set()
    goal[:] = [0.3, 0.6, 0.0]   # outside again: no longer clamped
    for _ in range(60):
        t += 0.01
        assert node.TeleopNode._arms(fake, t, 0.01, np.eye(4))
    np.testing.assert_allclose(arm.pos, [0.3, 0.6, 0.0], atol=1e-9)
    assert node.Arm().bounded   # a new engage clamps again


def test_failed_catch_up_stops_only_that_arm_until_a_new_double_tap():
    from types import SimpleNamespace
    node = pytest.importorskip("lookie_teleop.node")
    arm = node.Arm(np.array([0.3, 0.2, 0.0]), Q0.copy())
    arm.mode, arm.catch = node.CATCHING, CatchUp(0.15, 0.8, 0.02, 0.09, 5.0, 0.8, 3.0)
    arm.catch.start(0.0)
    other = node.Arm(np.array([0.3, -0.2, 0.0]), Q0.copy())
    tap = DoubleTapHold(0.4)
    for pressed, t in ((True, 0.0), (False, 0.1), (True, 0.2)):
        tap.update(pressed, t)
    assert tap.engaged
    published = []
    fake = SimpleNamespace(
        waist=SimpleNamespace(frame=lambda: np.eye(4)),
        arms={"left": arm, "right": other}, fb={},
        arm_map=SimpleNamespace(target=lambda *a: (np.array([0.3, 0.4, 0.0]), np.eye(3))),
        _controller=lambda s: None, p={}, ctrl_speed={"left": (0.0, 0.0)}, ws=WS, catch_aborts={},
        grip_tap={"left": tap}, get_logger=lambda: SimpleNamespace(warn=lambda m: None),
        _disable=lambda reason: pytest.fail(reason),
        _publish_arms=lambda ft: published.append(fake.arms["left"].pos.copy()))
    fake._abort_catch = lambda s, r: node.TeleopNode._abort_catch(fake, s, r)
    assert node.TeleopNode._arms(fake, 0.01, 0.01, np.eye(4))
    before = published[-1]
    fake.ctrl_speed["left"] = (1.0, 0.0)                    # the hand moves fast during the catch-up
    assert node.TeleopNode._arms(fake, 0.02, 0.01, np.eye(4))
    held = fake.arms["left"]
    assert held.mode == node.STOPPED and fake.catch_aborts == {"left": "controller moving too fast"}
    np.testing.assert_allclose(published[-1], before)       # held where it was, no jump
    assert fake.arms["right"] is other                      # the other arm is untouched
    assert not tap.engaged and tap.update(True, 0.3) is None  # the grip still held does not re-engage
    assert tap.update(False, 0.5) is None                   # releasing it is not a "release" event
    assert tap.update(True, 0.6) is None and tap.update(False, 0.7) is None
    assert tap.update(True, 0.8) == "engage"                # a fresh double-tap and hold catches up again


# --- safety ----------------------------------------------------------------------------------

def state(**mods):
    return {"fresh": True, "fault": "", "modules": {
        m: {"enabled": True, "held_by": h} for m, h in mods.items()}}


def test_kstone_health():
    assert safety.kstone_health(state(torso_pose=None), 0.0, 0.3, True, ["torso_pose"]) is None
    assert "timed out" in safety.kstone_health(state(), 1.0, 0.3, True, [])
    assert "held by" in safety.kstone_health(state(torso_pose="torso_joint"), 0.0, 0.3, True, ["torso_pose"])
    stale = state(torso_pose=None)
    stale["fresh"] = False
    assert safety.kstone_health(stale, 0.0, 0.3, False, ["torso_pose"]) is None
    assert safety.kstone_health(stale, 0.0, 0.3, True, ["torso_pose"])


def test_kstone_health_ignores_end_effector_groups():
    groups = [{"name": "left_hand", "fresh": True}, {"name": "right_hand", "fresh": False},
              {"name": "base", "fresh": True}, {"name": "default", "fresh": True}]
    hand_lost = {**state(), "fresh": False, "groups": groups}
    ee = ["left_hand", "right_hand"]
    assert safety.kstone_health(hand_lost, 0.0, 0.3, True, [], ignore_groups=ee) is None
    assert "right_hand" in safety.kstone_health(hand_lost, 0.0, 0.3, True, [])
    body_lost = {**hand_lost, "groups": [*groups[:3], {"name": "default", "fresh": False}]}
    assert "default" in safety.kstone_health(body_lost, 0.0, 0.3, True, [], ignore_groups=ee)


def test_xr_monitor_timeout_jump_and_reach():
    mon = safety.XrMonitor(0.2, 0.1, math.radians(30), 1.0)
    assert mon.stale({"headset": 0.0}, 0.3) and mon.stale({"left": None}, 0.0)
    assert mon.stale({"headset": 0.25}, 0.3) is None
    h = pose((0, 0, 1.4))
    assert mon.check({"headset": h, "left": pose((0.3, 0.2, 1.2))}, {"headset": 1.0, "left": 1.0}) is None
    assert "jump" in mon.check({"left": pose((0.6, 0.2, 1.2))}, {"left": 1.02})
    mon.reset()
    assert "too far" in mon.check({"headset": h, "left": pose((2.0, 0, 1.4))}, {"headset": 2.0, "left": 2.0})


def test_xr_monitor_per_name_issues_and_recenter():
    mon = safety.XrMonitor(0.2, 0.1, math.radians(30), 1.0)
    h, l, r = pose((0, 0, 1.4)), pose((0.3, 0.2, 1.2)), pose((0.3, -0.2, 1.2))
    st = {"headset": 1.0, "left": 1.0, "right": 1.0}
    assert mon.issues({"headset": h, "left": l, "right": r}, st) == {}
    # one controller jumps: only that name is reported (-> only that arm stops)
    issues = mon.issues({"headset": h, "left": pose((0.6, 0.2, 1.2)), "right": r},
                        {k: 1.02 for k in st})
    assert set(issues) == {"left"}
    # Home-key recenter: the whole XR world shifts at once -> headset jump -> full disable
    shift = pose((0.5, 0.3, 0.0), (0.0, 0.0, 1.0))
    issues = mon.issues({k: shift @ v for k, v in (("headset", h), ("left", l), ("right", r))},
                        {k: 1.04 for k in st})
    assert "headset" in issues and "jump" in mon_check_first(issues)
    assert mon.stale_issues({"headset": 0.9, "left": None, "right": 1.0}, 1.0) == {"left": "XR left data timed out"}
    mon.forget("left")
    assert "left" not in mon.issues({"left": pose((5.0, 0, 0))}, {"left": 2.0})


def test_xr_monitor_allows_fast_controller_motion_but_not_a_fast_headset():
    mon = safety.XrMonitor(0.2, 0.1, math.radians(30), 1.0, 6.0, math.radians(1200))
    h, l = pose((0, 0, 1.4)), pose((0.3, 0.2, 1.2))
    assert mon.issues({"headset": h, "left": l}, {"headset": 1.0, "left": 1.0}) == {}
    # a 5 m/s swing over a late 30 ms tick: 15 cm, within 0.10 + 6 * 0.03
    assert mon.issues({"left": pose((0.45, 0.2, 1.2))}, {"left": 1.03}) == {}
    # a 1500 deg/s wrist flick over 30 ms: 45 deg, within 30 + 1200 * 0.03
    flick = pose((0.45, 0.2, 1.2), (math.radians(45), 0.0, 0.0))
    assert mon.issues({"left": flick}, {"left": 1.06}) == {}
    # a tracking snap of 30 cm within 20 ms is still a jump
    assert "position jump" in mon.issues({"left": pose((0.75, 0.2, 1.2))}, {"left": 1.08})["left"]
    # the headset gets no allowance (a recenter is still caught)
    assert "position jump" in mon.issues({"headset": pose((0.15, 0, 1.4))}, {"headset": 1.03})["headset"]
    # after a 0.4 s gap the allowance is capped at 0.1 s: 0.10 + 6 * 0.1 = 0.7 m
    mon.reset()
    assert mon.issues({"left": l}, {"left": 2.0}) == {}
    assert "position jump" in mon.issues({"left": pose((1.1, 0.2, 1.2))}, {"left": 2.4})["left"]


def test_headset_timeout_stops_both_arms_instead_of_disabling():
    from types import SimpleNamespace
    node = pytest.importorskip("lookie_teleop.node")
    mon = safety.XrMonitor(0.5, 0.1, math.radians(30), 1.0, 6.0, math.radians(1200))
    h, l, r = pose((0, 0, 1.4)), pose((0.3, 0.2, 1.2)), pose((0.3, -0.2, 1.2))
    fake = SimpleNamespace(xr_monitor=mon, xr={"headset": (h, 1.0, 1.0), "left": (l, 1.6, 1.6), "right": (r, 1.6, 1.6)})
    issues = node.TeleopNode._xr_issues(fake, object(), object(), 1.6)
    assert issues == {"left": "XR headset data timed out", "right": "XR headset data timed out"}
    fake.xr["headset"] = (h, 1.6, 1.6)
    assert node.TeleopNode._xr_issues(fake, object(), object(), 1.62) == {}   # recovered: no spurious jump
    fake.xr["headset"] = (pose((0.5, 0.3, 1.4)), 1.62, 1.62)                 # a recenter still disables
    assert "headset" in node.TeleopNode._xr_issues(fake, object(), object(), 1.63)


def test_accepted_long_press_pulses_that_controller():
    from types import SimpleNamespace
    node = pytest.importorskip("lookie_teleop.node")
    sent = []
    pub = lambda side: SimpleNamespace(publish=lambda m: sent.append((side, list(m.data))))
    fake = SimpleNamespace(
        p={"haptic_long_press": [0.5, 80.0, 2.0, 250.0]}, state=node.HOMED, profile=SimpleNamespace(has=lambda a: True),
        body_mode="waist", stick_axis="z", dual_active=False, haptic_pubs={s: pub(s) for s in ("left", "right")},
        _haptic_queue=[], _haptic_pattern={},
        lp_b=ShortLongPress(1.0), lp_a=LongPress(1.0), lp_y=LongPress(1.0), lp_x=LongPress(1.0),
        _button=lambda msg, i: msg == i, _calibrate=lambda now: sent.append("calib"))
    fake._haptic = lambda s, pat, now: node.TeleopNode._haptic(fake, s, pat, now)
    fake._flush_haptics = lambda now: node.TeleopNode._flush_haptics(fake, now)
    for t in (0.0, 1.1):                                    # hold right A
        node.TeleopNode._buttons(fake, None, node.BTN_XA, t)
    assert sent == [("right", [0.5, 80.0]), "calib"]        # first pulse now, the second one is queued
    fake._flush_haptics(1.3)
    assert len(sent) == 2
    fake._flush_haptics(1.35)
    assert sent[2:] == [("right", [0.5, 80.0])] and fake._haptic_queue == []
    sent.clear()
    fake.p["haptic_long_press"] = [0.5, 0.0, 2.0, 250.0]    # 0 ms disables, the action still runs
    for t in (2.0, 3.1):                                    # hold left Y
        node.TeleopNode._buttons(fake, node.BTN_YB, None, t)
    fake._flush_haptics(4.0)
    assert sent == [] and fake.body_mode == "chassis"
    fake.p["haptic_long_press"] = [0.5, 80.0]               # the short form pulses once
    for t in (5.0, 6.1):                                    # hold left X
        node.TeleopNode._buttons(fake, node.BTN_XA, None, t)
    fake._flush_haptics(7.0)
    assert sent == [("left", [0.5, 80.0])] and fake.stick_axis == "pitch"


def mon_check_first(issues):
    return issues.get("headset") or next(iter(issues.values()))


# --- home ------------------------------------------------------------------------------------

def home_state(rid, home_state_, applied=True, held=None):
    return {"outcomes": [{"request_id": float(rid), "applied": applied, "error": ""}],
            "modules": {"torso_pose": {"home_state": home_state_, "held_by": held}}}


def test_parse_request_id():
    assert parse_request_id("accepted request_id=42") == 42
    assert parse_request_id("ok") is None


def test_home_tracker_success_and_failures():
    ht = HomeTracker(["torso_pose"], 0.0, 20.0, 1.0)
    ht.accepted("torso_pose", 7, 0.0)
    assert ht.update(home_state(7, "running"), 0.1) is None
    assert ht.update(home_state(7, "succeeded", held="torso_joint"), 0.2) is None
    assert ht.update(home_state(7, "succeeded"), 0.3) == "succeeded"

    ht = HomeTracker(["torso_pose"], 0.0, 20.0, 1.0)
    ht.accepted("torso_pose", 7, 0.0)
    assert ht.update({"outcomes": [], "modules": {}}, 1.5) == "failed" and "not found" in ht.failure

    ht = HomeTracker(["torso_pose"], 0.0, 20.0, 1.0)
    ht.accepted("torso_pose", 7, 0.0)
    assert ht.update(home_state(7, "idle", applied=False), 0.1) == "failed"

    ht = HomeTracker(["torso_pose"], 0.0, 20.0, 1.0)
    ht.accepted("torso_pose", None, 0.0)
    assert ht.update(None, 0.0) == "failed"


# --- buttons ---------------------------------------------------------------------------------

def test_long_press_fires_once():
    lp = LongPress(1.0)
    assert [lp.update(True, t) for t in (0.0, 0.5, 1.0, 1.5)] == [False, False, True, False]
    lp.update(False, 2.0)
    assert not lp.update(True, 2.1)


def test_long_press_reset_swallows_current_hold():
    lp = LongPress(1.0)
    lp.update(True, 0.0)
    lp.reset()
    assert not lp.update(True, 1.5)


def test_short_long_press():
    b = ShortLongPress(1.0)
    assert [b.update(p, t) for p, t in ((True, 0.0), (True, 0.5), (False, 0.6))] == [None, None, "short"]
    assert [b.update(p, t) for p, t in ((True, 1.0), (True, 2.0), (True, 2.5), (False, 3.0))] == \
        [None, "long", None, None]                       # a long press never also reports short
    assert b.update(False, 3.1) is None


def test_hysteresis():
    h = Hysteresis(0.6, 0.4)
    assert [h.update(v) for v in (0.5, 0.7, 0.5, 0.3)] == [False, True, True, False]


def test_double_tap_hold():
    d = DoubleTapHold(0.4)
    events = [d.update(p, t) for p, t in ((True, 0.0), (False, 0.1), (True, 0.2), (True, 1.0), (False, 1.1))]
    assert events == [None, None, "engage", None, "release"]
    d = DoubleTapHold(0.4)
    assert [d.update(p, t) for p, t in ((True, 0.0), (False, 0.1), (True, 0.6))] == [None, None, None]
