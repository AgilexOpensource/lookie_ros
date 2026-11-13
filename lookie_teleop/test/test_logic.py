"""Unit tests for the pure teleop logic (design §16)."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from lookie_teleop import calibration, safety
from lookie_teleop.buttons import DoubleTapHold, Hysteresis, LongPress, ShortLongPress
from lookie_teleop.catchup import CatchUp, step_toward
from lookie_teleop.frames import Mount, heading_frame, invert, rpy_to_matrix
from lookie_teleop.home import HomeTracker, parse_request_id
from lookie_teleop.mapping import (ArmMap, Chassis, ChassisConfig, Waist, WaistConfig,
                                       apply_deadzone, ee_target, to_base)
from lookie_teleop.profile import ALL_AXES, parse_discover
from lookie_teleop.workspace import WorkspaceConfig, project

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
    x_range=(0.15, 0.7), z_range=(-0.6, 0.3), min_cross=0.1, max_side=0.2, spacing_tol=0.1,
    rel_ori_warn=math.radians(45))
EYE_C = {"left": np.eye(3), "right": np.eye(3)}


def calib_inputs(spacing=0.4, headset=None):
    headset = pose((0.0, 0.0, 1.4)) if headset is None else headset
    ctrl = {"left": headset @ pose((0.35, spacing / 2, -0.25)),
            "right": headset @ pose((0.35, -spacing / 2, -0.25))}
    ee = {"left": pose((0.35, 0.2, -0.25)), "right": pose((0.35, -0.2, -0.25))}
    return headset, ctrl, ee


def test_calibration_accepts_nominal_pose():
    reason, ref = calibration.check(*calib_inputs(), np.eye(3), EYE_C, CAL)
    assert reason is None
    np.testing.assert_allclose(ref.M0, [0.35, 0.0, -0.25], atol=1e-9)
    assert ref.zh0 == pytest.approx(1.4) and ref.warnings == []


def test_calibration_rejects_spacing_mismatch():
    reason, ref = calibration.check(*calib_inputs(spacing=0.7), np.eye(3), EYE_C, CAL)
    assert ref is None and "间距" in reason


def test_calibration_rejects_tilted_mount():
    reason, _ = calibration.check(*calib_inputs(headset=pose((0, 0, 1.4), (0.0, 0.9, 0.0))),
                                  np.eye(3), EYE_C, CAL)
    assert "倾斜" in reason


def test_calibration_rejects_crossed_controllers():
    headset, ctrl, ee = calib_inputs()
    ctrl = {"left": ctrl["right"], "right": ctrl["left"]}
    reason, _ = calibration.check(headset, ctrl, ee, np.eye(3), EYE_C, CAL)
    assert "交叉" in reason


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
    assert w.target[1] == pytest.approx(cfg.pitch_range[1])
    slow = Waist([0.0, 0.0, 1.0], 0.0, 0.0, WaistConfig(z_rate=0.1), ALL_AXES)
    slow.set_body_reference(1.4, 0.0)
    slow.update(0.1, (1.2, 0.0, 0.0))
    assert slow.position[2] == pytest.approx(0.99)


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
    cfg = WaistConfig(pitch_range=(-2.0, 2.0), pitch_rate=100.0)
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
    assert w.target[1] == pytest.approx(-math.pi / 4)


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
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0, v_max=10, w_max=10))
    fh0 = heading_frame([0.0, 0.0, 1.4], 0.0)
    ch.record([1.0, 2.0, math.pi / 2], fh0)
    target = ch.update(heading_frame([0.5, 0.0, 1.4], 0.2), 1.0)
    np.testing.assert_allclose(target, [1.0, 2.5, math.pi / 2 + 0.2], atol=1e-9)


def test_chassis_deadband_and_rate_limit():
    ch = Chassis(ChassisConfig(deadband_pos=0.02, v_max=0.5))
    fh0 = heading_frame([0, 0, 0], 0.0)
    ch.record([0.0, 0.0, 0.0], fh0)
    np.testing.assert_allclose(ch.update(heading_frame([0.01, 0, 0], 0.0), 0.02), [0, 0, 0])
    assert ch.update(heading_frame([2.0, 0, 0], 0.0), 0.02)[0] == pytest.approx(0.01)


def test_chassis_target_clamped_to_feedback_deviation():
    ch = Chassis(ChassisConfig(deadband_pos=0.0, deadband_ang=0.0, v_max=10, w_max=10,
                               max_dev_pos=0.2, max_dev_ang=math.radians(15)))
    ch.record([0.0, 0.0, 0.0], heading_frame([0, 0, 0], 0.0))
    t = ch.update(heading_frame([1.0, 0, 0], 1.0), 1.0, feedback=[0.1, 0.0, 0.0])   # base lags
    np.testing.assert_allclose(t, [0.3, 0.0, math.radians(15)], atol=1e-9)        # chased at the limit
    t = ch.update(heading_frame([1.0, 0, 0], 1.0), 1.0, feedback=[0.9, 0.0, 0.95])
    np.testing.assert_allclose(t, [1.0, 0.0, 1.0], atol=1e-9)                      # inside: unclamped


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

WS = WorkspaceConfig(shoulder_offsets={"left": [0, 0.2, 0.05], "right": [0, -0.2, 0.05]})


def test_workspace_passes_reachable_targets():
    t = {"left": np.array([0.4, 0.2, 0.0]), "right": np.array([0.4, -0.2, 0.0])}
    out, clamped = project(t, WS, np.eye(3))
    assert clamped == set()
    np.testing.assert_allclose(out["left"], t["left"])


def test_workspace_clamps_reach_and_crossing():
    out, clamped = project({"left": np.array([2.0, 0.2, 0.0]), "right": np.array([0.4, 0.3, 0.0])},
                           WS, np.eye(3))
    assert clamped == {"left", "right"}
    assert np.linalg.norm(out["left"] - [0, 0.2, 0.05]) <= WS.r_max + 1e-6
    assert out["left"][1] - out["right"][1] >= WS.min_hand_gap - 1e-6


def test_workspace_fixed_arm_bounds_moving_arm():
    out, _ = project({"left": np.array([0.4, -0.15, 0.0])}, WS, np.eye(3),
                     {"right": np.array([0.4, -0.1, 0.0])})
    assert out["left"][1] >= -0.1 + WS.min_hand_gap - 1e-6


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
    assert cu.step(np.zeros(3), Q0, np.array([5.0, 0, 0]), Q0, 0.02, 2.5, (0, 0))[2] == "追赶超时"
    assert cu.step(np.zeros(3), Q0, goal, Q0, 0.02, 0.1, (1.0, 0))[2] == "手柄动作过快"


# --- safety ----------------------------------------------------------------------------------

def state(**mods):
    return {"fresh": True, "fault": "", "modules": {
        m: {"enabled": True, "held_by": h} for m, h in mods.items()}}


def test_kstone_health():
    assert safety.kstone_health(state(torso_pose=None), 0.0, 0.3, True, ["torso_pose"]) is None
    assert "超时" in safety.kstone_health(state(), 1.0, 0.3, True, [])
    assert "抢占" in safety.kstone_health(state(torso_pose="torso_joint"), 0.0, 0.3, True, ["torso_pose"])
    stale = state(torso_pose=None)
    stale["fresh"] = False
    assert safety.kstone_health(stale, 0.0, 0.3, False, ["torso_pose"]) is None
    assert safety.kstone_health(stale, 0.0, 0.3, True, ["torso_pose"])


def test_xr_monitor_timeout_jump_and_reach():
    mon = safety.XrMonitor(0.2, 0.1, math.radians(30), 1.0)
    assert mon.stale({"headset": 0.0}, 0.3) and mon.stale({"left": None}, 0.0)
    assert mon.stale({"headset": 0.25}, 0.3) is None
    h = pose((0, 0, 1.4))
    assert mon.check({"headset": h, "left": pose((0.3, 0.2, 1.2))}, {"headset": 1.0, "left": 1.0}) is None
    assert "跳变" in mon.check({"left": pose((0.6, 0.2, 1.2))}, {"left": 1.02})
    mon.reset()
    assert "过远" in mon.check({"headset": h, "left": pose((2.0, 0, 1.4))}, {"headset": 2.0, "left": 2.0})


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
    assert "headset" in issues and "跳变" in mon_check_first(issues)
    assert mon.stale_issues({"headset": 0.9, "left": None, "right": 1.0}, 1.0) == {"left": "XR left 数据超时"}
    mon.forget("left")
    assert "left" not in mon.issues({"left": pose((5.0, 0, 0))}, {"left": 2.0})


def mon_check_first(issues):
    return issues.get("headset") or next(iter(issues.values()))


def test_error_rate_monitor_ignores_constant_error_flags_jumps():
    mon = safety.ErrorRateMonitor(1.0, 6.0, 3, 10.0)
    assert not any(mon.update(i * 0.02, 0.2, 0.3) for i in range(50))
    mon.reset()
    tripped = False
    for i in range(20):
        tripped |= mon.update(i * 0.02, 0.1 * i, 0.0)   # 5 m/s growth
    assert tripped


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
    assert ht.update({"outcomes": [], "modules": {}}, 1.5) == "failed" and "查询不到" in ht.failure

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
