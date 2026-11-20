import xml.etree.ElementTree as ET
from fnmatch import fnmatch
from pathlib import Path

import pytest
import yaml

from lookie_bringup.config import (
    available_models, feedback_timeout_warnings, load_bridge_topics, load_bringup_config, load_kstone_config,
    load_teleop_config, motor_feedback_timeout_warnings, resolve_path, select_bringup_config)

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parents[1]
SHARES = {'lookie_description': SRC / 'lookie_description', 'lookie_bringup': PKG}
CONFIGS = sorted((PKG / 'config').glob('*/bringup.yaml'))


def _lookup(package):
    return str(SHARES[package])


def _model(path):
    return path.parent.name


def test_available_models_match_config_dirs():
    assert CONFIGS
    assert available_models(PKG / 'config') == [_model(p) for p in CONFIGS]
    assert 'lookie_teleop' not in available_models(PKG / 'config')


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_model_config_resolves(path):
    cfg = load_bringup_config(path, _lookup)
    assert cfg.urdf.parent == SRC / 'lookie_description' / 'urdf'
    assert cfg.collision_urdf.parent == cfg.urdf.parent
    assert cfg.robot_yaml.parent == path.parent
    assert cfg.params_file.parent == path.parent
    assert cfg.bridge_config.parent == path.parent
    topics = load_bridge_topics(cfg.bridge_config)
    assert topics.feedback_topic == 'feedback/joint_states'
    assert topics.control_topic == 'control/joint_states'


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_kstone_config_per_model(path):
    assert load_kstone_config(path, _lookup) == path.parent / 'kstone.yaml'


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_initial_pose_matches_kstone(path):
    cfg = load_bringup_config(path, _lookup)
    description = yaml.safe_load(cfg.initial.read_text())['zeros']
    kstone = yaml.safe_load(load_kstone_config(path, _lookup).read_text())['robot']['initial']
    assert set(kstone) <= set(description)
    assert {j: description[j] for j in kstone} == pytest.approx(kstone)


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_bridge_drops_silent_modules_before_kstone_feedback_timeout(path):
    bridge = yaml.safe_load((path.parent / 'driver_joint_bridge.yaml').read_text())
    kstone = yaml.safe_load((path.parent / 'kstone.yaml').read_text())
    assert 0 < bridge['feedback_timeout_s'] <= kstone['feedback_timeout_s']


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_model_feedback_timeouts_raise_no_launch_warning(path):
    assert feedback_timeout_warnings(path.parent / 'driver_joint_bridge.yaml', path.parent / 'kstone.yaml') == []


@pytest.mark.parametrize('bridge, kstone, warns', [
    ('feedback_timeout_s: 0.15', 'feedback_timeout_s: 0.25', False),
    ('feedback_timeout_s: 0.25', 'feedback_timeout_s: 0.25', False),
    ('feedback_timeout_s: 0.3', 'feedback_timeout_s: 0.25', True),
    ('feedback_timeout_s: 0', 'feedback_timeout_s: 0.25', True),
    ('modules: []', 'feedback_timeout_s: 0.1', True),  # bridge default 0.15
    ('feedback_timeout_s: 0.3', 'modules: {}', False),  # kstone timeout unset: nothing to compare
    ('feedback_timeout_s: x', 'feedback_timeout_s: 0.25', True),
])
def test_feedback_timeout_warnings(tmp_path, bridge, kstone, warns):
    (tmp_path / 'b.yaml').write_text(bridge)
    (tmp_path / 'k.yaml').write_text(kstone)
    assert bool(feedback_timeout_warnings(tmp_path / 'b.yaml', tmp_path / 'k.yaml')) == warns


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_model_motor_feedback_timeouts_raise_no_launch_warning(path):
    assert motor_feedback_timeout_warnings(path.parent / 'driver_ros.yaml', path.parent / 'kstone.yaml') == []


LIFT = '/**/lift_node:\n  ros__parameters:\n    module_key: lift\n'


@pytest.mark.parametrize('params, kstone, warns', [
    (LIFT, 'feedback_timeout_s: 0.25', False),  # driver default 0.5
    (LIFT + '    feedback_timeout: 0.35\n', 'feedback_timeout_s: 0.25', False),
    (LIFT + '    feedback_timeout: 0.28\n', 'feedback_timeout_s: 0.25', True),
    (LIFT + '    feedback_timeout: 0.0\n', 'feedback_timeout_s: 0.25', True),
    (LIFT, 'feedback_timeout_s: 0.5', True),
    (LIFT, 'modules: {}', False),
    ('/**/chassis/scout:\n  ros__parameters:\n    feedback_timeout: 0.0\n', 'feedback_timeout_s: 0.25', False),
    (LIFT + '    feedback_timeout: x\n', 'feedback_timeout_s: 0.25', True),
])
def test_motor_feedback_timeout_warnings(tmp_path, params, kstone, warns):
    (tmp_path / 'p.yaml').write_text(params)
    (tmp_path / 'k.yaml').write_text(kstone)
    assert bool(motor_feedback_timeout_warnings(tmp_path / 'p.yaml', tmp_path / 'k.yaml')) == warns


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_bridge_covers_every_independent_urdf_joint(path):
    """The driver bridge forwards exactly the joints kstone commands: no mimic follower, no locked wheel."""
    bridge = yaml.safe_load((path.parent / 'driver_joint_bridge.yaml').read_text())
    bridged = []
    for module in bridge['modules']:
        bridged += list(module['joint_map'].values()) if 'joint_map' in module else module['joints']
    kstone = yaml.safe_load((path.parent / 'kstone.yaml').read_text())
    locked = set(kstone['model'].get('locked_joints', []))
    urdf = load_bringup_config(path, _lookup).collision_urdf
    independent = {j.get('name') for j in ET.parse(urdf).getroot().findall('joint')
                   if j.get('type') != 'fixed' and j.find('mimic') is None} - locked
    assert len(bridged) == len(set(bridged))
    assert set(bridged) == independent
    # every commanded joint sits in a named feedback group, so no catch-all "default" group remains
    patterns = [p for g in kstone['feedback_groups'] for p in g['joints']]
    assert [j for j in sorted(independent) if not any(fnmatch(j, p) for p in patterns)] == []


@pytest.mark.parametrize('value', [
    'package://lookie_description/../x.yaml',
    'package://lookie_description',
    'package:///abs.yaml',
    'relative.yaml',
])
def test_resolve_rejects_bad_paths(value):
    with pytest.raises(ValueError):
        resolve_path(value, _lookup)


def test_resolve_relative_to_base(tmp_path):
    (tmp_path / 'a.yaml').write_text('{}\n', encoding='utf-8')
    assert resolve_path('a.yaml', _lookup, tmp_path) == tmp_path / 'a.yaml'


def test_resolve_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        resolve_path(str(tmp_path / 'missing.yaml'), _lookup)


def test_missing_keys(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('driver: {}\n', encoding='utf-8')
    with pytest.raises(ValueError, match="'namespace'.*description.initial"):
        load_bringup_config(path, _lookup)


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_teleop_config_resolves(path):
    cfg = load_teleop_config(path, _lookup)
    assert cfg.xr_media.parent.name == 'xr_media_ros'
    xr = yaml.safe_load(cfg.xr_media.read_text())['xr_media_node']['ros__parameters']
    assert 'media' not in xr
    assert cfg.lookie_teleop == (PKG / 'config' / 'lookie_teleop' / 'common.yaml',
                                 path.parent / 'teleop.yaml')
    merged = {}
    for p in cfg.lookie_teleop:
        merged.update(yaml.safe_load(p.read_text())['/**/lookie_teleop']['ros__parameters'])
    assert 'profile' not in merged
    assert 'chassis' in merged['body_axes']
    assert 'waist_pitch' in merged['body_axes']
    assert len(merged['ee_open']) == len(merged['ee_close'])


@pytest.mark.parametrize('path', CONFIGS, ids=_model)
def test_teleop_feedback_groups_match_kstone(path):
    merged = {}
    for p in load_teleop_config(path, _lookup).lookie_teleop:
        merged.update(yaml.safe_load(p.read_text())['/**/lookie_teleop']['ros__parameters'])
    groups = {g['name'] for g in yaml.safe_load((path.parent / 'kstone.yaml').read_text())['feedback_groups']}
    groups.add('base')   # the free root forms its own group
    keys = ('ee', 'arm', 'torso', 'chassis', 'ignored')
    listed = [g for k in keys for g in merged[f'{k}_feedback_groups']]
    assert len(merged['arm_feedback_groups']) == 2
    assert len(listed) == len(set(listed))
    # every kstone group is handled per part, so none falls back to a full disable
    assert set(listed) == groups


def test_teleop_missing_keys(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('teleop: {xr_media: x}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='teleop.lookie_teleop'):
        load_teleop_config(path, _lookup)


def test_select_bringup_config_by_model():
    model = _model(CONFIGS[0])
    path, cfg = select_bringup_config(model, '', _lookup)
    assert path == CONFIGS[0]
    assert cfg.namespace == yaml.safe_load(path.read_text())['namespace']


@pytest.mark.parametrize('model, match', [('', 'LOOKIE_MODEL is not set'), ('nope', 'unknown model')])
def test_select_bringup_config_lists_models(model, match):
    with pytest.raises(ValueError, match=match) as err:
        select_bringup_config(model, '', _lookup)
    assert _model(CONFIGS[0]) in str(err.value)


def test_select_bringup_config_by_path():
    path, _ = select_bringup_config('', str(CONFIGS[-1]), _lookup)
    assert path == CONFIGS[-1]
    with pytest.raises(ValueError, match='absolute'):
        select_bringup_config('', 'x.yaml', _lookup)


@pytest.mark.parametrize('path', sorted((SRC / 'lookie_description' / 'urdf').glob('*.urdf')),
                         ids=lambda p: p.stem)
def test_urdf_root_is_base_footprint(path):
    root = ET.parse(path).getroot()
    children = {j.find('child').get('link') for j in root.findall('joint')}
    assert [l.get('name') for l in root.findall('link') if l.get('name') not in children] == ['base_footprint']
