"""Driver config consistency for every lookie_bringup model directory.

URDF checks run only when lookie_description is a sibling checkout.
"""

from pathlib import Path

import pytest
import yaml

from lookie_driver.joint_bridge_node import _parse_module_topics
from lookie_driver.layout import validate_namespace
from lookie_driver.models import actuated_urdf_joints, torso_key, validate

PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR.parent / 'lookie_bringup' / 'config'
DESCRIPTION_DIR = PACKAGE_DIR.parents[1] / 'lookie_description'
MODELS = sorted(p.name for p in CONFIG_DIR.iterdir() if (p / 'bringup.yaml').is_file())
needs_urdf = pytest.mark.skipif(
    not (DESCRIPTION_DIR / 'urdf').is_dir(), reason='lookie_description not checked out')

HAND_JOINTS = (
    'thumb_metacarpal_joint',
    'thumb_proximal_joint',
    'index_proximal_joint',
    'middle_proximal_joint',
    'ring_proximal_joint',
    'pinky_proximal_joint',
)


def _paths(model: str) -> dict:
    d = CONFIG_DIR / model
    return {'robot_yaml': d / 'driver_motors.yaml', 'params_file': d / 'driver_ros.yaml',
            'bridge_config': d / 'driver_joint_bridge.yaml'}


def _urdf(model: str) -> Path:
    return DESCRIPTION_DIR / 'urdf' / f'{model}.urdf'


def _write(tmp_path, name: str, data) -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return path


def test_models_found():
    assert MODELS


@pytest.mark.parametrize('model', MODELS)
def test_configs_consistent_without_urdf(model):
    validate(*_paths(model).values())


@needs_urdf
@pytest.mark.parametrize('model', MODELS)
def test_configs_match_urdf(model):
    validate(*_paths(model).values(), _urdf(model))


@pytest.mark.parametrize('model', MODELS)
def test_bridge_uses_driver_side_names(model):
    paths = _paths(model)
    bridge = yaml.safe_load(paths['bridge_config'].read_text(encoding='utf-8'))
    params = yaml.safe_load(paths['params_file'].read_text(encoding='utf-8'))
    for module in bridge['modules']:
        if 'joint_map' not in module:
            continue
        node = f"/**/{module['name']}/agx_arm_ctrl_single_node"
        effector = params[node]['ros__parameters']['effector_type']
        prefix = 'left_' if module['name'] == 'left_arm' else 'right_'
        local = set(module['joint_map'])
        arm = {f'joint{i}' for i in range(1, 8)}
        if effector == 'revo2':
            assert local == arm | {prefix + j for j in HAND_JOINTS}
        else:
            assert local == arm | {'gripper'}


def test_torso_key_from_motor_yaml(tmp_path):
    keys = {torso_key(_paths(m)['robot_yaml']) for m in MODELS}
    assert keys <= {'lift', 'fold'}
    with pytest.raises(ValueError, match='expected one torso module'):
        torso_key(_write(tmp_path, 'm.yaml', {'head': {}, 'lift': {}, 'fold': {}}))
    with pytest.raises(ValueError, match='expected one torso module'):
        torso_key(_write(tmp_path, 'm.yaml', {'head': {}}))


def test_mismatched_torso_rejected():
    lift = next(m for m in MODELS if torso_key(_paths(m)['robot_yaml']) == 'lift')
    fold = next(m for m in MODELS if torso_key(_paths(m)['robot_yaml']) == 'fold')
    with pytest.raises(ValueError, match='Driver config mismatch'):
        validate(_paths(lift)['robot_yaml'], _paths(fold)['params_file'],
                 _paths(fold)['bridge_config'])


def test_bridge_module_namespace_mismatch_rejected(tmp_path):
    paths = _paths(MODELS[0])
    bridge = yaml.safe_load(paths['bridge_config'].read_text(encoding='utf-8'))
    torso = next(m for m in bridge['modules'] if m.get('namespace') == 'torso')
    torso['namespace'] = 'fold'
    with pytest.raises(ValueError, match="namespace 'fold' != 'torso'"):
        validate(paths['robot_yaml'], paths['params_file'],
                 _write(tmp_path, 'bridge.yaml', bridge))


def test_torque_limit_on_torso_rejected(tmp_path):
    paths = _paths(MODELS[0])
    motors = yaml.safe_load(paths['robot_yaml'].read_text(encoding='utf-8'))
    torso = torso_key(paths['robot_yaml'])
    motors[torso]['joints'][-1]['torque_limit'] = 50.0
    with pytest.raises(ValueError, match='torque_limit is not applied'):
        validate(_write(tmp_path, 'motors.yaml', motors), paths['params_file'], paths['bridge_config'])


def test_missing_params_section_rejected(tmp_path):
    paths = _paths(MODELS[0])
    params = yaml.safe_load(paths['params_file'].read_text(encoding='utf-8'))
    del params['/**/head_node']
    with pytest.raises(ValueError, match='missing /\\*\\*/head_node'):
        validate(paths['robot_yaml'], _write(tmp_path, 'ros.yaml', params),
                 paths['bridge_config'])


def test_namespace_validation():
    assert validate_namespace('/trio/') == 'trio'
    for bad in ('', '1x', 'a-b', 'a//b'):
        with pytest.raises(ValueError, match='invalid namespace'):
            validate_namespace(bad)


def test_module_topics_relative_to_node_namespace():
    assert _parse_module_topics({'namespace': 'torso'}, 'fold') == (
        'torso/feedback/joint_states', 'torso/control/joint_states')
    assert _parse_module_topics(
        {'feedback_topic': '/abs/fb', 'control_topic': 'rel/ctl'}, 'x') == ('/abs/fb', 'rel/ctl')


@needs_urdf
def test_mismatched_bridge_rejected_by_urdf():
    def effector(m):
        return yaml.safe_load(_paths(m)['params_file'].read_text(encoding='utf-8'))[
            '/**/left_arm/agx_arm_ctrl_single_node']['ros__parameters']['effector_type']
    model = next(m for m in MODELS if effector(m) == 'revo2')
    other = next(m for m in MODELS if effector(m) != 'revo2'
                 and torso_key(_paths(m)['robot_yaml']) == torso_key(_paths(model)['robot_yaml']))
    paths = _paths(model)
    with pytest.raises(ValueError, match='not actuated in URDF'):
        validate(paths['robot_yaml'], paths['params_file'], _paths(other)['bridge_config'],
                 _urdf(model))


@needs_urdf
def test_limit_wider_than_urdf_rejected(tmp_path):
    model = MODELS[0]
    paths = _paths(model)
    motors = yaml.safe_load(paths['robot_yaml'].read_text(encoding='utf-8'))
    torso = torso_key(paths['robot_yaml'])
    joint = motors[torso]['joints'][0]
    joint['upper_limit'] = actuated_urdf_joints(_urdf(model))[joint['joint_name']][1] + 0.1
    with pytest.raises(ValueError, match='exceed URDF'):
        validate(_write(tmp_path, 'motors.yaml', motors), paths['params_file'],
                 paths['bridge_config'], _urdf(model))


@needs_urdf
@pytest.mark.parametrize('model', MODELS)
def test_mimic_and_wheel_joints_not_actuated(model):
    joints = actuated_urdf_joints(_urdf(model))
    assert not any('distal' in name or 'gripper_joint' in name for name in joints)
    assert not any('wheel' in name for name in joints)
