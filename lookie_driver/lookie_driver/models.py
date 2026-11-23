"""Lookie driver config consistency checks."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from lookie_driver.layout import CHASSIS_NAMESPACE, HEAD_MODULE, module_namespaces

# Params file keys; wildcards match any namespace prefix (e.g. /trio/driver).
NODE = '/**/{key}_node'
# The chassis section is keyed by its driver package, e.g. /**/chassis/lookie_trio_base.
CHASSIS_PREFIX = f'/**/{CHASSIS_NAMESPACE}/'

# Joint limits in driver YAML may be tighter than URDF, never wider.
LIMIT_TOLERANCE = 1e-4


def _load_yaml(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping at top level')
    return data


def chassis_packages(params: dict) -> list:
    """Chassis driver packages named by the params file chassis sections."""
    return sorted(k[len(CHASSIS_PREFIX):] for k in params
                  if k.startswith(CHASSIS_PREFIX) and '/' not in k[len(CHASSIS_PREFIX):])


def chassis_package(params_file) -> str:
    """Driver package of the single chassis section in the params file."""
    packages = chassis_packages(_load_yaml(params_file))
    if len(packages) != 1:
        raise ValueError(
            f'{params_file}: expected one {CHASSIS_PREFIX}<package> section, got {packages}')
    return packages[0]


def actuated_urdf_joints(urdf_path) -> dict:
    """Map non-fixed, non-mimic, non-wheel URDF joints to (lower, upper)."""
    joints = {}
    for joint in ET.parse(urdf_path).getroot().findall('joint'):
        name = joint.get('name')
        jtype = joint.get('type')
        if jtype in ('fixed', 'floating', 'planar') or joint.find('mimic') is not None:
            continue
        if jtype == 'continuous' or 'wheel' in name:
            continue  # chassis wheels are not driven by this bringup
        limit = joint.find('limit')
        if limit is None:
            raise ValueError(f'{urdf_path}: joint {name!r} has no <limit>')
        joints[name] = (float(limit.get('lower', '-inf')), float(limit.get('upper', 'inf')))
    return joints


def bridge_joints(bridge_config) -> dict:
    """Map bridge module name to the list of robot-side joint names."""
    data = _load_yaml(bridge_config)
    result = {}
    for module in data.get('modules') or []:
        if 'joint_map' in module:
            result[module['name']] = [str(v) for v in module['joint_map'].values()]
        else:
            result[module['name']] = [str(v) for v in module.get('joints') or []]
    return result


def motor_joint_limits(robot_yaml) -> dict:
    """Map motor YAML module key to {joint_name: (lower, upper)}."""
    data = _load_yaml(robot_yaml)
    result = {}
    for key, module in data.items():
        result[key] = {
            str(j['joint_name']): (float(j['lower_limit']), float(j['upper_limit']))
            for j in module.get('joints') or []
        }
    return result


def torso_key(robot_yaml) -> str:
    """Return the motor YAML module key that is not ``head`` (e.g. lift or fold)."""
    keys = sorted(k for k in _load_yaml(robot_yaml) if k != HEAD_MODULE)
    if len(keys) != 1:
        raise ValueError(f'{robot_yaml}: expected one torso module besides {HEAD_MODULE!r}, got {keys}')
    return str(keys[0])


def validate(robot_yaml, params_file, bridge_config, urdf_path=None) -> None:
    """Raise ValueError if driver configs are inconsistent (and, if given, mismatch the URDF)."""
    urdf = actuated_urdf_joints(urdf_path) if urdf_path is not None else None
    errors = []

    motors = motor_joint_limits(robot_yaml)
    torso = torso_key(robot_yaml)
    if HEAD_MODULE not in motors:
        errors.append(f'{robot_yaml}: missing motor module {HEAD_MODULE!r}')
    for key, joints in motors.items():
        for name, (lower, upper) in joints.items():
            if lower > upper:
                errors.append(f'{robot_yaml}: {name} lower_limit > upper_limit')
            if urdf is None:
                continue
            if name not in urdf:
                errors.append(f'{robot_yaml}: {key}.{name} is not an actuated URDF joint')
                continue
            u_lower, u_upper = urdf[name]
            if lower < u_lower - LIMIT_TOLERANCE or upper > u_upper + LIMIT_TOLERANCE:
                errors.append(
                    f'{robot_yaml}: {name} limits [{lower}, {upper}] exceed '
                    f'URDF [{u_lower}, {u_upper}]')

    # Only head_ros applies torque_limit (MIT current limit); lift/fold drivers would silently ignore it.
    for key, module in _load_yaml(robot_yaml).items():
        if key == HEAD_MODULE:
            continue
        unsupported = [str(j.get('joint_name')) for j in module.get('joints') or [] if 'torque_limit' in j]
        if unsupported or 'torque_limit' in module:
            errors.append(f'{robot_yaml}: {key}: torque_limit is not applied by this driver: {unsupported or key}')

    bridged = bridge_joints(bridge_config)
    flat = [name for names in bridged.values() for name in names]
    duplicated = sorted({name for name in flat if flat.count(name) > 1})
    if duplicated:
        errors.append(f'{bridge_config}: joints bridged more than once: {duplicated}')
    if urdf is not None:
        unknown = sorted(set(flat) - set(urdf))
        missing = sorted(set(urdf) - set(flat))
        if unknown:
            errors.append(f'{bridge_config}: joints not actuated in URDF: {unknown}')
        if missing:
            errors.append(f'{bridge_config}: URDF joints not bridged: {missing}')
    for module in (HEAD_MODULE, torso):
        if sorted(bridged.get(module, [])) != sorted(motors.get(module, {})):
            errors.append(
                f'{module}: bridge joints {bridged.get(module)} != motor joints '
                f'{sorted(motors.get(module, {}))}')

    bridge_ns = {str(m.get('name')): str(m.get('namespace', '')).strip('/')
                 for m in _load_yaml(bridge_config).get('modules') or []}
    for module, ns in module_namespaces(torso).items():
        if bridge_ns.get(module) != ns:
            errors.append(
                f'{bridge_config}: module {module!r} namespace {bridge_ns.get(module)!r} != {ns!r}')

    params = _load_yaml(params_file)
    for node in (NODE.format(key=HEAD_MODULE), NODE.format(key=torso)):
        if node not in params:
            errors.append(f'{params_file}: missing {node} section')
    chassis = chassis_packages(params)
    if len(chassis) != 1:
        errors.append(f'{params_file}: expected one {CHASSIS_PREFIX}<package> section, got {chassis}')

    if errors:
        raise ValueError('Driver config mismatch:\n  ' + '\n  '.join(errors))
