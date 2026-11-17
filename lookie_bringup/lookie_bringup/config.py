"""Bringup YAML loading (no ROS runtime dependency)."""

from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Callable

import yaml

from lookie_driver.layout import validate_namespace

PACKAGE_SCHEME = 'package://'
BRINGUP_FILE = 'bringup.yaml'
DESCRIPTION_KEYS = ('urdf', 'collision_urdf', 'initial')
DRIVER_KEYS = ('robot_yaml', 'params_file', 'bridge_config')
MODEL_ENV = 'LOOKIE_MODEL'


@dataclass(frozen=True)
class BringupConfig:
    namespace: str
    urdf: Path
    collision_urdf: Path
    initial: Path
    robot_yaml: Path
    params_file: Path
    bridge_config: Path


@dataclass(frozen=True)
class BridgeTopics:
    feedback_topic: str  # relative to the driver root <namespace>/driver
    control_topic: str  # relative to the driver root <namespace>/driver


def _load_mapping(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping at top level')
    return data


def resolve_path(value: str, share_lookup: Callable[[str], str], base=None) -> Path:
    """Resolve ``package://<pkg>/<relative>``, an absolute path, or a path relative to ``base``."""
    value = str(value).strip()
    if value.startswith(PACKAGE_SCHEME):
        package, _, relative = value[len(PACKAGE_SCHEME):].partition('/')
        rel = PurePosixPath(relative)
        if not package or not relative or rel.is_absolute() or '..' in rel.parts:
            raise ValueError(f'invalid package path: {value!r}')
        path = Path(share_lookup(package)) / rel
    else:
        path = Path(value)
        if not path.is_absolute():
            if base is None or not value:
                raise ValueError(f'path must be absolute or package://, got {value!r}')
            path = Path(base) / path
    if not path.is_file():
        raise FileNotFoundError(f'file not found: {path}')
    return path


def available_models(config_dir) -> list:
    """Model names: subdirectories of ``config_dir`` that contain a bringup.yaml."""
    config_dir = Path(config_dir)
    if not config_dir.is_dir():
        return []
    return sorted(p.name for p in config_dir.iterdir() if (p / BRINGUP_FILE).is_file())


def load_bringup_config(path, share_lookup: Callable[[str], str]) -> BringupConfig:
    data = _load_mapping(path)
    base = Path(path).parent
    sections = {'description': DESCRIPTION_KEYS, 'driver': DRIVER_KEYS}
    missing = [] if data.get('namespace') else ['namespace']
    missing += [f'{s}.{k}' for s, keys in sections.items() for k in keys
                if k not in (data.get(s) or {})]
    if missing:
        raise ValueError(f'{path}: missing keys {missing}')
    return BringupConfig(
        namespace=validate_namespace(data['namespace']),
        **{k: resolve_path(data[s][k], share_lookup, base)
           for s, keys in sections.items() for k in keys},
    )


def select_bringup_config(model: str, value: str, share_lookup: Callable[[str], str]):
    """Return ``(path, config)``; empty ``value`` selects lookie_bringup ``config/<model>/bringup.yaml``."""
    value = value.strip()
    if value:
        path = Path(value)
        if not path.is_absolute():
            raise ValueError(f'config must be an absolute path, got {value!r}')
    else:
        config_dir = Path(share_lookup('lookie_bringup')) / 'config'
        models = available_models(config_dir)
        if not model:
            raise ValueError(f'{MODEL_ENV} is not set; export {MODEL_ENV}=<model> or pass '
                             f'model:=<model>. Available models: {models}')
        if model not in models:
            raise ValueError(f'unknown model {model!r}; available models in {config_dir}: {models}')
        path = config_dir / model / BRINGUP_FILE
    return path, load_bringup_config(path, share_lookup)


@dataclass(frozen=True)
class TeleopConfig:
    xr_media: Path
    lookie_teleop: tuple  # parameter files in load order


def load_teleop_config(path, share_lookup: Callable[[str], str]) -> TeleopConfig:
    teleop = _load_mapping(path).get('teleop') or {}
    files = teleop.get('lookie_teleop')
    if isinstance(files, str):
        files = [files]
    missing = [f'teleop.{k}' for k, v in (('xr_media', teleop.get('xr_media')), ('lookie_teleop', files)) if not v]
    if missing:
        raise ValueError(f'{path}: missing keys {missing}')
    return TeleopConfig(
        xr_media=resolve_path(teleop['xr_media'], share_lookup, Path(path).parent),
        lookie_teleop=tuple(resolve_path(p, share_lookup, Path(path).parent) for p in files),
    )


def load_kstone_config(path, share_lookup: Callable[[str], str]) -> Path:
    """Return the kstone engine config selected by ``kstone.config``."""
    value = (_load_mapping(path).get('kstone') or {}).get('config')
    if not value:
        raise ValueError(f'{path}: missing keys [\'kstone.config\']')
    return resolve_path(value, share_lookup, Path(path).parent)


def load_bridge_topics(bridge_config) -> BridgeTopics:
    data = _load_mapping(bridge_config)
    return BridgeTopics(
        feedback_topic=str(data.get('feedback_topic', 'feedback/joint_states')).strip('/'),
        control_topic=str(data.get('control_topic', 'control/joint_states')).strip('/'),
    )


BRIDGE_FEEDBACK_TIMEOUT_DEFAULT = 0.15  # joint_bridge_node default when the YAML omits it


def feedback_timeout_warnings(bridge_config, kstone_config) -> list:
    """Cross-file timing rule: keep the bridge timeout at or below kstone's.

    The bridge restamps cached feedback, so kstone only sees the republish time. After a driver
    goes silent, kstone detaches its group only after bridge + kstone timeouts, solving on frozen
    joint positions until then; a larger bridge timeout makes that window longer.
    Returns warnings; never raises.
    """
    try:
        bridge = float(_load_mapping(bridge_config).get('feedback_timeout_s',
                                                        BRIDGE_FEEDBACK_TIMEOUT_DEFAULT))
        kstone = _load_mapping(kstone_config).get('feedback_timeout_s')
        if kstone is None:
            return []
        kstone = float(kstone)
    except Exception as exc:  # noqa: BLE001 - a check must not stop the launch
        return [f'could not compare feedback timeouts: {exc}']
    if 0.0 < bridge <= kstone:
        return []
    return [f'joint bridge feedback_timeout_s ({bridge}) must be > 0 and at most kstone '
            f'feedback_timeout_s ({kstone}); a silent driver is detected only after {bridge + kstone:.2f} s, '
            f'with kstone solving on its frozen joint positions until then '
            f'({bridge_config}, {kstone_config})']
