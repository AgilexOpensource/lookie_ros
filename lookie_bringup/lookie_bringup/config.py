"""Bringup YAML loading (no ROS runtime dependency)."""

from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Callable

import yaml

PACKAGE_SCHEME = 'package://'
DRIVER_KEYS = ('robot_yaml', 'params_file', 'bridge_config')


@dataclass(frozen=True)
class BringupConfig:
    model: str
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


def resolve_path(value: str, share_lookup: Callable[[str], str]) -> Path:
    """Resolve ``package://<pkg>/<relative>`` or an absolute path to an existing file."""
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
            raise ValueError(f'path must be absolute or package://, got {value!r}')
    if not path.is_file():
        raise FileNotFoundError(f'file not found: {path}')
    return path


def load_bringup_config(path, share_lookup: Callable[[str], str]) -> BringupConfig:
    data = _load_mapping(path)
    description = data.get('description') or {}
    driver = data.get('driver') or {}
    missing = [f'driver.{k}' for k in DRIVER_KEYS if k not in driver]
    if 'initial' not in description:
        missing.append('description.initial')
    if 'model' not in data:
        missing.append('model')
    if missing:
        raise ValueError(f'{path}: missing keys {missing}')
    return BringupConfig(
        model=str(data['model']),
        initial=resolve_path(description['initial'], share_lookup),
        **{k: resolve_path(driver[k], share_lookup) for k in DRIVER_KEYS},
    )


def select_bringup_config(model: str, value: str, share_lookup: Callable[[str], str]):
    """Return ``(path, config)``; empty ``value`` selects lookie_bringup ``config/<model>.yaml``."""
    value = value.strip()
    path = Path(value) if value else Path(share_lookup('lookie_bringup')) / 'config' / f'{model}.yaml'
    if not path.is_absolute():
        raise ValueError(f'config must be an absolute path, got {value!r}')
    cfg = load_bringup_config(path, share_lookup)
    if cfg.model != model:
        raise ValueError(f'{path}: model {cfg.model!r} != launch model {model!r}')
    return path, cfg


@dataclass(frozen=True)
class TeleopConfig:
    xr_media: Path
    wbc: tuple  # parameter files in load order


def load_teleop_config(path, share_lookup: Callable[[str], str]) -> TeleopConfig:
    teleop = _load_mapping(path).get('teleop') or {}
    wbc = teleop.get('wbc')
    if isinstance(wbc, str):
        wbc = [wbc]
    missing = [f'teleop.{k}' for k, v in (('xr_media', teleop.get('xr_media')), ('wbc', wbc)) if not v]
    if missing:
        raise ValueError(f'{path}: missing keys {missing}')
    return TeleopConfig(
        xr_media=resolve_path(teleop['xr_media'], share_lookup),
        wbc=tuple(resolve_path(p, share_lookup) for p in wbc),
    )


def load_kstone_config(path, share_lookup: Callable[[str], str]) -> Path:
    """Return the kstone engine config selected by ``kstone.config``."""
    value = (_load_mapping(path).get('kstone') or {}).get('config')
    if not value:
        raise ValueError(f'{path}: missing keys [\'kstone.config\']')
    return resolve_path(value, share_lookup)


def load_bridge_topics(bridge_config) -> BridgeTopics:
    data = _load_mapping(bridge_config)
    return BridgeTopics(
        feedback_topic=str(data.get('feedback_topic', 'feedback/joint_states')).strip('/'),
        control_topic=str(data.get('control_topic', 'control/joint_states')).strip('/'),
    )
