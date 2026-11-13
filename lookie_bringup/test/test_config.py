import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from lookie_bringup.config import (
    load_bridge_topics, load_bringup_config, load_kstone_config,
    load_teleop_config, resolve_path, select_bringup_config)

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
SHARES = {
    'lookie_description': SRC / 'lookie_description',
    'driver_bringup': SRC / 'lookie_driver' / 'driver_bringup',
    'lookie_bringup': PKG,
}
CONFIGS = sorted((PKG / 'config').glob('*.yaml'))


def _lookup(package):
    return str(SHARES[package])


def test_four_model_configs():
    assert len(CONFIGS) == 4


@pytest.mark.parametrize('path', CONFIGS, ids=lambda p: p.stem)
def test_model_config_resolves(path):
    cfg = load_bringup_config(path, _lookup)
    assert cfg.model == path.stem
    topics = load_bridge_topics(cfg.bridge_config)
    assert topics.feedback_topic == 'feedback/joint_states'
    assert topics.control_topic == 'control/joint_states'


@pytest.mark.parametrize('path', CONFIGS, ids=lambda p: p.stem)
def test_kstone_config_per_model(path):
    assert load_kstone_config(path, _lookup) == PKG / 'config' / 'kstone_ros' / path.name


@pytest.mark.parametrize('value', [
    'package://lookie_description/../x.yaml',
    'package://lookie_description',
    'package:///abs.yaml',
    'relative.yaml',
])
def test_resolve_rejects_bad_paths(value):
    with pytest.raises(ValueError):
        resolve_path(value, _lookup)


def test_resolve_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        resolve_path(str(tmp_path / 'missing.yaml'), _lookup)


def test_missing_keys(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('model: x\ndriver: {}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='description.initial'):
        load_bringup_config(path, _lookup)


@pytest.mark.parametrize('path', CONFIGS, ids=lambda p: p.stem)
def test_teleop_config_resolves(path):
    cfg = load_teleop_config(path, _lookup)
    assert cfg.xr_media.parent.name == 'xr_media_ros'
    xr = yaml.safe_load(cfg.xr_media.read_text())['xr_media_node']['ros__parameters']
    assert 'media' not in xr
    assert [p.name for p in cfg.wbc] == ['common.yaml', f'{path.stem}.yaml']
    merged = {}
    for p in cfg.wbc:
        merged.update(yaml.safe_load(p.read_text())['/**/lookie_wbc_teleop']['ros__parameters'])
    assert 'profile' not in merged
    expected = ['waist_pitch'] if '_neo_' in path.stem else ['chassis', 'waist_z', 'waist_pitch', 'waist_yaw']
    assert merged['body_axes'] == expected
    assert len(merged['ee_open']) == len(merged['ee_close'])


def test_teleop_missing_keys(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('teleop: {xr_media: x}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='teleop.wbc'):
        load_teleop_config(path, _lookup)



def test_select_bringup_config(tmp_path):
    model = CONFIGS[0].stem
    assert select_bringup_config(model, '', _lookup)[0] == PKG / 'config' / f'{model}.yaml'
    with pytest.raises(ValueError, match='absolute'):
        select_bringup_config(model, 'x.yaml', _lookup)
    with pytest.raises(ValueError, match='launch model'):
        select_bringup_config(CONFIGS[1].stem, str(CONFIGS[0]), _lookup)


@pytest.mark.parametrize('path', sorted((SRC / 'lookie_description' / 'urdf').glob('*.urdf')),
                         ids=lambda p: p.stem)
def test_urdf_root_is_base_footprint(path):
    root = ET.parse(path).getroot()
    children = {j.find('child').get('link') for j in root.findall('joint')}
    assert [l.get('name') for l in root.findall('link') if l.get('name') not in children] == ['base_footprint']
