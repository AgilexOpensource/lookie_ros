"""Per-package .repos files must copy lookie.repos entries and together cover it."""

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


def _repos(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))['repositories']


def test_package_repos_cover_full_list():
    full = _repos(ROOT / 'lookie.repos')
    paths = sorted(ROOT.glob('*/*.repos'))
    seen = set()
    for path in paths:
        assert path.stem == path.parent.name, f'{path} must be named after its package'
        for name, entry in _repos(path).items():
            assert full.get(name) == entry, f'{path.name}: {name} differs from lookie.repos'
            seen.add(name)
    assert seen == set(full)
    assert {p.stem for p in paths} == {'lookie_bringup', 'lookie_driver', 'lookie_control', 'lookie_teleop'}
