"""Per-package .repos files must copy lookie.repos entries and together cover it."""

import re
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


def _requirements(path: Path) -> dict:
    """``{name: requirement}`` for requirements.txt-style lines."""
    try:
        from packaging.requirements import Requirement
    except ImportError:
        from pip._vendor.packaging.requirements import Requirement
    lines = (re.sub(r'(^|\s)#.*$', '', l).strip() for l in path.read_text(encoding='utf-8').splitlines())
    reqs = [Requirement(l) for l in lines if l]
    return {r.name: r for r in reqs}


def test_python_lock_pins_every_requirement():
    """requirements.lock.txt must pin exactly the SDKs in requirements.txt, from the same sources."""
    loose = _requirements(ROOT / 'requirements.txt')
    locked = _requirements(ROOT / 'requirements.lock.txt')
    assert set(locked) == set(loose), 'regenerate: python3 scripts/lock_python.py'
    for name, req in loose.items():
        pin = locked[name]
        if req.url:
            base, _, commit = pin.url.rpartition('@')
            assert base == req.url, f'{name}: lock source {base} differs from {req.url}'
            assert re.fullmatch(r'[0-9a-f]{40}', commit), f'{name}: not pinned to a commit: {pin}'
        elif req.specifier:
            assert pin.specifier == req.specifier, f'{name}: constraint changed, regenerate the lock'
        else:
            assert len(pin.specifier) == 1 and next(iter(pin.specifier)).operator == '==', \
                f'{name}: not pinned to an exact version: {pin}'
