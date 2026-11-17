#!/usr/bin/env python3
"""Report workspace packages whose colcon install no longer matches the source tree.

Python modules, launch files and config/URDF files are compared by content; compiled
sources are compared by modification time against the package's last colcon build.
Content differences mark a package STALE; newer compiled sources only mark it CHECK, since
a git checkout or copy also updates modification times. Exit code 1 for either, or when a
package is not built.

  scripts/check_install.py                      # every package under <workspace>/src
  scripts/check_install.py lookie_bringup kstone_ros
  scripts/check_install.py --workspace ~/robo_backup
"""

import argparse
import filecmp
import os
import sys
import sysconfig
import xml.etree.ElementTree as ET
from pathlib import Path

DATA_DIRS = ('launch', 'config', 'urdf', 'rviz', 'params', 'meshes')
DATA_SUFFIXES = {'.py', '.yaml', '.yml', '.urdf', '.xacro', '.rviz', '.json', '.xml', '.stl', '.dae', '.obj'}
COMPILED_SUFFIXES = {'.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx', '.msg', '.srv', '.action'}
BUILD_FILES = {'CMakeLists.txt', 'setup.py', 'setup.cfg', 'package.xml'}
SKIP_DIRS = {'.git', '__pycache__', 'build', 'install', 'log', 'test', 'tests', '.pytest_cache'}
MAX_LISTED = 8


def find_workspace(start: Path) -> Path:
    for path in [start, *start.parents]:
        if (path / 'src').is_dir() and (path / 'install').is_dir():
            return path
    raise SystemExit(f'no workspace (a directory with src/ and install/) above {start}; use --workspace')


def source_packages(src: Path) -> dict:
    found = {}
    for root, dirs, files in os.walk(src):
        if 'COLCON_IGNORE' in files:
            dirs[:] = []
            continue
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS and not d.startswith('.'))
        if 'package.xml' in files:
            xml = ET.parse(Path(root) / 'package.xml').getroot()
            name = (xml.findtext('name') or '').strip()
            build_type = (xml.findtext('export/build_type') or 'ament_cmake').strip()
            if name:
                found.setdefault(name, (Path(root), build_type))
            dirs[:] = []
    return found


def _files(base: Path, suffixes):
    for root, dirs, files in os.walk(base):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            path = Path(root) / name
            if path.suffix in suffixes or name in BUILD_FILES:
                yield path


def _site_packages(prefix: Path):
    rel = Path(sysconfig.get_path('purelib')).relative_to(sysconfig.get_path('data'))
    candidates = [prefix / rel, *prefix.glob('lib/python3*/site-packages'),
                  *prefix.glob('local/lib/python3*/dist-packages')]
    return next((p for p in candidates if p.is_dir()), None)


def check(name: str, src: Path, build_type: str, ws: Path) -> list:
    prefix = ws / 'install' / name
    marker = ws / 'build' / name / 'colcon_build.rc'
    if not prefix.is_dir():
        return ['not built (no install/{})'.format(name)]
    problems = []
    share = prefix / 'share' / name

    for top in DATA_DIRS:
        if not (src / top).is_dir() or not (share / top).is_dir():
            continue
        for path in _files(src / top, DATA_SUFFIXES):
            if path.name in BUILD_FILES:
                continue
            installed = share / path.relative_to(src)
            if not installed.exists():
                problems.append(f'not installed: {path.relative_to(src)}')
            elif not installed.is_symlink() and not filecmp.cmp(path, installed, shallow=False):
                problems.append(f'differs: {path.relative_to(src)}')

    if build_type == 'ament_python':
        site = _site_packages(prefix)
        module_dir = src / name
        if site and module_dir.is_dir():
            for path in _files(module_dir, {'.py'}):
                installed = site / path.relative_to(src)
                if not installed.exists():
                    problems.append(f'not installed: {path.relative_to(src)}')
                elif not installed.is_symlink() and not filecmp.cmp(path, installed, shallow=False):
                    problems.append(f'differs: {path.relative_to(src)}')

    if marker.is_file():
        built = marker.stat().st_mtime
        suffixes = COMPILED_SUFFIXES if build_type != 'ament_python' else set()
        for path in _files(src, suffixes):
            if path.parts[len(src.parts)] in DATA_DIRS and path.name not in BUILD_FILES:
                continue
            if path.stat().st_mtime > built:
                problems.append(f'newer than last build: {path.relative_to(src)}')
    return problems


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('packages', nargs='*', help='package names (default: all under src/)')
    parser.add_argument('--workspace', type=Path, help='colcon workspace (default: found above this script)')
    parser.add_argument('-v', '--verbose', action='store_true', help='list every difference')
    args = parser.parse_args(argv)

    ws = (args.workspace or find_workspace(Path(__file__).resolve())).resolve()
    packages = source_packages(ws / 'src')
    unknown = sorted(set(args.packages) - set(packages))
    if unknown:
        parser.error(f'unknown packages: {unknown}')
    stale = 0
    for name in sorted(args.packages or packages):
        src, build_type = packages[name]
        problems = check(name, src, build_type, ws)
        if not problems:
            print(f'OK     {name}')
            continue
        stale += 1
        definite = any(not p.startswith('newer than last build') for p in problems)
        print(f'{"STALE " if definite else "CHECK "} {name}')
        shown = problems if args.verbose else problems[:MAX_LISTED]
        for line in shown:
            print(f'         {line}')
        if len(problems) > len(shown):
            print(f'         ... {len(problems) - len(shown)} more (-v lists all)')
    if stale:
        print(f'\n{stale} package(s) out of date: colcon build --packages-select <name> ...', file=sys.stderr)
    return 1 if stale else 0


if __name__ == '__main__':
    sys.exit(main())
