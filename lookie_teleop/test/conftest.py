"""Import the packages from this checkout, not from a possibly stale colcon install."""

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
for _pkg in ('lookie_teleop',):
    sys.path.insert(0, str(_ROOT / _pkg))
