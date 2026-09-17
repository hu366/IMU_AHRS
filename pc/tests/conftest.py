from __future__ import annotations

import sys
from pathlib import Path

_PC_ROOT = Path(__file__).resolve().parents[1]
if str(_PC_ROOT) not in sys.path:
    sys.path.insert(0, str(_PC_ROOT))
