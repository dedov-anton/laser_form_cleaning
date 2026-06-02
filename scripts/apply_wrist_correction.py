#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.transforms.wrist_correction_export import (
    format_wrist_correction_summary,
    run_cylinder_wrist_correction,
)


def main() -> int:
    try:
        outputs = run_cylinder_wrist_correction(PROJECT_ROOT)
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(error, file=sys.stderr)
        return 1

    print(format_wrist_correction_summary(outputs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
