"""RunDiagnostics.py

Run :class:`LegoBalance.ConnectionDiagnostics.ConnectionDiagnostics` against
the desktop side mock hub. Useful as a sanity check that the diagnostics
runner itself is wired correctly. Replace ``MockHub`` with a real hub
adapter once you have one.

Usage:

    python scripts/RunDiagnostics.py
"""

from __future__ import annotations

import sys

from LegoBalance.ConnectionDiagnostics import ConnectionDiagnostics
from LegoBalance.MockAdapters import MockHub


def Main() -> int:
    hub = MockHub()
    diagnostics = ConnectionDiagnostics(hub)
    report = diagnostics.Run()
    print("LegoBalance: connection diagnostics")
    print("-" * 60)
    print(report.Summary())
    return 0 if report.Passed else 1


if __name__ == "__main__":
    sys.exit(Main())
