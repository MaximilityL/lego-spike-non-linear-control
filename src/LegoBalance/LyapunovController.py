"""Backward-compatible alias for :mod:`LegoBalance.NonLinearController`.

``NonLinearController`` is now the canonical name for the future balancing
controller. This module remains only so older imports keep working while the
repo transitions.
"""

from __future__ import annotations

from LegoBalance.NonLinearController import NonLinearController


class LyapunovController(NonLinearController):
    """Backward-compatible alias for :class:`NonLinearController`."""

