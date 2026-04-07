"""DataLogger.

A small in memory logger that buffers samples and writes them out as CSV
when asked. Hub side scripts cannot use this directly because Pybricks does
not have a normal filesystem. The hub side equivalent is just printing every
Nth iteration to the Pybricks Code terminal. The CSV writer here is for the
desktop side simulation and for offline analysis of any data you stream
back.
"""

from __future__ import annotations

import csv
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .BalanceState import BalanceState
from .ControlInterfaces import ControlOutput


@dataclass
class LogRecord:
    """One row of the log."""

    timestamp: float
    state: BalanceState
    controlOutput: ControlOutput | None = None
    extras: dict[str, Any] = field(default_factory=dict)

    def AsRow(self) -> dict[str, Any]:
        """Flatten the record into a CSV friendly dict."""
        row: dict[str, Any] = {
            "timestamp": self.timestamp,
            "tilt": self.state.tilt,
            "tiltRate": self.state.tiltRate,
            "wheelPosition": self.state.wheelPosition,
            "wheelVelocity": self.state.wheelVelocity,
            "stateValid": self.state.valid,
        }
        if self.controlOutput is not None:
            row["leftCommand"] = self.controlOutput.leftCommand
            row["rightCommand"] = self.controlOutput.rightCommand
            row["mode"] = self.controlOutput.mode.value
        else:
            row["leftCommand"] = ""
            row["rightCommand"] = ""
            row["mode"] = ""
        row.update(self.extras)
        return row


class DataLogger:
    """Bounded ring buffer plus CSV export."""

    def __init__(self, bufferSize: int = 2048) -> None:
        if bufferSize <= 0:
            raise ValueError("bufferSize must be positive")
        self._bufferSize = bufferSize
        self._records: list[LogRecord] = []

    def Record(
        self,
        state: BalanceState,
        controlOutput: ControlOutput | None = None,
        **extras: Any,
    ) -> None:
        """Append one record. Drops the oldest if the buffer is full."""
        record = LogRecord(
            timestamp=state.timestamp,
            state=state.Copy(),
            controlOutput=controlOutput,
            extras=dict(extras),
        )
        self._records.append(record)
        if len(self._records) > self._bufferSize:
            self._records.pop(0)

    def Records(self) -> list[LogRecord]:
        """Return a shallow copy of the current records."""
        return list(self._records)

    def Clear(self) -> None:
        self._records.clear()

    def WriteCsv(self, path: Path) -> None:
        """Write all buffered records to ``path``.

        Creates the parent directory if needed. The header is the union of
        keys present across the records, with deterministic ordering.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not self._records:
            path.write_text("")
            return

        # Compute deterministic header.
        baseFields = [
            "timestamp",
            "tilt",
            "tiltRate",
            "wheelPosition",
            "wheelVelocity",
            "stateValid",
            "leftCommand",
            "rightCommand",
            "mode",
        ]
        extraKeys: list[str] = []
        for record in self._records:
            for key in record.extras.keys():
                if key not in extraKeys:
                    extraKeys.append(key)
        fieldnames = baseFields + extraKeys

        with path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for record in self._records:
                writer.writerow(record.AsRow())

    def __len__(self) -> int:
        return len(self._records)

    def __iter__(self) -> Iterable[LogRecord]:
        return iter(self._records)
