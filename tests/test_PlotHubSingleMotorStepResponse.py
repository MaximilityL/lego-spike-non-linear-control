from __future__ import annotations

import math
from io import StringIO

from scripts.PlotHubSingleMotorStepResponse import (
    CollectSamples,
    ContiguousPhaseSpans,
    ParseStepResponseLine,
)


def test_ParseStepResponseLineParsesTelemetryRow():
    sample = ParseStepResponseLine(
        "DATA,velocity,step,1.250,nan,+42.000,+360.000,+355.000"
    )
    assert sample is not None
    assert sample.phase == "velocity"
    assert sample.segment == "step"
    assert sample.t_s == 1.25
    assert math.isnan(sample.angle_ref_deg)
    assert sample.angle_meas_deg == 42.0
    assert sample.speed_ref_deg_per_sec == 360.0
    assert sample.speed_meas_deg_per_sec == 355.0


def test_CollectSamplesIgnoresNonTelemetryLines():
    stream = StringIO(
        "banner line\n"
        "DATA,velocity,pre,0.000,nan,+0.000,+0.000,+0.000\n"
        "DATA,position,step,3.000,+180.000,+120.000,nan,+90.000\n"
    )
    samples = CollectSamples(stream)
    assert len(samples) == 2
    assert samples[0].phase == "velocity"
    assert samples[1].phase == "position"
    assert math.isnan(samples[1].speed_ref_deg_per_sec)


def test_ContiguousPhaseSpansGroupsSamplesByPhase():
    stream = StringIO(
        "DATA,velocity,pre,0.000,nan,+0.000,+0.000,+0.000\n"
        "DATA,velocity,step,1.000,nan,+10.000,+360.000,+350.000\n"
        "DATA,position,pre,2.000,+0.000,+0.000,nan,+0.000\n"
        "DATA,position,step,3.000,+180.000,+160.000,nan,+50.000\n"
    )
    spans = ContiguousPhaseSpans(CollectSamples(stream))
    assert spans == [
        ("velocity", 0.0, 1.0),
        ("position", 2.0, 3.0),
    ]
