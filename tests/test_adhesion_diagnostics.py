import pytest

from window_cleaner.adhesion_diagnostics import summarize_samples
from window_cleaner.arduino_imu import parse_sensor_line


def sample(line):
    parsed = parse_sensor_line(line)
    assert parsed is not None
    return parsed


def test_summarize_pressure_samples():
    summary = summarize_samples([
        sample("SENSOR,1000,500,1,0,0,0,220,0"),
        sample("SENSOR,1050,620,1,0,0,0,240,1"),
        sample("SENSOR,1100,680,1,0,0,0,255,1"),
    ])

    assert summary.count == 3
    assert summary.pressure_min == 500
    assert summary.pressure_max == 680
    assert summary.pressure_mean == pytest.approx(600.0)
    assert summary.blower_min == 220
    assert summary.blower_max == 255
    assert summary.adhesion_secure_ratio == pytest.approx(2 / 3)


def test_summarize_requires_samples():
    with pytest.raises(ValueError):
        summarize_samples([])
