from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SimDirtDot:
    name: str
    x_cm: float
    y_cm: float


DEFAULT_SIM_DIRT_DOTS: tuple[SimDirtDot, ...] = (
    SimDirtDot("sim_dirt_dot_00", 10.71, 131.14),
    SimDirtDot("sim_dirt_dot_01", 20.67, 22.76),
    SimDirtDot("sim_dirt_dot_02", 48.91, 56.89),
    SimDirtDot("sim_dirt_dot_03", 16.42, 140.77),
    SimDirtDot("sim_dirt_dot_04", 23.05, 41.85),
    SimDirtDot("sim_dirt_dot_05", 13.34, 142.20),
    SimDirtDot("sim_dirt_dot_06", 8.88, 62.52),
    SimDirtDot("sim_dirt_dot_07", 44.86, 148.85),
    SimDirtDot("sim_dirt_dot_08", 72.76, 37.84),
    SimDirtDot("sim_dirt_dot_09", 24.30, 16.92),
    SimDirtDot("sim_dirt_dot_10", 51.72, 14.69),
    SimDirtDot("sim_dirt_dot_11", 19.39, 73.53),
    SimDirtDot("sim_dirt_dot_12", 63.43, 51.31),
    SimDirtDot("sim_dirt_dot_13", 58.10, 130.09),
    SimDirtDot("sim_dirt_dot_14", 62.35, 98.65),
    SimDirtDot("sim_dirt_dot_15", 36.52, 32.10),
    SimDirtDot("sim_dirt_dot_16", 6.86, 102.80),
    SimDirtDot("sim_dirt_dot_17", 26.02, 120.99),
)


def dirt_under_robot_footprint(
    robot_x_cm: float,
    robot_y_cm: float,
    dots: list[SimDirtDot] | tuple[SimDirtDot, ...],
    half_width_cm: float,
    half_height_cm: float,
) -> list[SimDirtDot]:
    return [
        dot
        for dot in dots
        if abs(dot.x_cm - robot_x_cm) <= half_width_cm and abs(dot.y_cm - robot_y_cm) <= half_height_cm
    ]
